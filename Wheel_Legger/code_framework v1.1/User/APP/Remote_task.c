#include "Remote_task.h"
#include "Remote_Control.h"
#include "chassis_task.h"

/* ==================== 日常修改区：操控参数 ==================== */
#define REMOTE_FORWARD_MAX_MPS 2.0f  /* 前进摇杆满杆速度，单位 m/s */
#define REMOTE_YAW_MAX_RADPS   3.0f  /* 转向摇杆满杆角速度，单位 rad/s */
#define REMOTE_SPIN_YAW_RADPS  6.0f  /* 小陀螺固定角速度，单位 rad/s；修改正负号可反向 */
/* Leg length presets are shared with the balance ramp in chassis_task.h. */

typedef enum
{
    REMOTE_MODE_INVALID = 0,
    REMOTE_MODE_DISABLE,
    REMOTE_MODE_ENABLE
} Remote_ModeRequest_e;

/*
 * 1. 模式操作：更换使能拨杆或按键，修改本函数的判断条件和返回请求。
 *    当前 S2 下档主动关闭，上档/中档请求使能，零值/其他值为无效操作。
 *    主动关闭请求也用于上电及两种来源复联后的重新使能；无效操作不能解除复联锁定。
 *
 *    chassis_enable 是使能开关，chassis_mode 是底盘实际运行阶段：
 *    OFFLINE -> ONLINE 后，底盘自动执行 FALLING_DOWN -> FALLING_TO_NORMAL -> NORMAL。
 *    倒地或自起失败进入 ZERO_FORCE，关闭后重新使能才重新启动自起。
 *    实际模式转换见 chassis_task.c 的 falling_down_detect() 和 chassis_recovery_control()。
 */
static Remote_ModeRequest_e Remote_MapModeRequest(const RC_Ctl_t *input)
{
    switch (input->rc.s2)  /* 修改这里可更换模式操作使用的拨杆或按键。 */
    {
        case RC_SW_DOWN:
            return REMOTE_MODE_DISABLE;
        case RC_SW_UP:
        case RC_SW_MID:
            return REMOTE_MODE_ENABLE;
        default:
            return REMOTE_MODE_INVALID;
    }
}

/* NORMAL 内任务选择：S2 上档请求小陀螺，中档恢复正常运动；iw 不参与控制。
 * 只生成请求；实际状态、自起及故障处理仍由底盘任务维护。
 */
static Chassis_Normal_State_e Remote_MapNormalState(const RC_Ctl_t *input)
{
    if (input->rc.s2 == RC_SW_UP)
    {
        return NORMAL_SPIN;
    }
    return NORMAL_MOVE;
}

/*
 * 2. 摇杆映射：更换通道改 ch3/ch0，反向改正负号，满杆速度改文件顶部参数。
 *    ch3：前进/后退，正值前进；正常运动由 ch0 转向，沿用原来的负号方向。
 *    小陀螺使用文件顶部的固定角速度，不读取 iw 或 ch0；正负号决定 yaw 目标方向。
 *    RC_RESOLUTION = 660，除以它将遥控通道换算为满杆比例。
 */
static void Remote_MapJoystick(const RC_Ctl_t *input, Chassis_Normal_State_e normal_state,
                               float *v_set, float *yaw_set_v)
{
    *v_set = (float)input->rc.ch3 / RC_RESOLUTION * REMOTE_FORWARD_MAX_MPS;
    if (normal_state == NORMAL_SPIN)
    {
        *yaw_set_v = REMOTE_SPIN_YAW_RADPS;
    }
    else
    {
        *yaw_set_v = -(float)input->rc.ch0 / RC_RESOLUTION * REMOTE_YAW_MAX_RADPS;
    }
}

/*
 * 3. 腿长映射：更换操作改 s1 和各 case，腿长数值改 chassis_task.h 中的 LEG_LENGTH_*_M 参数，单位 m。
 *    仅在输入有效且底盘为 NORMAL 时应用；自起阶段的腿长由自起流程控制。
 *    返回 false 表示不更新目标，保留左右腿已有的目标腿长。
 */
static bool Remote_MapLegLength(const RC_Ctl_t *input, float *length_m)
{
    switch (input->rc.s1)
    {
        case 0U:
        case RC_SW_DOWN:
            *length_m = LEG_LENGTH_SHORT_M;
            return true;
        case RC_SW_MID:
            *length_m = LEG_LENGTH_MIDDLE_M;
            return true;
        case RC_SW_UP:
            *length_m = LEG_LENGTH_LONG_M;
            return true;
        default:
            return false;
    }
}

/* ==================== 任务流程：按顺序读取即可 ==================== */
static uint8_t Remote_CheckEnable(uint8_t valid, uint32_t loss_seq, Remote_ModeRequest_e request);
static void Remote_ApplyControl(const RC_Ctl_t *input, uint8_t valid, uint32_t rx_tick,
                                uint8_t enabled, Chassis_Normal_State_e normal_state,
                                float v_set, float yaw_set_v);

void Remote_ControlStep(void)
{
    RC_Ctl_t input;
    uint32_t rx_tick;
    uint32_t loss_seq;
    uint8_t valid;
    Remote_ModeRequest_e request;
    Chassis_Normal_State_e normal_state;
    uint8_t enabled;
    float v_set;
    float yaw_set_v;

    valid = (uint8_t)Remote_GetSnapshot(&input, &rx_tick, &loss_seq); /* 取选中来源 */
    request = Remote_MapModeRequest(&input);
    enabled = Remote_CheckEnable(valid, loss_seq, request);        /* 判断使能 */
    normal_state = Remote_MapNormalState(&input);
    Remote_MapJoystick(&input, normal_state, &v_set, &yaw_set_v);   /* 映射操作 */
    Remote_ApplyControl(&input, valid, rx_tick, enabled, normal_state, v_set, yaw_set_v);
}

void remote_task(void)
{
    uint32_t previous_wake_time = osKernelSysTick();
    for (;;)
    {
        Remote_ControlStep();
        osDelayUntil(&previous_wake_time, REMOTE_TASK_PERIOD_MS);
    }
}

/* ==================== 内部处理区：输入、保护、记录 ==================== */
/* 启动诊断变量由 chassis_task.c 保存，仅记录选中来源的使能事件。 */
extern volatile uint32_t recovery_dbg_rx_on_tick;
extern volatile uint32_t recovery_dbg_rx_on_seq;
extern volatile uint32_t recovery_dbg_task_on_tick;
extern volatile uint32_t recovery_dbg_task_on_seq;
extern volatile uint32_t recovery_dbg_first_call_delay_ms;
extern volatile uint32_t recovery_dbg_reference_delay_ms;
extern volatile uint32_t recovery_dbg_first_result;
extern volatile uint32_t recovery_dbg_first_phase[2];
extern volatile float recovery_dbg_first_phi0[2];
extern volatile uint32_t recovery_dbg_abort_tick;
extern volatile uint32_t recovery_dbg_abort_count;

static uint8_t remote_was_enabled;
/* 两路共用：上电、失联或无效操作后为 1；有效主动关闭才清零，供 Watch 观察。 */
volatile uint8_t remote_rearm_required = 1U;
static uint32_t remote_seen_loss_seq;

static uint8_t Remote_CheckEnable(uint8_t valid, uint32_t loss_seq, Remote_ModeRequest_e request)
{
    if (valid == 0U || request == REMOTE_MODE_INVALID || loss_seq != remote_seen_loss_seq)
    {
        remote_rearm_required = 1U;
    }
    remote_seen_loss_seq = loss_seq;
    /* 使用前部的主动关闭请求；无效操作和失联前保存的下档均不能解锁。 */
    if (valid != 0U && request == REMOTE_MODE_DISABLE)
    {
        remote_rearm_required = 0U;
    }
    return valid != 0U && remote_rearm_required == 0U && request == REMOTE_MODE_ENABLE;
}

static void Remote_RecordEnable(uint32_t rx_tick)
{
    recovery_dbg_rx_on_tick = rx_tick;
    recovery_dbg_first_call_delay_ms = UINT32_MAX;
    recovery_dbg_reference_delay_ms = UINT32_MAX;
    recovery_dbg_first_result = UINT32_MAX;
    recovery_dbg_first_phase[0] = UINT32_MAX;
    recovery_dbg_first_phase[1] = UINT32_MAX;
    recovery_dbg_first_phi0[0] = 0.0f;
    recovery_dbg_first_phi0[1] = 0.0f;
    recovery_dbg_abort_tick = UINT32_MAX;
    recovery_dbg_abort_count = 0U;
    ++recovery_dbg_rx_on_seq;
    recovery_dbg_task_on_tick = HAL_GetTick();
    recovery_dbg_task_on_seq = recovery_dbg_rx_on_seq;
}

static void Remote_ApplyControl(const RC_Ctl_t *input, uint8_t valid, uint32_t rx_tick,
                                uint8_t enabled, Chassis_Normal_State_e normal_state,
                                float v_set, float yaw_set_v)
{
    float length_m;

    if (enabled != 0U)
    {
        if (remote_was_enabled == 0U)
        {
            Remote_RecordEnable(rx_tick);
        }
        Chassis.set_goal.v_set = v_set;
        Chassis.set_goal.yaw_set_v = yaw_set_v;
        Chassis.normal_state_request = Chassis.chassis_mode == NORMAL ? normal_state : NORMAL_MOVE;
        Chassis.chassis_enable = ONLINE;
    }
    else
    {
        Chassis.chassis_enable = OFFLINE;
        Chassis.set_goal.v_set = 0.0f;
        Chassis.set_goal.yaw_set_v = 0.0f;
        Chassis.normal_state_request = NORMAL_MOVE;
    }
    remote_was_enabled = enabled;

    if (valid != 0U && Chassis.chassis_mode == NORMAL &&
        Remote_MapLegLength(input, &length_m))
    {
        Chassis.set_goal.set_L0_Left = length_m;
        Chassis.set_goal.set_L0_Right = length_m;
    }
}
