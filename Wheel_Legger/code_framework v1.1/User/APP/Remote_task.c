#include "Remote_task.h"
#include "Remote_Control.h"
#include "board_comm_task.h"
#include "chassis_task.h"
#include "user_sys_config.h"

/* ==================== 日常修改区：操控参数 ==================== */
#define REMOTE_FORWARD_MAX_MPS 2.0f  /* 前进摇杆满杆速度，单位 m/s */
#define REMOTE_YAW_MAX_RADPS   3.0f  /* 转向摇杆满杆角速度，单位 rad/s */
#define REMOTE_LEG_SHORT_M     0.159f
#define REMOTE_LEG_MIDDLE_M    0.20f
#define REMOTE_LEG_LONG_M      0.25f

typedef enum
{
    REMOTE_MODE_INVALID = 0,
    REMOTE_MODE_DISABLE,
    REMOTE_MODE_ENABLE
} Remote_ModeRequest_e;

/*
 * 1. 模式操作：更换使能拨杆或按键，修改本函数的判断条件和返回请求。
 *    当前 S2 下档主动关闭，上档/中档请求使能，零值/其他值为无效操作。
 *    主动关闭请求也用于云台复联后的重新使能；无效操作不能解除复联锁定。
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

/*
 * 2. 摇杆映射：更换通道改 ch3/ch0，反向改正负号，满杆速度改文件顶部参数。
 *    ch3：前进/后退，正值前进；ch0：转向，沿用原来的负号方向。
 *    RC_RESOLUTION = 660，除以它将遥控通道换算为满杆比例。
 */
static void Remote_MapJoystick(const RC_Ctl_t *input, float *v_set, float *yaw_set_v)
{
    *v_set = (float)input->rc.ch3 / RC_RESOLUTION * REMOTE_FORWARD_MAX_MPS;
    *yaw_set_v = -(float)input->rc.ch0 / RC_RESOLUTION * REMOTE_YAW_MAX_RADPS;
}

/*
 * 3. 腿长映射：更换操作改 s1 和各 case，腿长数值改文件顶部参数，单位 m。
 *    仅在输入有效且底盘为 NORMAL 时应用；自起阶段的腿长由自起流程控制。
 *    返回 false 表示不更新目标，保留左右腿已有的目标腿长。
 */
static bool Remote_MapLegLength(const RC_Ctl_t *input, float *length_m)
{
    switch (input->rc.s1)
    {
        case 0U:
        case RC_SW_DOWN:
            *length_m = REMOTE_LEG_SHORT_M;
            return true;
        case RC_SW_MID:
            *length_m = REMOTE_LEG_MIDDLE_M;
            return true;
        case RC_SW_UP:
            *length_m = REMOTE_LEG_LONG_M;
            return true;
        default:
            return false;
    }
}

/* ==================== 任务流程：按顺序读取即可 ==================== */
typedef struct
{
    RC_Ctl_t remote;
    uint32_t rx_tick;
    uint32_t loss_seq;
    uint8_t valid;
} Remote_Input_t;

static void Remote_ReadInput(Remote_Input_t *input);
static uint8_t Remote_CheckEnable(const Remote_Input_t *input, Remote_ModeRequest_e request);
static void Remote_ApplyControl(const Remote_Input_t *input, uint8_t enabled,
                                float v_set, float yaw_set_v);

void Remote_ControlStep(void)
{
    Remote_Input_t input;
    Remote_ModeRequest_e request;
    uint8_t enabled;
    float v_set;
    float yaw_set_v;

    Remote_ReadInput(&input);                                     /* 读取选中来源 */
    request = Remote_MapModeRequest(&input.remote);               /* 解析模式操作 */
    enabled = Remote_CheckEnable(&input, request);                /* 失联/复联保护 */
    Remote_MapJoystick(&input.remote, &v_set, &yaw_set_v);         /* 映射摇杆目标 */
    Remote_ApplyControl(&input, enabled, v_set, yaw_set_v);        /* 应用底盘目标 */
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
volatile uint8_t remote_gimbal_rearm_required;
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
static uint8_t remote_board_seen_valid;
static uint32_t remote_board_loss_seq;
#endif

static void Remote_ReadInput(Remote_Input_t *input)
{
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    BoardComm_Remote_t board;
    input->valid = (uint8_t)BoardComm_GetRemoteSnapshot(&board);
    input->remote = board.remote;
    input->rx_tick = board.rx_tick[1];
    input->loss_seq = board.loss_seq;
#else
    input->valid = (uint8_t)Remote_GetSnapshot(&input->remote, &input->rx_tick);
    input->loss_seq = 0U;
#endif
}

static uint8_t Remote_CheckEnable(const Remote_Input_t *input, Remote_ModeRequest_e request)
{
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    if (input->loss_seq != remote_board_loss_seq)
    {
        remote_gimbal_rearm_required = 1U;
        remote_board_loss_seq = input->loss_seq;
    }
    if (input->valid == 0U && remote_board_seen_valid != 0U)
    {
        remote_gimbal_rearm_required = 1U;
    }
    if (input->valid != 0U)
    {
        remote_board_seen_valid = 1U;
        /* 只接受映射区的主动关闭请求，零值/无效操作和失联输入不能解锁。 */
        if (request == REMOTE_MODE_DISABLE)
        {
            remote_gimbal_rearm_required = 0U;
        }
    }
    return input->valid != 0U && remote_gimbal_rearm_required == 0U &&
           request == REMOTE_MODE_ENABLE;
#else
    return input->valid != 0U && request == REMOTE_MODE_ENABLE;
#endif
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

static void Remote_ApplyControl(const Remote_Input_t *input, uint8_t enabled,
                                float v_set, float yaw_set_v)
{
    float length_m;

    if (enabled != 0U)
    {
        if (remote_was_enabled == 0U)
        {
            Remote_RecordEnable(input->rx_tick);
        }
        Chassis.set_goal.v_set = v_set;
        Chassis.set_goal.yaw_set_v = yaw_set_v;
        Chassis.chassis_enable = ONLINE;
    }
    else
    {
        Chassis.chassis_enable = OFFLINE;
        Chassis.set_goal.v_set = 0.0f;
        Chassis.set_goal.yaw_set_v = 0.0f;
    }
    remote_was_enabled = enabled;

    if (input->valid != 0U && Chassis.chassis_mode == NORMAL &&
        Remote_MapLegLength(&input->remote, &length_m))
    {
        Chassis.set_goal.set_L0_Left = length_m;
        Chassis.set_goal.set_L0_Right = length_m;
    }
}
