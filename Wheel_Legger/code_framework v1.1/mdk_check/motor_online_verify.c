#include "chassis_task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

Chassis_Info_Typedef Chassis;
FDCAN_HandleTypeDef hfdcan1 = {FDCAN1}, hfdcan2 = {FDCAN2}, hfdcan3 = {FDCAN3};
FDCAN_RxHeaderTypeDef RxHeader1, RxHeader2, RxHeader3;
uint8_t g_Can1RxData[64], g_Can2RxData[64], g_Can3RxData[64];
static uint32_t now, interrupt_mask;
static unsigned int checks, failures, wheel_sends, joint_sends, leso_calls;
static unsigned int enable_sends, enable_delays, enable_per_joint[4];
static int enable_refresh_feedback;
static uint8_t feedback_state[4];
static void RefreshExcept(int omitted);
static uint8_t wheel_frame[8];
static float sent_torque[4];
static int pending, rx_fail, inject_before_mask;
static FDCAN_HandleTypeDef *rx_can;
static uint32_t rx_fifo;
static FDCAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8];
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; fprintf(stderr, "MOTOR FAIL line %d: %s\n", __LINE__, #c); } } while (0)
#define NEAR(a,b) CHECK(fabsf((a) - (b)) < 0.00001f)

uint32_t HAL_GetTick(void) { return now; }
uint32_t __get_PRIMASK(void) { return interrupt_mask; }
void __disable_irq(void)
{
    if (inject_before_mask)
    {
        unsigned int motor;
        inject_before_mask = 0;
        ++now;
        for (motor = 0; motor < CHASSIS_MOTOR_COUNT; ++motor)
            Chassis_MotorFeedbackReceived((Chassis_MotorFeedback_e)motor);
    }
    interrupt_mask = 1U;
}
void __set_PRIMASK(uint32_t mask) { interrupt_mask = mask; }
void osDelay(uint32_t delay)
{
    CHECK(delay == 1U && interrupt_mask == 0U);
    now += delay;
    ++enable_delays;
    if (enable_refresh_feedback) RefreshExcept(-1);
}
int HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *can, uint32_t fifo,
                          FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    CHECK(can == rx_can && fifo == rx_fifo);
    if (!pending || rx_fail) return HAL_ERROR;
    *header = rx_header;
    memcpy(data, rx_data, sizeof(rx_data));
    pending = 0;
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *can, uint32_t fifo)
{
    return can == rx_can && fifo == rx_fifo && pending ? 1U : 0U;
}
uint8_t canx_send_data(hcan_t *can, uint16_t id, uint8_t *data, uint32_t len)
{
    unsigned int motor;
    CHECK(len == 8U);
    if (id == Chassis_3508_MotorA_TxID)
    {
        CHECK(can == &hfdcan1);
        memcpy(wheel_frame, data, sizeof(wheel_frame));
        ++wheel_sends;
        return 0;
    }
    CHECK(id >= 1U && id <= 4U);
    if (id < 1U || id > 4U) return 0;
    motor = id - 1U;
    CHECK(can == (motor == LEFT_FRONT_id || motor == LEFT_BACK_id ? &hfdcan2 : &hfdcan1));
    for (unsigned int i = 0; i < 7; ++i) CHECK(data[i] == 0xFF);
    CHECK(data[7] == 0xFC);
    ++enable_sends;
    ++enable_per_joint[motor];
    return 0;
}
void DM_Motor_Ctrl(hcan_t *can, volatile DM_Motor_Info_Typedef *motor, float pos,
                   float vel, float kp, float kd, float torque, uint8_t delay)
{
    unsigned int index;
    CHECK(pos == 0 && vel == 0 && kp == 0 && kd == 0 && delay == 0);
    for (index = 0; index < 4; ++index)
    {
        if (motor == &Chassis.Joint_Motor[index])
        {
            CHECK(can == (index == LEFT_FRONT_id || index == LEFT_BACK_id ? &hfdcan2 : &hfdcan1));
            sent_torque[index] = torque;
            ++joint_sends;
            return;
        }
    }
    CHECK(0);
}
static void LESO_Feedback(float left, float right, float left_tp, float right_tp)
{
    (void)left; (void)right; (void)left_tp; (void)right_tp;
    ++leso_calls;
}
static void BoardComm_OnCanRx(const FDCAN_RxHeaderTypeDef *header, const uint8_t *data)
{ (void)header; (void)data; }
#include "motor_implementation.inc"

typedef struct
{
    FDCAN_HandleTypeDef *can;
    uint32_t fifo, id;
    Chassis_MotorFeedback_e slot;
} MotorRoute;
static const MotorRoute routes[] = {
    {&hfdcan2, FDCAN_RX_FIFO1, JOINT_DM_LEFT_FRONT_RxID_Set, CHASSIS_MOTOR_LEFT_FRONT},
    {&hfdcan2, FDCAN_RX_FIFO1, JOINT_DM_LEFT_BACK_RxID_Set, CHASSIS_MOTOR_LEFT_BACK},
    {&hfdcan1, FDCAN_RX_FIFO0, JOINT_DM_RIGHT_FRONT_RxID_Set, CHASSIS_MOTOR_RIGHT_FRONT},
    {&hfdcan1, FDCAN_RX_FIFO0, JOINT_DM_RIGHT_BACK_RxID_Set, CHASSIS_MOTOR_RIGHT_BACK},
    {&hfdcan1, FDCAN_RX_FIFO0, Chassis_3508_Motor2_RxID, CHASSIS_MOTOR_LEFT_WHEEL},
    {&hfdcan1, FDCAN_RX_FIFO0, Chassis_3508_Motor1_RxID, CHASSIS_MOTOR_RIGHT_WHEEL},
};
static void Reset(void)
{
    unsigned int motor;
    /* Finish the previous test's event through the real recovery predicate. */
    now = interrupt_mask = 0;
    for (motor = 0; motor < 4; ++motor)
    {
        motor_feedback_seen[motor] = 1U;
        motor_feedback_tick[motor] = now;
        Chassis.Joint_Motor[motor].Data.state = 1U;
    }
    chassis_joint_enable_check();
    memset(&Chassis, 0, sizeof(Chassis));
    memset((void *)motor_feedback_tick, 0, sizeof(motor_feedback_tick));
    memset((void *)motor_feedback_seen, 0, sizeof(motor_feedback_seen));
    now = interrupt_mask = 0;
    pending = rx_fail = inject_before_mask = 0;
    Chassis.chassis_enable = ONLINE;
    Chassis.chassis_mode = NORMAL;
    enable_sends = enable_delays = 0;
    enable_refresh_feedback = 0;
    memset(enable_per_joint, 0, sizeof(enable_per_joint));
    for (motor = 0; motor < 4; ++motor)
    {
        feedback_state[motor] = 1U;
        Chassis.Joint_Motor[motor].Motor_Type = DM_J8009;
        Chassis.Joint_Motor[motor].ID_Set.TxIdentifier = motor + 1U;
        Chassis.Joint_Motor[motor].Mode = Mit_mode;
    }
    for (motor = 0; motor < 2; ++motor)
    {
        Chassis.Wheel_Motor[motor].Motor_Type = DJI_M3508;
        Chassis.Wheel_Motor[motor].ID_Set.TxIdentifier = Chassis_3508_MotorA_TxID;
    }
}
/* bad: 1=DLC, 2=extended ID, 3=remote, 4=FD, 5=HAL error,
 * 6=unknown ID, 7=wrong bus, 8=no new-message interrupt. */
static void Receive(unsigned int motor, unsigned int bad)
{
    const MotorRoute *route = &routes[motor];
    rx_can = route->can;
    rx_fifo = route->fifo;
    rx_header = (FDCAN_RxHeaderTypeDef){route->id, FDCAN_STANDARD_ID,
                                    FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8, FDCAN_CLASSIC_CAN};
    memset(rx_data, 0, sizeof(rx_data));
    rx_data[0] = motor < 4 ? (uint8_t)((feedback_state[motor] << 4) | 1U) : 0x11;
    rx_data[2] = 1;
    pending = 1;
    rx_fail = bad == 5;
    if (bad == 1) rx_header.DataLength = 7U << 16;
    if (bad == 2) rx_header.IdType = FDCAN_EXTENDED_ID;
    if (bad == 3) rx_header.RxFrameType = FDCAN_REMOTE_FRAME;
    if (bad == 4) rx_header.FDFormat = FDCAN_FD_CAN;
    if (bad == 6) rx_header.Identifier = 0x777;
    if (bad == 7) { rx_can = &hfdcan3; rx_fifo = FDCAN_RX_FIFO0; }
    if (rx_fifo == FDCAN_RX_FIFO0)
        HAL_FDCAN_RxFifo0Callback(rx_can, bad == 8 ? 0 : FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    else
        HAL_FDCAN_RxFifo1Callback(rx_can, bad == 8 ? 0 : FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
    pending = rx_fail = 0;
}
static void RefreshExcept(int omitted)
{
    unsigned int motor;
    for (motor = 0; motor < CHASSIS_MOTOR_COUNT; ++motor)
        if ((int)motor != omitted) Receive(motor, 0);
}
static int16_t SentCurrent(unsigned int motor)
{
    return (int16_t)(((uint16_t)wheel_frame[motor * 2] << 8) | wheel_frame[motor * 2 + 1]);
}
static unsigned int SendCheck(uint8_t all_online, uint8_t motion)
{
    unsigned int index;
    Chassis_Enable_e enable = Chassis.chassis_enable;
    Chassis_Mode_e mode = Chassis.chassis_mode;
    unsigned int before_enables = enable_sends;
    uint32_t mask = interrupt_mask;
    Chassis.Wheel_Motor[LEFT_Wheel].wheel_T = 1.0f;
    Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T = 2.0f;
    Chassis.leg_situation[LEFT_Leg].vmc.torque_set[0] = 1.0f;
    Chassis.leg_situation[LEFT_Leg].vmc.torque_set[1] = 2.0f;
    Chassis.leg_situation[RIGHT_Leg].vmc.torque_set[0] = 3.0f;
    Chassis.leg_situation[RIGHT_Leg].vmc.torque_set[1] = 4.0f;
    wheel_sends = joint_sends = leso_calls = 0;
    for (index = 0; index < 4; ++index) sent_torque[index] = 99.0f;
    Chassis_CanTransimit();
    CHECK(Chassis.all_motors_online == all_online);
    CHECK(Chassis.chassis_enable == enable && Chassis.chassis_mode == mode);
    CHECK(interrupt_mask == mask);
    CHECK(wheel_sends == 1 && joint_sends == 4);
    CHECK(leso_calls == (unsigned int)motion);
    CHECK(SentCurrent(LEFT_Wheel) == (motion ? -3330 : 0));
    CHECK(SentCurrent(RIGHT_Wheel) == (motion ? 6660 : 0));
    CHECK(SentCurrent(2) == 0 && SentCurrent(3) == 0);
    if (motion)
    {
        NEAR(sent_torque[LEFT_BACK_id], 1.0f);
        NEAR(sent_torque[LEFT_FRONT_id], 2.0f);
        NEAR(sent_torque[RIGHT_BACK_id], -3.0f);
        NEAR(sent_torque[RIGHT_FRONT_id], -4.0f);
    }
    else
    {
        for (index = 0; index < 4; ++index) NEAR(sent_torque[index], 0.0f);
    }
    return enable_sends - before_enables;
}
static void EnableEvents(void)
{
    const Chassis_Mode_e modes[] = {NORMAL, FALLING_DOWN, FALLING_TO_NORMAL, ZERO_FORCE};
    unsigned int mode, enabled, motor, repeat;

    for (mode = 0; mode < sizeof(modes) / sizeof(modes[0]); ++mode)
    {
        for (enabled = 0; enabled < 2; ++enabled)
        {
            Reset(); RefreshExcept(-1);
            Chassis.chassis_mode = modes[mode];
            Chassis.chassis_enable = enabled ? ONLINE : OFFLINE;
            enable_refresh_feedback = 1;
            feedback_state[0] = 0U; Receive(0, 0);
            CHECK(SendCheck(1, (uint8_t)enabled) == 40U);
            CHECK(now == 40U && enable_delays == 40U);
            for (motor = 0; motor < 4; ++motor) CHECK(enable_per_joint[motor] == 10U);
            for (repeat = 0; repeat < 5; ++repeat)
            {
                ++now; RefreshExcept(-1);
                CHECK(SendCheck(1, (uint8_t)enabled) == 0U);
            }
            feedback_state[1] = 8U; Receive(1, 0);
            CHECK(SendCheck(1, (uint8_t)enabled) == 0U);
            feedback_state[0] = 1U; Receive(0, 0);
            CHECK(SendCheck(1, (uint8_t)enabled) == 0U);
            feedback_state[1] = 1U; Receive(1, 0);
            CHECK(SendCheck(1, (uint8_t)enabled) == 0U);
            feedback_state[2] = 9U; Receive(2, 0);
            CHECK(SendCheck(1, (uint8_t)enabled) == 40U);
            CHECK(enable_sends == 80U && enable_delays == 80U);
            for (motor = 0; motor < 4; ++motor) CHECK(enable_per_joint[motor] == 20U);
        }
    }
}

static void EnableFreshness(void)
{
    unsigned int motor;
    Reset();
    feedback_state[0] = 0U; Receive(0, 1); /* Invalid first frame. */
    CHECK(SendCheck(0, 0) == 0U);
    Receive(0, 0); /* A single fresh disabled joint can trigger the group request. */
    CHECK(SendCheck(0, 0) == 40U);
    feedback_state[0] = 1U; Receive(0, 0);
    CHECK(SendCheck(0, 0) == 0U); /* Unseen joints prevent rearming. */
    feedback_state[0] = 0U; Receive(0, 0);
    CHECK(SendCheck(0, 0) == 0U);
    for (motor = 0; motor < 4; ++motor)
    {
        feedback_state[motor] = 1U; Receive(motor, 0);
    }
    CHECK(SendCheck(0, 0) == 0U); /* Joint recovery can rearm even with wheels missing. */
    feedback_state[3] = 0U; Receive(3, 0);
    CHECK(SendCheck(0, 0) == 40U);

    Reset(); RefreshExcept(-1);
    feedback_state[0] = 0U; Receive(0, 0);
    now = 100; /* An expired disabled state must not trigger. */
    CHECK(SendCheck(0, 0) == 0U);
    Receive(0, 0); CHECK(SendCheck(0, 0) == 40U);
    feedback_state[0] = 1U; Receive(0, 0);
    CHECK(SendCheck(0, 0) == 0U); /* Stale normal states cannot unlock the latch. */
    feedback_state[0] = 0U; Receive(0, 0);
    CHECK(SendCheck(0, 0) == 0U);
    feedback_state[0] = 1U; RefreshExcept(-1);
    CHECK(SendCheck(1, 1) == 0U);
    feedback_state[0] = 0U; Receive(0, 0);
    CHECK(SendCheck(1, 1) == 40U);

    Reset(); RefreshExcept(-1);
    feedback_state[0] = 0U; Receive(0, 0);
    enable_refresh_feedback = 1;
    CHECK(SendCheck(1, 1) == 40U);
    feedback_state[0] = 1U; Receive(0, 0);
    now += 100U;
    CHECK(SendCheck(0, 0) == 0U); /* All states are 1, but all timestamps expired. */
    feedback_state[0] = 0U; Receive(0, 0);
    CHECK(SendCheck(0, 0) == 0U);

    Reset(); RefreshExcept(-1);
    now = 60; feedback_state[0] = 0U; Receive(0, 0);
    CHECK(SendCheck(0, 0) == 40U); /* 40 ms enable delay makes other feedback expire. */
    CHECK(now == 100U && Chassis.all_motors_online == 0U);
    CHECK(SendCheck(0, 0) == 0U);
    RefreshExcept(-1); CHECK(SendCheck(1, 1) == 0U);
}

int main(void)
{
    unsigned int motor, bad;
    CHECK(sizeof(routes) / sizeof(routes[0]) == CHASSIS_MOTOR_COUNT);
    Reset();
    SendCheck(0, 0);
    for (motor = 0; motor < CHASSIS_MOTOR_COUNT; ++motor)
    {
        CHECK(routes[motor].slot == (Chassis_MotorFeedback_e)motor);
        Receive(motor, 0);
        SendCheck(motor + 1 == CHASSIS_MOTOR_COUNT, motor + 1 == CHASSIS_MOTOR_COUNT);
    }
    now = 99; SendCheck(1, 1);
    now = 100; SendCheck(0, 0);
    for (motor = 0; motor < CHASSIS_MOTOR_COUNT; ++motor)
    {
        Reset(); RefreshExcept(-1);
        now = 99; SendCheck(1, 1);
        now = 100; RefreshExcept((int)motor); SendCheck(0, 0);
        for (bad = 0; bad < 4; ++bad) CHECK(Chassis.Joint_Motor[bad].Data.state == 1);
        now = 101; Receive(motor, 0); SendCheck(1, 1);
    }
    for (motor = 0; motor < CHASSIS_MOTOR_COUNT; ++motor)
    {
        for (bad = 1; bad <= 8; ++bad)
        {
            Reset(); RefreshExcept(-1);
            now = 100; RefreshExcept((int)motor);
            Receive(motor, bad); SendCheck(0, 0);
            Receive(motor, 0); SendCheck(1, 1);
        }
    }
    Reset(); RefreshExcept(-1);
    Chassis.chassis_enable = OFFLINE; SendCheck(1, 0);
    Chassis.chassis_enable = ONLINE; SendCheck(1, 1);
    now = 100; Chassis.chassis_enable = OFFLINE; SendCheck(0, 0);
    Reset(); now = UINT32_MAX - 49U; RefreshExcept(-1);
    now += 99U; SendCheck(1, 1);
    ++now; SendCheck(0, 0);
    RefreshExcept(-1); SendCheck(1, 1);
    interrupt_mask = 1U; SendCheck(1, 1);
    now += 100U; SendCheck(0, 0);
    Reset(); RefreshExcept(-1);
    now = 100; inject_before_mask = 1; SendCheck(1, 1);
    CHECK(now == 101);
    Reset();
    Chassis_MotorFeedbackReceived((Chassis_MotorFeedback_e)-1);
    Chassis_MotorFeedbackReceived(CHASSIS_MOTOR_COUNT);
    for (motor = 0; motor < CHASSIS_MOTOR_COUNT; ++motor) CHECK(motor_feedback_seen[motor] == 0U);
    SendCheck(0, 0);
    CHECK(enable_sends == 0U);
    EnableEvents();
    EnableFreshness();
    printf("motor online and enable-once: feedback, output, event latch, all modes and delay expiry: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
