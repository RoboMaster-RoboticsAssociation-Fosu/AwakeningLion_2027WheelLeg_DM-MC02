/* Exact source replay through motor CAN packing; no physical dynamics model. */
#include <math.h>
#include <float.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define __packed
#define FDCAN_DLC_BYTES_8 (8U << 16)
typedef int hcan_t;
typedef int Pid_Set_Typedef;
typedef int PidTypedef;
#include "pitch_stop_api.inc"

INS_t INS;
hcan_t hfdcan1 = 1, hfdcan2 = 2;
static uint32_t clock_ms, interrupt_mask;
static unsigned checks, failures, sent_mask;
static uint8_t sent_joint[4][8], sent_wheel[8];
static uint32_t HAL_GetTick(void) { return clock_ms; }
static uint32_t osKernelSysTick(void) { return clock_ms; }
static void osDelay(uint32_t delay) { (void)delay; }
static void osDelayUntil(uint32_t *previous, uint32_t period) { *previous += period; }
static uint32_t __get_PRIMASK(void) { return interrupt_mask; }
static void __disable_irq(void) { interrupt_mask = 1U; }
static void __set_PRIMASK(uint32_t mask) { interrupt_mask = mask; }
static float DWT_GetDeltaT(uint32_t *counter) { (void)counter; return 0.001f; }
static float arm_sin_f32(float value) { return sinf(value); }
static float arm_cos_f32(float value) { return cosf(value); }
static uint8_t canx_send_data(hcan_t *can, uint16_t id, uint8_t *data, uint32_t len)
{
    (void)can;
    if (len == 8U && id >= 1U && id <= 4U)
    {
        memcpy(sent_joint[id - 1U], data, 8U);
        sent_mask |= 1U << (id - 1U);
    }
    if (len == 8U && id == 0x200U)
    {
        memcpy(sent_wheel, data, 8U);
        sent_mask |= 16U;
    }
    return 0U;
}
#include "pitch_stop_implementation.inc"

#define CHECK(condition) do { ++checks; if (!(condition)) { ++failures; \
    fprintf(stderr, "PITCH STOP FAIL line %d: %s\n", __LINE__, #condition); \
    exit(EXIT_FAILURE); \
} } while (0)

static void Frame(uint8_t s2)
{
    uint8_t data[18] = {0};
    for (unsigned channel = 0U; channel < 4U; ++channel)
    {
        unsigned bit = channel * 11U + 10U; /* Neutral = 1024. */
        data[bit / 8U] |= (uint8_t)(1U << (bit % 8U));
    }
    data[5] |= (uint8_t)((RC_SW_MID << 6) | (s2 << 4));
    data[17] = 4U;
    SBUS_TO_RC(data, &remote_ctrl);
    Remote_ControlStep();
}
static void Pose(float pitch, float phi0, float length, float pitch_rate, float phi0_rate)
{
    float delta = acosf((length * length + 0.208f * 0.208f - 0.25212f * 0.25212f)
                       / (2.0f * 0.208f * length));
    INS.Pitch = pitch;
    INS.Gyro[1] = -pitch_rate;
    Chassis.Joint_Motor[LEFT_BACK_id].Data.pos = phi0 + delta - PI / 2.0f;
    Chassis.Joint_Motor[LEFT_FRONT_id].Data.pos = phi0 - delta - PI / 2.0f;
    Chassis.Joint_Motor[RIGHT_BACK_id].Data.pos = -Chassis.Joint_Motor[LEFT_BACK_id].Data.pos;
    Chassis.Joint_Motor[RIGHT_FRONT_id].Data.pos = -Chassis.Joint_Motor[LEFT_FRONT_id].Data.pos;
    Chassis.Joint_Motor[LEFT_BACK_id].Data.vel = phi0_rate;
    Chassis.Joint_Motor[LEFT_FRONT_id].Data.vel = phi0_rate;
    Chassis.Joint_Motor[RIGHT_BACK_id].Data.vel = -phi0_rate;
    Chassis.Joint_Motor[RIGHT_FRONT_id].Data.vel = -phi0_rate;
}
static void Cycle(void)
{
    for (unsigned joint = 0U; joint < 4U; ++joint) Chassis.Joint_Motor[joint].Data.state = 1U;
    for (unsigned motor = 0U; motor < CHASSIS_MOTOR_COUNT; ++motor)
        Chassis_MotorFeedbackReceived((Chassis_MotorFeedback_e)motor);
    sent_mask = 0U;
    chassis_cycle();
    CHECK(sent_mask == 31U);
    CHECK(interrupt_mask == 0U);
    ++clock_ms;
}
static void CheckZeroFrames(void)
{
    for (unsigned joint = 0U; joint < 4U; ++joint)
    {
        const uint8_t *data = sent_joint[joint];
        unsigned kp = ((unsigned)(data[3] & 15U) << 8) | data[4];
        unsigned kd = ((unsigned)data[5] << 4) | (data[6] >> 4);
        unsigned torque = ((unsigned)(data[6] & 15U) << 8) | data[7];
        CHECK(kp == 0U && kd == 0U);
        CHECK(torque == (unsigned)float_to_uint(0.0f, J8009_T_MIN, J8009_T_MAX, 12));
    }
    for (unsigned i = 0U; i < 8U; ++i) CHECK(sent_wheel[i] == 0U);
}
static void StartNormal(void)
{
    Frame(RC_SW_DOWN);
    falling_down_detect();
    Frame(RC_SW_MID);
    falling_down_detect(); /* A completed recovery is a premise of this replay. */
    CHECK(Chassis.chassis_enable == ONLINE);
    CHECK(Chassis.chassis_mode == FALLING_DOWN);
    Chassis.chassis_mode = NORMAL;
    Pose(0.0f, PI / 2.0f, 0.20f, 0.0f, 0.0f);
    for (unsigned i = 0U; i < 1200U; ++i)
    {
        Frame(RC_SW_MID);
        Cycle();
    }
    CHECK(Chassis.chassis_mode == NORMAL);
    CHECK(leso_dbg_comp_leg > 0.8f);
}
int main(void)
{
    INS.ins_flag = 1U;
    Chassis_init();
    Pose(0.0f, PI / 2.0f, 0.20f, 0.0f, 0.0f);
    Frame(RC_SW_MID); Cycle();
    CHECK(Chassis.chassis_enable == OFFLINE && recovery_dbg_rx_on_seq == 0U);
    CheckZeroFrames();
    Frame(RC_SW_UP); Cycle();
    CHECK(Chassis.chassis_enable == OFFLINE && recovery_dbg_rx_on_seq == 0U);
    CheckZeroFrames();
    printf("startup: held MID/UP keep four joints and wheels zero\n");
    StartNormal();
    Pose(-0.3f, PI / 2.0f, 0.20f, -5.0f, 0.0f);
    Frame(RC_SW_MID);
    Cycle();
    printf("pitch=-0.3, pitch_rate=-5: mode=%d Tp=(%+.3f,%+.3f)\n",
           Chassis.chassis_mode, Chassis.leg_situation[0].vmc.Tp, Chassis.leg_situation[1].vmc.Tp);
    CHECK(Chassis.chassis_mode == NORMAL);
    Frame(RC_SW_DOWN);
    Cycle();
    CHECK(remote_ctrl.rc.s2 == RC_SW_DOWN);
    CHECK(Chassis.chassis_enable == OFFLINE);
    CheckZeroFrames();
    for (unsigned i = 0U; i < 1000U; ++i)
    {
        Pose(-0.3f, -PI + fmodf(0.08f * (float)i, 2.0f * PI), 0.20f, -5.0f, -40.0f);
        Frame(RC_SW_DOWN);
        Cycle();
        CHECK(Chassis.chassis_enable == OFFLINE);
        CheckZeroFrames();
    }
    printf("S2 DOWN: four MIT joint frames and wheel frame remain zero during rotating feedback\n");
    StartNormal();
    Pose(-0.51f, PI / 2.0f, 0.20f, -5.0f, 0.0f);
    Frame(RC_SW_MID);
    Cycle();
    CHECK(Chassis.chassis_mode == ZERO_FORCE);
    CheckZeroFrames();
    for (unsigned i = 0U; i < 50U; ++i)
    {
        Pose(-0.3f, 0.39f, 0.20f, 0.0f, -40.0f);
        Frame(RC_SW_MID);
        Cycle();
        CHECK(Chassis.chassis_mode == ZERO_FORCE);
        CheckZeroFrames();
    }
    printf("fall protection: pitch beyond -0.5 latches ZERO_FORCE, no automatic recovery\n");
    StartNormal();
    Pose(-0.3f, 0.39f, 0.20f, 0.0f, -40.0f);
    Frame(RC_SW_MID);
    Cycle();
    CHECK(Chassis.leg_situation[LEFT_Leg].vmc.phi0 < 0.4f);
    CHECK(Chassis.chassis_mode == ZERO_FORCE);
    CheckZeroFrames();
    printf("fall protection: phi0 below 0.4 latches ZERO_FORCE\n");
    {
        uint32_t enable_seq = recovery_dbg_rx_on_seq;
        clock_ms = remote_last_rx_tick + REMOTE_TIMEOUT_MS - 1U;
        Remote_ControlStep(); Cycle();
        CHECK(Chassis.chassis_enable == ONLINE && Chassis.chassis_mode == ZERO_FORCE);
        CheckZeroFrames();
        Remote_ControlStep(); Cycle();
        CHECK(Chassis.chassis_enable == OFFLINE && Chassis.chassis_mode == ZERO_FORCE);
        CheckZeroFrames();
        for (unsigned i = 0U; i < 200U; ++i) {
            if (i == 50U || i == 100U) {
                clock_ms += REMOTE_TIMEOUT_MS;
                Remote_ControlStep(); Cycle(); CheckZeroFrames();
            }
            Frame(RC_SW_MID); Cycle();
            CHECK(Chassis.chassis_enable == OFFLINE && Chassis.chassis_mode == ZERO_FORCE);
            CHECK(recovery_dbg_rx_on_seq == enable_seq);
            CheckZeroFrames();
        }
        Frame(RC_SW_DOWN); Cycle();
        CHECK(Chassis.chassis_enable == OFFLINE && recovery_dbg_rx_on_seq == enable_seq);
        CheckZeroFrames();
        Frame(RC_SW_MID); Cycle();
        CHECK(Chassis.chassis_enable == ONLINE && Chassis.chassis_mode == FALLING_DOWN);
        CHECK(recovery_dbg_rx_on_seq == enable_seq + 1U);
        for (unsigned i = 0U; i < 100U; ++i) {
            Frame(RC_SW_MID); Cycle();
            CHECK(recovery_dbg_rx_on_seq == enable_seq + 1U);
        }
        Frame(RC_SW_DOWN); Cycle(); CheckZeroFrames();
    }
    printf("fall/reconnect: held MID stays zero; fresh DOWN->MID starts recovery once\n");
    StartNormal();
    Pose(-0.3f, PI / 2.0f, 0.20f, -5.0f, 0.0f);
    clock_ms += REMOTE_TIMEOUT_MS;
    Remote_ControlStep();
    Cycle();
    CHECK(Chassis.chassis_enable == OFFLINE);
    CheckZeroFrames();
    printf("local RC timeout: four joints and wheels zero\n");
    printf("pitch/stop replay: %u checks, %u failures (hardware scheduling/CAN excluded)\n", checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
