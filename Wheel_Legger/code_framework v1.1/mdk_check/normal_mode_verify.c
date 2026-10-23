/* Tests use production types/functions; only hardware and other task chains are stubbed. */
#include <float.h>
#include <math.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define __packed
#define Chassis_Time 1
#define mirror (-1.0f)
typedef int hcan_t;
typedef int Pid_Set_Typedef;
typedef int PidTypedef;
#include "normal_api.inc"
Chassis_Info_Typedef Chassis;
INS_t INS;
float fb_dt = 0.001f, F_roll;
float t_fb, t_mode, t_leso, t_can, t_sum;
volatile uint8_t ground_offline_debug_enable;
static uint32_t primask;
static uint8_t rewrite_remote_on_unlock;
static float test_spin_yaw_rate;
static uint8_t test_motor_online = 1U;
static float cycle_dt = 0.001f;
static int detected_mode = -1;
static uint8_t recovery_handoff;
static unsigned checks, sends, leso_calls;
static float sent_force;
static jmp_buf cycle_end;
static void chassis_zero_outputs(void);
static uint32_t __get_PRIMASK(void) { return primask; }
static void __disable_irq(void) { primask = 1U; }
static void __set_PRIMASK(uint32_t value)
{
    primask = value;
    if (value == 0U && rewrite_remote_on_unlock != 0U)
    {
        Chassis.normal_state_request = rewrite_remote_on_unlock == 1U ? NORMAL_SPIN : NORMAL_MOVE;
        rewrite_remote_on_unlock = 0U;
        Chassis.set_goal.v_set = -2.0f;
        Chassis.set_goal.yaw_set_v = -3.0f;
    }
}
static float DWT_GetDeltaT(uint32_t *counter) { (void)counter; return cycle_dt; }
static float arm_sin_f32(float angle) { return sinf(angle); }
static float arm_cos_f32(float angle) { return cosf(angle); }
void VMC_calc_1(vmc_leg_t *vmc, INS_t *ins, float dt) { (void)vmc; (void)ins; (void)dt; }
void GroundContact_Update(float dt, float accel, const float torque[2][2])
{ (void)dt; (void)accel; (void)torque; }
void GroundContact_Reset(void)
{
    uint8_t enabled = Chassis.ground.control_enabled;
    memset(&Chassis.ground, 0, sizeof(Chassis.ground));
    Chassis.ground.control_enabled = enabled;
}
static void chassis_motor_online_update(void) { Chassis.all_motors_online = test_motor_online; }
static void Chassis_init(void) {}
static void falling_down_detect(void)
{ if (detected_mode >= 0) Chassis.chassis_mode = (Chassis_Mode_e)detected_mode; }
static void zero_force(void) { chassis_zero_outputs(); }
static void falling_down(void)
{
    if (recovery_handoff != 0U)
    {
        Chassis.chassis_mode = NORMAL;
        Chassis.leg_situation[LEFT_Leg].vmc.F0 = 42.0f;
        Chassis.leg_situation[RIGHT_Leg].vmc.F0 = 42.0f;
    }
}
static void falling_to_down(void) { falling_down(); }
static void LESO_Service(void) { ++leso_calls; }
static void VMC_translate(void) {}
static void Chassis_CanTransimit(void)
{ ++sends; sent_force = Chassis.leg_situation[LEFT_Leg].vmc.F0; }
static uint32_t osKernelSysTick(void) { return 0U; }
static void osDelay(uint32_t delay) { (void)delay; }
static void osDelayUntil(uint32_t *previous, uint32_t period)
{ (void)previous; (void)period; longjmp(cycle_end, 1); }
#include "normal_implementation.inc"
#define CHECK(condition) do { \
    ++checks; \
    if (!(condition)) { \
        fprintf(stderr, "NORMAL FAIL line %d: %s\n", __LINE__, #condition); \
        exit(EXIT_FAILURE); \
    } \
} while (0)
#define NEAR(actual, expected, tolerance) CHECK(isfinite(actual) && fabsf((actual) - (expected)) <= (tolerance))
static void Start(void)
{
    memset(&Chassis, 0, sizeof(Chassis));
    memset(&INS, 0, sizeof(INS));
    INS.ins_flag = 1U;
    test_motor_online = 1U;
    primask = 0U;
    rewrite_remote_on_unlock = 0U;
    detected_mode = -1;
    recovery_handoff = 0U;
    fb_dt = cycle_dt = 0.001f;
    normal_state_reset();
    Chassis.chassis_enable = ONLINE;
    Chassis.chassis_mode = NORMAL;
    Chassis.all_motors_online = 1U;
    Chassis.set_goal.set_L0_Left = Chassis.set_goal.set_L0_Right = 0.20f;
    for (unsigned i = 0U; i < 2U; ++i)
        Chassis.leg_situation[i].vmc.L0 = 0.20f;
    leg_length_ramp_initialized = 0U;
    chassis_leg_length_reference_update();
    normal_mode();
}
static void Cycle(float dt)
{
    cycle_dt = dt;
    if (setjmp(cycle_end) == 0) chassis_task();
}
static void CheckSlew(float dt)
{
    float v = Chassis.set_goal.v_ref, yaw = Chassis.set_goal.yaw_v_ref;
    fb_dt = dt;
    normal_mode();
    CHECK(fabsf(Chassis.set_goal.v_ref - v) <= NORMAL_V_RATE_MPS2 * dt + 1.0e-6f);
    CHECK(fabsf(Chassis.set_goal.yaw_v_ref - yaw) <= NORMAL_YAW_RATE_RADPS2 * dt + 1.0e-6f);
    CHECK(primask == 0U);
}
static void MotionAndTransitions(void)
{
    Start();
    Chassis.set_goal.v_set = 1.0f;
    Chassis.set_goal.yaw_set_v = -0.6f;
    CheckSlew(0.001f);
    NEAR(Chassis.set_goal.v_ref, 0.002f, 1.0e-7f);
    NEAR(Chassis.set_goal.yaw_v_ref, -0.003f, 1.0e-7f);
    for (unsigned i = 0U; i < 550U; ++i) CheckSlew(0.001f);
    NEAR(Chassis.set_goal.v_ref, 1.0f, 0.0f);
    NEAR(Chassis.set_goal.yaw_v_ref, -0.6f, 0.0f);
    float v_before = Chassis.set_goal.v_ref, yaw_before = Chassis.set_goal.yaw_v_ref;
    Chassis.body_state.x = 0.24f;
    Chassis.body_state.yaw = INS.Yaw = 0.45f;
    Chassis.set_goal.yaw_set = -0.8f;
    Chassis.chassis_flag.start_situate_flag = 1U;
    Chassis.normal_state_request = NORMAL_SPIN;
    Chassis.set_goal.yaw_set_v = test_spin_yaw_rate;
    CheckSlew(0.001f);
    CHECK(Chassis.normal_state == NORMAL_SPIN);
    NEAR(Chassis.body_state.x, 0.0f, 0.0f);
    CHECK(Chassis.chassis_flag.start_situate_flag == 0U);
    NEAR(Chassis.set_goal.v_ref, v_before - 0.002f, 1.0e-6f);
    CHECK(fabsf(Chassis.set_goal.yaw_v_ref - yaw_before) <= 0.003001f);
    NEAR(u[0], 0.0f, 0.0f); NEAR(u[2], 0.0f, 0.0f);
    NEAR(Chassis.set_goal.v_set, 1.0f, 0.0f);
    NEAR(Chassis.set_goal.yaw_set_v, test_spin_yaw_rate, 0.0f);
    NEAR(leg_length_ramp[0].out, 0.20f, 0.0f);
    for (unsigned i = 0U; i < 1500U; ++i) CheckSlew(0.001f);
    NEAR(Chassis.set_goal.v_ref, 0.0f, 0.0f);
    NEAR(Chassis.set_goal.yaw_v_ref, test_spin_yaw_rate, 0.0f);
    /* Position errors cannot create a return-to-old-heading command in SPIN. */
    Chassis.body_state.x = 0.3f;
    Chassis.set_goal.yaw_set = -1.0f;
    LQR_Calc(0.20f, 0.20f);
    NEAR(u[0], 0.0f, 0.0f); NEAR(u[2], 0.0f, 0.0f);
    Chassis.normal_state_request = NORMAL_MOVE;
    Chassis.set_goal.v_set = Chassis.set_goal.yaw_set_v = 0.0f;
    Chassis.body_state.d_yaw = 2.0f; /* Actual chassis is still turning. */
    Chassis.body_state.yaw = INS.Yaw = 0.6f;
    float spin_before = Chassis.set_goal.yaw_v_ref;
    CheckSlew(0.001f);
    CHECK(Chassis.normal_state == NORMAL_MOVE);
    CHECK(Chassis.body_state.d_yaw == 2.0f);
    CHECK(fabsf(Chassis.set_goal.yaw_v_ref) > 2.9f);
    CHECK(fabsf(Chassis.set_goal.yaw_v_ref - spin_before) <= 0.003001f);
    NEAR(Chassis.body_state.x, 0.0f, 0.0f);
    NEAR(Chassis.set_goal.yaw_set, 0.6f, 0.0f);
    /* Normal position formulas are restored immediately, without a wait state. */
    Chassis.body_state.x = 0.12f;
    Chassis.set_goal.yaw_set = 0.8f;
    LQR_Calc(0.20f, 0.20f);
    NEAR(u[0], -0.12f, 1.0e-6f); NEAR(u[2], 0.2f, 1.0e-6f);
    Chassis.normal_state_request = NORMAL_SPIN;
    Chassis.set_goal.yaw_set_v = test_spin_yaw_rate;
    CheckSlew(0.006f);
    CHECK(Chassis.normal_state == NORMAL_SPIN);
    CHECK(fabsf(Chassis.set_goal.yaw_v_ref) > 2.9f);
    /* Repeating the request does not repeat the transition's parking reset. */
    Chassis.body_state.x = 0.23f;
    CheckSlew(0.001f);
    NEAR(Chassis.body_state.x, 0.23f, 0.0f);
    Chassis.normal_state_request = (Chassis_Normal_State_e)99;
    CheckSlew(0.001f);
    CHECK(Chassis.normal_state == NORMAL_SPIN);
}
static void PeriodsAndInputOwnership(void)
{
    const float periods[] = {0.001f, 0.003f, 0.006f};
    for (unsigned i = 0U; i < 3U; ++i)
    {
        Start();
        Chassis.set_goal.v_set = 2.5f;
        Chassis.set_goal.yaw_set_v = -3.5f;
        CheckSlew(periods[i]);
        NEAR(Chassis.set_goal.v_ref, NORMAL_V_RATE_MPS2 * periods[i], 1.0e-7f);
        NEAR(Chassis.set_goal.yaw_v_ref, -NORMAL_YAW_RATE_RADPS2 * periods[i], 1.0e-7f);
        for (unsigned j = 0U; j < 1500U; ++j) CheckSlew(periods[i]);
        NEAR(Chassis.set_goal.v_ref, 2.5f, 0.0f);
        NEAR(Chassis.set_goal.yaw_v_ref, -3.5f, 0.0f);
    }
    Start();
    Chassis.set_goal.v_set = Chassis.set_goal.yaw_set_v = 1.0f;
    rewrite_remote_on_unlock = 1U;
    normal_mode();
    NEAR(Chassis.set_goal.v_ref, 0.002f, 1.0e-7f);
    NEAR(Chassis.set_goal.yaw_v_ref, 0.003f, 1.0e-7f);
    NEAR(Chassis.set_goal.v_set, -2.0f, 0.0f);
    NEAR(Chassis.set_goal.yaw_set_v, -3.0f, 0.0f);
    CHECK(Chassis.normal_state == NORMAL_MOVE && Chassis.normal_state_request == NORMAL_SPIN);
    /* The next frame may arrive after the snapshot in the opposite transition. */
    Start();
    Chassis.normal_state_request = NORMAL_SPIN;
    Chassis.set_goal.yaw_set_v = Chassis.set_goal.v_set = 1.0f;
    rewrite_remote_on_unlock = 2U;
    normal_mode();
    CHECK(Chassis.normal_state == NORMAL_SPIN && Chassis.normal_state_request == NORMAL_MOVE);
    NEAR(Chassis.set_goal.v_ref, 0.0f, 0.0f);
    NEAR(Chassis.set_goal.yaw_v_ref, 0.003f, 1.0e-7f);
    const float bad_periods[] = {0.0f, -0.001f, 0.051f, NAN, INFINITY};
    for (unsigned i = 0U; i < 5U; ++i)
    {
        Start();
        Chassis.set_goal.v_set = 1.0f;
        fb_dt = bad_periods[i];
        normal_mode();
        NEAR(Chassis.set_goal.v_ref, 0.002f, 1.0e-7f);
    }
}
static void GroundAndReset(void)
{
    Start();
    Chassis.normal_state_request = NORMAL_SPIN;
    Chassis.set_goal.yaw_set_v = test_spin_yaw_rate;
    for (unsigned i = 0U; i < 200U; ++i) CheckSlew(0.001f);
    Chassis.ground.control_enabled = 1U;
    Chassis.ground.all_off_flag = 1U;
    normal_mode();
    CHECK(Chassis.normal_state == NORMAL_SPIN);
    NEAR(Chassis.set_goal.v_ref, 0.0f, 0.0f);
    NEAR(Chassis.set_goal.yaw_v_ref, 0.0f, 0.0f);
    NEAR(Chassis.Wheel_Motor[LEFT_Wheel].wheel_T, 0.0f, 0.0f);
    NEAR(Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T, 0.0f, 0.0f);
    NEAR(Chassis.leg_situation[0].vmc.F0, 0.0f, 1.0e-6f);
    NEAR(F_roll, 0.0f, 0.0f);
    Chassis.ground.all_off_flag = 0U;
    normal_mode();
    NEAR(fabsf(Chassis.set_goal.yaw_v_ref), 0.003f, 1.0e-7f);
    Chassis.ground.control_enabled = 0U;
    Chassis.ground.all_off_flag = 1U;
    normal_mode();
    CHECK(fabsf(Chassis.set_goal.yaw_v_ref) > 0.003f);
    Chassis.chassis_enable = OFFLINE;
    normal_mode();
    CHECK(Chassis.normal_state == NORMAL_MOVE && Chassis.normal_state_request == NORMAL_MOVE);
    CHECK(normal_ramp_initialized == 0U);
    NEAR(Chassis.set_goal.v_ref, 0.0f, 0.0f); NEAR(Chassis.set_goal.yaw_v_ref, 0.0f, 0.0f);
    NEAR(Chassis.Wheel_Motor[LEFT_Wheel].wheel_T, 0.0f, 0.0f);
    NEAR(Chassis.leg_situation[0].vmc.F0, 0.0f, 0.0f);
    Chassis.chassis_enable = ONLINE;
    Chassis.all_motors_online = 0U;
    Chassis.normal_state_request = NORMAL_SPIN;
    Chassis.set_goal.yaw_set_v = test_spin_yaw_rate;
    normal_mode();
    CHECK(Chassis.normal_state_request == NORMAL_MOVE);
}
static void FeedbackParkingAndTaskOrder(void)
{
    Start();
    Chassis.body_state.x = 0.15f;
    Chassis.set_goal.v_set = 0.0f;
    Chassis.set_goal.v_ref = 0.1f;
    chassis_feedback_update();
    NEAR(Chassis.body_state.x, 0.0f, 0.0f);
    Chassis.body_state.x = 0.15f;
    Chassis.set_goal.v_set = 1.0f;
    Chassis.set_goal.v_ref = 0.0f;
    chassis_feedback_update();
    NEAR(Chassis.body_state.x, 0.15f, 0.0f);
    Chassis.normal_state = NORMAL_SPIN;
    chassis_feedback_update();
    NEAR(Chassis.body_state.x, 0.0f, 0.0f);
    Start();
    Chassis.normal_state_request = NORMAL_SPIN;
    Chassis.set_goal.yaw_set_v = test_spin_yaw_rate;
    Cycle(0.001f);
    CHECK(Chassis.normal_state == NORMAL_SPIN);
    NEAR(fabsf(Chassis.set_goal.yaw_v_ref), 0.003f, 1.0e-7f);
    detected_mode = ZERO_FORCE;
    Cycle(0.001f);
    CHECK(Chassis.normal_state == NORMAL_MOVE && Chassis.normal_state_request == NORMAL_MOVE);
    NEAR(sent_force, 0.0f, 0.0f);
    detected_mode = -1;
    Chassis.chassis_mode = FALLING_TO_NORMAL;
    Chassis.normal_state_request = NORMAL_SPIN;
    Chassis.set_goal.yaw_set_v = test_spin_yaw_rate;
    Chassis.set_goal.v_set = 1.0f;
    recovery_handoff = 1U;
    unsigned sends_before = sends, leso_before = leso_calls;
    Cycle(0.001f);
    CHECK(Chassis.chassis_mode == NORMAL);
    CHECK(normal_ramp_initialized == 0U);
    CHECK(Chassis.normal_state == NORMAL_MOVE && Chassis.normal_state_request == NORMAL_MOVE);
    NEAR(sent_force, 42.0f, 0.0f);
    CHECK(sends == sends_before + 1U && leso_calls == leso_before + 1U);
    Cycle(0.001f);
    CHECK(normal_ramp_initialized == 1U);
    NEAR(Chassis.set_goal.v_ref, 0.002f, 1.0e-7f);
    Chassis.normal_state_request = NORMAL_SPIN;
    Chassis.set_goal.yaw_set_v = test_spin_yaw_rate;
    Cycle(0.001f);
    test_motor_online = 0U;
    Cycle(0.001f);
    CHECK(Chassis.normal_state == NORMAL_MOVE && Chassis.normal_state_request == NORMAL_MOVE);
    CHECK(normal_ramp_initialized == 0U);
    NEAR(sent_force, 0.0f, 0.0f);
}
static void DynamicSpinTargets(void)
{
    Start();
    Chassis.normal_state_request = NORMAL_SPIN;
    const float targets[] = {0.75f, 1.5f, -1.5f, 0.0f};
    for (unsigned i = 0U; i < 4U; ++i)
    {
        Chassis.set_goal.yaw_set_v = targets[i];
        for (unsigned j = 0U; j < 1100U; ++j) CheckSlew(0.001f);
        CHECK(Chassis.normal_state == NORMAL_SPIN);
        NEAR(Chassis.set_goal.yaw_v_ref, targets[i], 0.0f);
        NEAR(Chassis.set_goal.v_ref, 0.0f, 0.0f);
    }
}
int main(int argc, char **argv)
{
    if (argc != 2) return 2;
    test_spin_yaw_rate = strtof(argv[1], NULL);
    MotionAndTransitions();
    PeriodsAndInputOwnership();
    GroundAndReset();
    DynamicSpinTargets();
    FeedbackParkingAndTaskOrder();
    printf("NORMAL two-state machine (spin=%+.1f rad/s): %u checks passed\n",
           (double)test_spin_yaw_rate, checks);
    return EXIT_SUCCESS;
}
