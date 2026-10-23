/* The runner extracts today's firmware implementation, including task ordering. */
#include <math.h>
#include <float.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { OFFLINE, ONLINE };
enum { FALLING_DOWN, FALLING_TO_NORMAL, NORMAL, ZERO_FORCE };
enum { LEFT_Leg, RIGHT_Leg };
#include "ramp_definitions.inc"
#include "ground_contact.h"

static struct
{
    int chassis_enable, chassis_mode;
    uint8_t all_motors_online;
    Chassis_Normal_State_e normal_state, normal_state_request;
    struct { uint8_t start_situate_flag; } chassis_flag;
    GroundContact_t ground;
    struct { float set_L0_Left, set_L0_Right, roll_set, roll_set_v;
             float v_set, yaw_set_v, v_ref, yaw_v_ref, yaw_set; } set_goal;
    struct { struct { float L0, d_L0, theta, F0; } vmc; } leg_situation[2];
    struct { float roll, d_roll, x, yaw; } body_state;
} Chassis = {
#include "ramp_initial_goal.inc"
};
static struct { int ins_flag; } INS = { 1 };
volatile uint8_t ground_offline_debug_enable = 0U;
static float fb_dt, F_roll, t_fb, t_mode, t_leso, t_can, t_sum;
static float cycle_dt = 0.001f;
static int detected_mode = -1, recovery_handoff;
static unsigned int checks, balance_calls;
static jmp_buf cycle_end;
typedef struct { struct { uint8_t s1; } rc; } RC_Ctl_t;
static uint8_t remote_was_enabled;
static void Remote_RecordEnable(uint32_t tick) { (void)tick; }

/* Hardware/task dependencies only; balance control and ramp are real firmware. */
static void Chassis_init(void) {}
static uint32_t __get_PRIMASK(void) { return 0U; }
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t mask) { (void)mask; }
static void chassis_zero_outputs(void)
{
    Chassis.leg_situation[0].vmc.F0 = 0.0f;
    Chassis.leg_situation[1].vmc.F0 = 0.0f;
}
/* 检测行为由 ground_contact_verify 验证；此处只提供任务接入依赖。 */
void GroundContact_Reset(void)
{
    uint8_t enabled = Chassis.ground.control_enabled;
    memset(&Chassis.ground, 0, sizeof(Chassis.ground));
    Chassis.ground.control_enabled = enabled;
}
static void chassis_feedback_update(void) { fb_dt = cycle_dt; }
static void YAW_Parameter_Processing(void) {}
static void falling_down_detect(void)
{
    if (detected_mode >= 0) Chassis.chassis_mode = detected_mode;
}
static void zero_force(void) {}
static void falling_down(void) { if (recovery_handoff) Chassis.chassis_mode = NORMAL; }
static void falling_to_down(void) { falling_down(); }
static void LQR(void) { ++balance_calls; }
static void LESO_Service(void) {}
static void VMC_translate(void) {}
static void Chassis_CanTransimit(void) {}
static uint32_t osKernelSysTick(void) { return 0U; }
static void osDelay(uint32_t delay) { (void)delay; }
static void osDelayUntil(uint32_t *last, uint32_t period)
{
    (void)last;
    (void)period;
    longjmp(cycle_end, 1);
}
static float DWT_GetDeltaT(uint32_t *counter) { (void)counter; return cycle_dt; }
static float arm_cos_f32(float angle) { return cosf(angle); }
#include "ramp_implementation.inc"

#define CHECK(condition) do { \
    ++checks; \
    if (!(condition)) { \
        fprintf(stderr, "LENGTH RAMP FAIL line %d: %s\n", __LINE__, #condition); \
        exit(1); \
    } \
} while (0)
#define NEAR(actual, expected, tolerance) CHECK(fabsf((actual) - (expected)) <= (tolerance))

static void Cycle(float dt)
{
    cycle_dt = dt;
    if (setjmp(cycle_end) == 0) chassis_task();
}

static void Start(float left, float right, float target)
{
    memset(&Chassis, 0, sizeof(Chassis));
    detected_mode = -1;
    recovery_handoff = 0;
    Chassis.chassis_mode = NORMAL;
    Cycle(0.001f); /* Offline iteration must invalidate a previous reference. */
    CHECK(leg_length_ramp_initialized == 0U);
    Chassis.chassis_enable = ONLINE;
    Chassis.all_motors_online = 1U;
    Chassis.set_goal.set_L0_Left = target;
    Chassis.set_goal.set_L0_Right = target;
    Chassis.leg_situation[LEFT_Leg].vmc.L0 = left;
    Chassis.leg_situation[RIGHT_Leg].vmc.L0 = right;
    Cycle(0.001f);
    CHECK(leg_length_ramp_initialized == 1U);
    CHECK(leg_length_ramp[LEFT_Leg].out == left);
    CHECK(leg_length_ramp[RIGHT_Leg].out == right);
    NEAR(Chassis.leg_situation[LEFT_Leg].vmc.F0, body_mg / 2.0f, 0.00001f);
    NEAR(Chassis.leg_situation[RIGHT_Leg].vmc.F0, body_mg / 2.0f, 0.00001f);
}

static void Step(float dt, float effective_dt)
{
    float previous[2] = { leg_length_ramp[0].out, leg_length_ramp[1].out };
    float targets[2] = { Chassis.set_goal.set_L0_Left, Chassis.set_goal.set_L0_Right };
    unsigned int leg;
    Cycle(dt);
    for (leg = 0U; leg < 2U; ++leg)
    {
        float next = leg_length_ramp[leg].out;
        CHECK(fabsf(next - previous[leg]) <= LEG_LENGTH_RAMP_RATE_M_S * effective_dt + 0.00000004f);
        CHECK(next >= fminf(previous[leg], targets[leg]));
        CHECK(next <= fmaxf(previous[leg], targets[leg]));
        CHECK(isfinite(next));
    }
}

static void SelectLegSwitch(uint8_t s1)
{
    RC_Ctl_t input = {{s1}};
    Remote_ApplyControl(&input, 1U, 0U, 1U, NORMAL_MOVE, 0.0f, 0.0f);
}

static float SwitchLength(uint8_t s1)
{
    RC_Ctl_t input = {{s1}};
    float target = 0.0f;
    CHECK(Remote_MapLegLength(&input, &target));
    return target;
}

static void CheckRemoteSwitchTransitions(void)
{
    const uint8_t switches[] = {RC_SW_DOWN, RC_SW_MID, RC_SW_UP};
    float middle = SwitchLength(RC_SW_MID), upper = SwitchLength(RC_SW_UP);
    unsigned int tick, from, to;
    unsigned int duration = (unsigned int)ceilf((upper - middle) /
                            (LEG_LENGTH_RAMP_RATE_M_S * 0.001f));
    Start(middle, middle, middle);
    SelectLegSwitch(RC_SW_UP);
    CHECK(Chassis.set_goal.set_L0_Left == upper);
    CHECK(Chassis.set_goal.set_L0_Right == upper);
    for (tick = 0U; tick < duration + 1U; ++tick) Step(0.001f, 0.001f);
    printf("S1 MID -> UP: target %.3f m, reference %.3f m after %.3f s\n",
           upper, leg_length_ramp[0].out, (duration + 1U) * 0.001f);
    CHECK(leg_length_ramp[0].out == upper);
    CHECK(leg_length_ramp[1].out == upper);
    for (from = 0U; from < 3U; ++from)
    for (to = 0U; to < 3U; ++to)
    {
        float start = SwitchLength(switches[from]);
        float target = SwitchLength(switches[to]);
        unsigned int count = (unsigned int)ceilf(fabsf(target - start) /
                             (LEG_LENGTH_RAMP_RATE_M_S * 0.001f)) + 2U;
        Start(start, start, start);
        SelectLegSwitch(switches[to]);
        for (tick = 0U; tick < count; ++tick) Step(0.001f, 0.001f);
        CHECK(leg_length_ramp[0].out == target);
        CHECK(leg_length_ramp[1].out == target);
    }
    /* Reverse above the old 0.25 m cap without reseeding the reference. */
    Start(middle, middle, middle);
    SelectLegSwitch(RC_SW_UP);
    for (tick = 0U; tick < duration / 2U; ++tick) Step(0.001f, 0.001f);
    CHECK(leg_length_ramp[0].out > middle);
    SelectLegSwitch(RC_SW_DOWN);
    for (tick = 0U; tick < 2000U; ++tick) Step(0.001f, 0.001f);
    CHECK(leg_length_ramp[0].out == SwitchLength(RC_SW_DOWN));
    /* Recovery starts below the short preset and must reach the upper preset. */
    Start(0.139f, 0.142f, middle);
    SelectLegSwitch(RC_SW_UP);
    Step(0.001f, 0.001f);
    CHECK(leg_length_ramp[0].out < SwitchLength(RC_SW_DOWN));
    for (tick = 0U; tick < 2000U; ++tick) Step(0.001f, 0.001f);
    CHECK(leg_length_ramp[0].out == upper);
    CHECK(leg_length_ramp[1].out == upper);
}

static void CheckTransitions(void)
{
    const float levels[] = { SwitchLength(RC_SW_DOWN), SwitchLength(RC_SW_MID), SwitchLength(RC_SW_UP) };
    const float periods[] = { 0.001f, 0.003f, 0.006f };
    unsigned int from, to, period, tick;
    for (from = 0; from < 3; ++from)
    for (to = 0; to < 3; ++to)
    for (period = 0; period < 3; ++period)
    {
        float dt = periods[period];
        unsigned int count = (unsigned int)ceilf(fabsf(levels[to] - levels[from]) /
                             (LEG_LENGTH_RAMP_RATE_M_S * dt)) + 2U;
        Start(levels[from], levels[from], levels[to]);
        for (tick = 0; tick < count; ++tick) Step(dt, dt);
        CHECK(leg_length_ramp[0].out == levels[to]);
        CHECK(leg_length_ramp[1].out == levels[to]);
        for (tick = 0; tick < 10; ++tick) Step(dt, dt);
        CHECK(leg_length_ramp[0].out == levels[to]);
    }
    Start(0.15f, 0.15f, 0.25f);
    for (tick = 0; tick < 500; ++tick) Step(0.001f, 0.001f);
    NEAR(leg_length_ramp[0].out, fminf(0.25f, 0.15f + LEG_LENGTH_RAMP_RATE_M_S * 0.5f), 0.00001f);
    for (tick = 0; tick < 500; ++tick) Step(0.001f, 0.001f);
    NEAR(leg_length_ramp[0].out, fminf(0.25f, 0.15f + LEG_LENGTH_RAMP_RATE_M_S), 0.00001f);

    Start(0.15f, 0.15f, 0.25f);
    for (tick = 0; tick < 300; ++tick) Step(0.001f, 0.001f);
    Chassis.set_goal.set_L0_Left = 0.15f;
    Chassis.set_goal.set_L0_Right = 0.20f;
    unsigned int retarget_ticks = (unsigned int)ceilf(fmaxf(fabsf(leg_length_ramp[0].out - 0.15f),
                                 fabsf(leg_length_ramp[1].out - 0.20f)) /
                                 (LEG_LENGTH_RAMP_RATE_M_S * 0.001f)) + 2U;
    for (tick = 0; tick < retarget_ticks; ++tick) Step(0.001f, 0.001f);
    CHECK(leg_length_ramp[0].out == 0.15f);
    CHECK(leg_length_ramp[1].out == 0.20f);
}

static void CheckTimingAndEntry(void)
{
    const float invalid_dt[] = { 0.0f, -1.0f, 0.051f, NAN, INFINITY };
    unsigned int tick;
    float previous;
    Start(0.139f, 0.39f, 0.20f);
    Step(0.001f, 0.001f);
    CHECK(leg_length_ramp[0].out < 0.15f);
    CHECK(leg_length_ramp[1].out > LEG_LENGTH_LONG_M);
    for (tick = 0; tick < 100; ++tick) Step(tick % 2 ? 0.006f : 0.001f, tick % 2 ? 0.006f : 0.001f);
    NEAR(leg_length_ramp[0].out, fminf(0.20f, 0.139f + LEG_LENGTH_RAMP_RATE_M_S * 0.351f), 0.00001f);
    for (tick = 0; tick < sizeof(invalid_dt) / sizeof(invalid_dt[0]); ++tick)
    {
        previous = leg_length_ramp[0].out;
        Step(invalid_dt[tick], 0.001f);
        NEAR(leg_length_ramp[0].out - previous, fminf(LEG_LENGTH_RAMP_RATE_M_S * 0.001f,
             fmaxf(0.0f, 0.20f - previous)), 0.00000004f);
    }

    Chassis.chassis_enable = OFFLINE;
    previous = leg_length_ramp[0].out;
    Cycle(0.01f);
    CHECK(leg_length_ramp_initialized == 0U);
    CHECK(leg_length_ramp[0].out == previous);
    Chassis.leg_situation[0].vmc.L0 = 0.18f;
    Chassis.leg_situation[1].vmc.L0 = 0.22f;
    Chassis.chassis_enable = ONLINE;
    Cycle(0.001f);
    CHECK(leg_length_ramp[0].out == 0.18f);
    CHECK(leg_length_ramp[1].out == 0.22f);

    detected_mode = ZERO_FORCE; /* Detection must invalidate BEFORE mode dispatch. */
    Cycle(0.001f);
    CHECK(leg_length_ramp_initialized == 0U);
    detected_mode = -1;
    Chassis.chassis_mode = FALLING_DOWN;
    Cycle(0.001f);
    CHECK(leg_length_ramp_initialized == 0U);
    Chassis.chassis_mode = FALLING_TO_NORMAL;
    Chassis.leg_situation[0].vmc.L0 = 0.139f;
    Chassis.leg_situation[1].vmc.L0 = 0.142f;
    recovery_handoff = 1;
    balance_calls = 0;
    Cycle(0.001f);
    CHECK(Chassis.chassis_mode == NORMAL);
    CHECK(balance_calls == 0U); /* Last recovery frame still owns the outputs. */
    CHECK(leg_length_ramp_initialized == 0U);
    Cycle(0.001f);
    CHECK(balance_calls == 1U);
    CHECK(leg_length_ramp[0].out == 0.139f);
    CHECK(leg_length_ramp[1].out == 0.142f);
    Step(0.001f, 0.001f);
}

static void CheckPD(void)
{
    float left, right, roll;
    Start(0.18f, 0.21f, 0.25f);
    Chassis.leg_situation[0].vmc.d_L0 = 0.02f;
    Chassis.leg_situation[1].vmc.d_L0 = -0.03f;
    Chassis.leg_situation[0].vmc.theta = 0.1f;
    Chassis.leg_situation[1].vmc.theta = -0.2f;
    Chassis.body_state.roll = 0.01f;
    Chassis.body_state.d_roll = 0.02f;
    Step(0.001f, 0.001f);
    roll = -ROLL_PID_KP * 0.01f - ROLL_PID_KD * 0.02f;
    left = body_mg / 2.0f * cosf(0.1f) + LEG_PID_KP * (leg_length_ramp[0].out - 0.18f)
           - LEG_PID_KD_RATE * 0.02f + roll;
    right = body_mg / 2.0f * cosf(-0.2f) + LEG_PID_KP * (leg_length_ramp[1].out - 0.21f)
            + LEG_PID_KD_RATE * 0.03f - roll;
    NEAR(Chassis.leg_situation[0].vmc.F0, left, 0.00001f);
    NEAR(Chassis.leg_situation[1].vmc.F0, right, 0.00001f);
}

static void CheckOfflineObservationReset(void)
{
    detected_mode=-1; recovery_handoff=0;
    Chassis.chassis_enable=OFFLINE; Chassis.chassis_mode=ZERO_FORCE;
    ground_offline_debug_enable=1U; Chassis.ground.all_off_flag=1U;
    Cycle(0.001f); CHECK(Chassis.ground.all_off_flag==1U);
    ground_offline_debug_enable=0U;
    Cycle(0.001f); CHECK(Chassis.ground.all_off_flag==0U);
    Chassis.chassis_enable=ONLINE; Chassis.chassis_mode=FALLING_DOWN;
    ground_offline_debug_enable=1U; Chassis.ground.all_off_flag=1U;
    Cycle(0.001f); CHECK(Chassis.ground.all_off_flag==0U);
    ground_offline_debug_enable=0U;
}
int main(void)
{
    CHECK(Chassis.set_goal.set_L0_Left == LEG_LENGTH_SHORT_M);
    CHECK(Chassis.set_goal.set_L0_Right == LEG_LENGTH_SHORT_M);
    CheckRemoteSwitchTransitions();
    CheckTransitions();
    CheckTimingAndEntry();
    CheckPD();
    CheckOfflineObservationReset();
    printf("Balance leg length ramp: %u checks passed\n", checks);
    return 0;
}
