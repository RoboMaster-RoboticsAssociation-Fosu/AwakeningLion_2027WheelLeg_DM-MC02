/* The runner extracts today's firmware implementation, including task ordering. */
#include <math.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { OFFLINE, ONLINE };
enum { FALLING_DOWN, FALLING_TO_NORMAL, NORMAL, ZERO_FORCE };
enum { LEFT_Leg, RIGHT_Leg };
#include "ramp_definitions.inc"

static struct
{
    int chassis_enable, chassis_mode;
    struct { float set_L0_Left, set_L0_Right, roll_set, roll_set_v; } set_goal;
    struct { struct { float L0, d_L0, theta, F0; } vmc; } leg_situation[2];
    struct { float roll, d_roll; } body_state;
} Chassis = {
#include "ramp_initial_goal.inc"
};
static struct { int ins_flag; } INS = { 1 };
static float fb_dt, F_roll, t_fb, t_mode, t_leso, t_can, t_sum;
static float cycle_dt = 0.001f;
static int detected_mode = -1, recovery_handoff;
static unsigned int checks, balance_calls;
static jmp_buf cycle_end;

/* Hardware/task dependencies only; balance control and ramp are real firmware. */
static void Chassis_init(void) {}
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

static void CheckTransitions(void)
{
    const float levels[] = { 0.15f, 0.20f, 0.25f };
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
    NEAR(leg_length_ramp[0].out, 0.20f, 0.00001f);
    for (tick = 0; tick < 500; ++tick) Step(0.001f, 0.001f);
    NEAR(leg_length_ramp[0].out, 0.25f, 0.00001f);

    Start(0.15f, 0.15f, 0.25f);
    for (tick = 0; tick < 300; ++tick) Step(0.001f, 0.001f);
    Chassis.set_goal.set_L0_Left = 0.15f;
    Chassis.set_goal.set_L0_Right = 0.20f;
    for (tick = 0; tick < 400; ++tick) Step(0.001f, 0.001f);
    CHECK(leg_length_ramp[0].out == 0.15f);
    CHECK(leg_length_ramp[1].out == 0.20f);
}

static void CheckTimingAndEntry(void)
{
    const float invalid_dt[] = { 0.0f, -1.0f, 0.051f, NAN, INFINITY };
    unsigned int tick;
    float previous;
    Start(0.139f, 0.31f, 0.20f);
    Step(0.001f, 0.001f);
    CHECK(leg_length_ramp[0].out < 0.15f);
    CHECK(leg_length_ramp[1].out > 0.25f);
    for (tick = 0; tick < 100; ++tick) Step(tick % 2 ? 0.006f : 0.001f, tick % 2 ? 0.006f : 0.001f);
    NEAR(leg_length_ramp[0].out, 0.1741f, 0.00001f);
    for (tick = 0; tick < sizeof(invalid_dt) / sizeof(invalid_dt[0]); ++tick)
    {
        previous = leg_length_ramp[0].out;
        Step(invalid_dt[tick], 0.001f);
        NEAR(leg_length_ramp[0].out - previous, 0.0001f, 0.00000004f);
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

int main(void)
{
    CHECK(Chassis.set_goal.set_L0_Left == 0.15f);
    CHECK(Chassis.set_goal.set_L0_Right == 0.15f);
    CheckTransitions();
    CheckTimingAndEntry();
    CheckPD();
    printf("Balance leg length ramp: %u checks passed\n", checks);
    return 0;
}
