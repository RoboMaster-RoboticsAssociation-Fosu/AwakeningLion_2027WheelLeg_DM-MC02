/* Host harness around the actual recovery functions extracted by the runner.
 * Only hardware dependencies and the storage layout are substituted. */
#include "chassis_recovery.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { OFFLINE, ONLINE };
enum { FALLING_DOWN, FALLING_TO_NORMAL, NORMAL, ZERO_FORCE };
enum { LEFT_Leg, RIGHT_Leg };
enum { RIGHT_Wheel, LEFT_Wheel };
enum { RIGHT_FRONT_id, RIGHT_BACK_id, LEFT_BACK_id, LEFT_FRONT_id };

typedef struct
{
    float phi0, d_phi0, L0, d_L0, F0, Tp, torque_set[2];
} vmc_leg_t;

static struct
{
    int chassis_enable;
    int chassis_mode;
    struct { vmc_leg_t vmc; } leg_situation[2];
    struct { float wheel_T; struct { int SET_Current; } Data; } Wheel_Motor[2];
    struct { struct { int state; } Data; } Joint_Motor[4];
    struct { float theta; } body_state;
} Chassis;

typedef int Chassis_Enable_e;
static uint32_t clock_ms;
static unsigned int checks;
#define TARGET_STANCE_DEG ((SPIN_TARGET_PHI0_RAD - LEG_MOTION_PI * 0.5f) * LEG_MOTION_RAD_TO_DEG)
static unsigned int mapping_calls;
static int bad_mapping;

static uint32_t HAL_GetTick(void) { return clock_ms; }
static void DM_Enable(void) {}
static void osDelay(unsigned int delay) { (void)delay; }
static void LQR(void) {}
static void LEG_Lenth_Control(void) {}
static void VMC_calc_2(vmc_leg_t *vmc)
{
    ++mapping_calls;
    vmc->torque_set[0] = bad_mapping ? NAN : vmc->F0;
    vmc->torque_set[1] = vmc->Tp;
}

static void chassis_zero_outputs(void);
static void chassis_recovery_reset(void);
static void chassis_recovery_abort(void);
#include "chassis_recovery_under_test.inc"

#define CHECK(condition) do { \
    ++checks; \
    if (!(condition)) { \
        fprintf(stderr, "RECOVERY FAIL line %d: %s\n", __LINE__, #condition); \
        exit(1); \
    } \
} while (0)
#define NEAR(actual, expected) CHECK(fabsf((actual) - (expected)) < 0.0001f)

static void SetLeg(unsigned int leg, float angle_deg, float length)
{
    vmc_leg_t *vmc = &Chassis.leg_situation[leg].vmc;
    float phi0 = (angle_deg + 90.0f) * LEG_MOTION_DEG_TO_RAD;
    vmc->phi0 = atan2f(sinf(phi0), cosf(phi0));
    vmc->d_phi0 = 0.0f;
    vmc->L0 = length;
    vmc->d_L0 = 0.0f;
}

static void Reset(void)
{
    unsigned int joint;
    memset(&Chassis, 0, sizeof(Chassis));
    memset(&chassis_recovery, 0, sizeof(chassis_recovery));
    chassis_recovery_reset();
    clock_ms = 0U;
    mapping_calls = 0U;
    bad_mapping = 0;
    for (joint = 0U; joint < 4U; ++joint)
    {
        Chassis.Joint_Motor[joint].Data.state = 1;
    }
    Chassis.chassis_enable = OFFLINE;
    falling_down_detect(); /* Reset the detector's enable edge between cases. */
    Chassis.chassis_enable = ONLINE;
    Chassis.chassis_mode = FALLING_DOWN;
}

static void CheckZero(void)
{
    unsigned int leg;
    for (leg = 0U; leg < 2U; ++leg)
    {
        NEAR(Chassis.leg_situation[leg].vmc.F0, 0.0f);
        NEAR(Chassis.leg_situation[leg].vmc.Tp, 0.0f);
        NEAR(Chassis.leg_situation[leg].vmc.torque_set[0], 0.0f);
        NEAR(Chassis.leg_situation[leg].vmc.torque_set[1], 0.0f);
        NEAR(Chassis.Wheel_Motor[leg].wheel_T, 0.0f);
        CHECK(Chassis.Wheel_Motor[leg].Data.SET_Current == 0);
    }
}

static void SetPhi(unsigned int leg, float phi0, float length)
{
    SetLeg(leg, (phi0 - LEG_MOTION_PI * 0.5f) * LEG_MOTION_RAD_TO_DEG, length);
}

static void PrepareHandoff(void)
{
    Reset();
    SetPhi(LEFT_Leg, 1.2f, SPIN_RETRACT_LENGTH);
    SetPhi(RIGHT_Leg, 1.2f, SPIN_RETRACT_LENGTH);
    falling_down();
    clock_ms = SPIN_RETRACT_TIME_MS;
    falling_down();
    CHECK(chassis_recovery.phase[LEFT_Leg] == CHASSIS_RECOVERY_WAIT_ALIGN);
    CHECK(chassis_recovery.phase[RIGHT_Leg] == CHASSIS_RECOVERY_WAIT_ALIGN);
    CHECK(Chassis.chassis_mode == FALLING_DOWN);
    ++clock_ms;
    falling_down();
    CHECK(chassis_recovery.phase[LEFT_Leg] == CHASSIS_RECOVERY_ALIGN);
    CHECK(chassis_recovery.motion[LEFT_Leg].start_tick == chassis_recovery.motion[RIGHT_Leg].start_tick);
    clock_ms += SPIN_ALIGN_TIME_MS;
    SetPhi(LEFT_Leg, SPIN_ALIGN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
    SetPhi(RIGHT_Leg, SPIN_ALIGN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
    falling_down();
    CHECK(Chassis.chassis_mode == FALLING_TO_NORMAL);
}

static void WaitingThenCommonRetraction(void)
{
    Reset();
    SetPhi(LEFT_Leg, 1.2f, SPIN_SCAN_LENGTH);
    SetPhi(RIGHT_Leg, -0.57f, SPIN_SCAN_LENGTH);
    falling_down();
    CHECK(chassis_recovery.phase[LEFT_Leg] == CHASSIS_RECOVERY_SWING);
    CHECK(chassis_recovery.phase[RIGHT_Leg] == CHASSIS_RECOVERY_SWING);
    CHECK(chassis_recovery.sweep_arrived[LEFT_Leg] == 1U);
    CHECK(Chassis.leg_situation[LEFT_Leg].vmc.Tp == 0.0f);
    clock_ms = SPIN_RETRACT_TIME_MS;
    SetPhi(LEFT_Leg, -0.1f, SPIN_SCAN_LENGTH - 0.01f);
    falling_down();
    CHECK(chassis_recovery.phase[LEFT_Leg] == CHASSIS_RECOVERY_SWING);
    CHECK(chassis_recovery.phase[RIGHT_Leg] == CHASSIS_RECOVERY_SWING);
    NEAR(chassis_recovery.motion[LEFT_Leg].reference_length_m, SPIN_SCAN_LENGTH);
    CHECK(Chassis.leg_situation[LEFT_Leg].vmc.Tp == 0.0f);
    clock_ms = SPIN_SWING_ANGLE_TIME_MS;
    SetPhi(RIGHT_Leg, SPIN_TARGET_PHI0_RAD, SPIN_SCAN_LENGTH);
    falling_down();
    CHECK(chassis_recovery.phase[LEFT_Leg] == CHASSIS_RECOVERY_RETRACT);
    CHECK(chassis_recovery.phase[RIGHT_Leg] == CHASSIS_RECOVERY_RETRACT);
    CHECK(chassis_recovery.motion[LEFT_Leg].start_tick == clock_ms);
    CHECK(chassis_recovery.motion[RIGHT_Leg].start_tick == clock_ms);
    CHECK(chassis_recovery.sweep_arrived[0] == 0U && chassis_recovery.sweep_arrived[1] == 0U);
    NEAR(chassis_recovery.motion[LEFT_Leg].start_length_m, SPIN_SCAN_LENGTH - 0.01f);
    clock_ms += SPIN_RETRACT_TIME_MS;
    SetPhi(LEFT_Leg, 1.4f, SPIN_RETRACT_LENGTH);
    falling_down();
    CHECK(chassis_recovery.phase[LEFT_Leg] == CHASSIS_RECOVERY_WAIT_ALIGN);
    CHECK(chassis_recovery.phase[RIGHT_Leg] == CHASSIS_RECOVERY_RETRACT);
    CHECK(Chassis.leg_situation[LEFT_Leg].vmc.Tp == 0.0f);
    SetPhi(RIGHT_Leg, SPIN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
    ++clock_ms;
    falling_down();
    CHECK(chassis_recovery.phase[LEFT_Leg] == CHASSIS_RECOVERY_WAIT_ALIGN);
    CHECK(chassis_recovery.phase[RIGHT_Leg] == CHASSIS_RECOVERY_WAIT_ALIGN);
    SetPhi(LEFT_Leg, 1.3f, SPIN_RETRACT_LENGTH);
    ++clock_ms;
    falling_down();
    CHECK(chassis_recovery.phase[LEFT_Leg] == CHASSIS_RECOVERY_ALIGN);
    CHECK(chassis_recovery.phase[RIGHT_Leg] == CHASSIS_RECOVERY_ALIGN);
    CHECK(chassis_recovery.motion[LEFT_Leg].start_tick == clock_ms);
    CHECK(chassis_recovery.motion[RIGHT_Leg].start_tick == clock_ms);
    CHECK(chassis_recovery.motion[LEFT_Leg].command.direction == LEG_MOTION_POSITIVE);
    CHECK(chassis_recovery.motion[RIGHT_Leg].command.direction == LEG_MOTION_NEGATIVE);
    clock_ms += SPIN_ALIGN_TIME_MS;
    SetPhi(LEFT_Leg, SPIN_ALIGN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
    SetPhi(RIGHT_Leg, SPIN_ALIGN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
    falling_down();
    CHECK(Chassis.chassis_mode == FALLING_TO_NORMAL);
    ++clock_ms;
    falling_to_down();
    CHECK(Chassis.chassis_mode == NORMAL);
}

static void VerticalAndEquivalentTurns(void)
{
    Reset();
    SetLeg(LEFT_Leg, 0.0f, SPIN_RETRACT_LENGTH);
    SetLeg(RIGHT_Leg, -360.0f, SPIN_RETRACT_LENGTH);
    falling_down();
    CHECK(chassis_recovery.phase[LEFT_Leg] == CHASSIS_RECOVERY_RETRACT);
    CHECK(chassis_recovery.phase[RIGHT_Leg] == CHASSIS_RECOVERY_RETRACT);
    clock_ms = SPIN_RETRACT_TIME_MS;
    falling_down();
    CHECK(Chassis.chassis_mode == FALLING_DOWN);
    ++clock_ms;
    falling_down();
    CHECK(chassis_recovery.motion[LEFT_Leg].command.direction == LEG_MOTION_POSITIVE);
    clock_ms += SPIN_ALIGN_TIME_MS;
    SetPhi(LEFT_Leg, SPIN_ALIGN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
    SetPhi(RIGHT_Leg, SPIN_ALIGN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
    falling_down();
    CHECK(Chassis.chassis_mode == FALLING_TO_NORMAL);
    ++clock_ms;
    falling_to_down();
    CHECK(Chassis.chassis_mode == NORMAL);
}

static void FailureOutputs(void)
{
    Reset();
    SetLeg(LEFT_Leg, -120.0f, 0.20f);
    SetLeg(RIGHT_Leg, -120.0f, 0.20f);
    Chassis.leg_situation[LEFT_Leg].vmc.d_phi0 = NAN;
    Chassis.leg_situation[RIGHT_Leg].vmc.F0 = 8.0f;
    Chassis.leg_situation[RIGHT_Leg].vmc.torque_set[1] = 9.0f;
    Chassis.Wheel_Motor[LEFT_Wheel].wheel_T = 3.0f;
    Chassis.Wheel_Motor[RIGHT_Wheel].Data.SET_Current = 100;
    falling_down();
    CHECK(Chassis.chassis_mode == ZERO_FORCE);
    CheckZero();
    VMC_translate();
    CHECK(mapping_calls == 0U);
    CheckZero();

    Reset();
    SetLeg(LEFT_Leg, -120.0f, 0.20f);
    SetLeg(RIGHT_Leg, -120.0f, 0.20f);
    falling_down();
    bad_mapping = 1;
    VMC_translate();
    CHECK(Chassis.chassis_mode == ZERO_FORCE);
    CHECK(mapping_calls == 2U);
    CheckZero();

    Reset();
    SetLeg(LEFT_Leg, -120.0f, 0.20f);
    SetLeg(RIGHT_Leg, -120.0f, 0.20f);
    falling_down();
    Chassis.chassis_enable = OFFLINE;
    falling_down();
    VMC_translate();
    CHECK(mapping_calls == 0U);
    CheckZero();
}

/* A failed left leg must not advance the right context in the same cycle.
 * A failed right leg must clear the output already computed for the left. */
static void FailureOrdering(void)
{
    unsigned int failed_leg;
    for (failed_leg = LEFT_Leg; failed_leg <= RIGHT_Leg; ++failed_leg)
    {
        Reset();
        SetLeg(LEFT_Leg, -120.0f, 0.20f);
        SetLeg(RIGHT_Leg, -120.0f, 0.20f);
        Chassis.leg_situation[failed_leg].vmc.d_phi0 = NAN;
        falling_down();
        CHECK(Chassis.chassis_mode == ZERO_FORCE);
        CHECK(chassis_recovery.motion[failed_leg].result == LEG_MOTION_INVALID);
        if (failed_leg == LEFT_Leg)
        {
            CHECK(chassis_recovery.motion[RIGHT_Leg].result == LEG_MOTION_IDLE);
        }
        else
        {
            CHECK(chassis_recovery.motion[LEFT_Leg].result == LEG_MOTION_RUNNING);
        }
        CheckZero();

        PrepareHandoff();
        SetPhi(RIGHT_Leg, SPIN_ALIGN_TARGET_PHI0_RAD + 0.1f, SPIN_RETRACT_LENGTH);
        ++clock_ms;
        Chassis.leg_situation[failed_leg].vmc.d_phi0 = NAN;
        falling_to_down();
        CHECK(Chassis.chassis_mode == ZERO_FORCE);
        CHECK(chassis_recovery.motion[failed_leg].result == LEG_MOTION_INVALID);
        if (failed_leg == LEFT_Leg)
        {
            NEAR(chassis_recovery.motion[RIGHT_Leg].actual_angle_rad, SPIN_ALIGN_TARGET_PHI0_RAD - LEG_MOTION_PI * 0.5f);
        }
        CheckZero();
    }
}

static void SharedDeadline(void)
{
    Reset();
    SetLeg(LEFT_Leg, -120.0f, 0.20f);
    SetLeg(RIGHT_Leg, -120.0f, 0.20f);
    falling_down();
    clock_ms = SPIN_SCAN_TIMEOUT_MS - 500U;
    SetLeg(LEFT_Leg, TARGET_STANCE_DEG, 0.30f);
    SetLeg(RIGHT_Leg, TARGET_STANCE_DEG, 0.30f);
    falling_down();
    CHECK(chassis_recovery.phase[LEFT_Leg] == CHASSIS_RECOVERY_RETRACT);
    CHECK(chassis_recovery.motion[LEFT_Leg].command.timeout_ms == 500U);
    clock_ms = SPIN_SCAN_TIMEOUT_MS;
    falling_down();
    CHECK(Chassis.chassis_mode == ZERO_FORCE);
    CheckZero();
}

static void HandoffGuards(void)
{
    unsigned int scenario;
    for (scenario = 0U; scenario < 2U; ++scenario)
    {
        uint32_t handoff_tick;
        PrepareHandoff();
        handoff_tick = clock_ms;
        if (scenario == 0U)
        {
            Chassis.body_state.theta = SPIN_HANDOFF_PITCH_MAX_RAD;
        }
        else
        {
            SetPhi(LEFT_Leg, SPIN_ALIGN_TARGET_PHI0_RAD + 0.1f, SPIN_RETRACT_LENGTH);
        }
        ++clock_ms;
        falling_to_down();
        CHECK(Chassis.chassis_mode == FALLING_TO_NORMAL);
        clock_ms = handoff_tick + SPIN_HANDOFF_TIMEOUT_MS;
        falling_to_down();
        CHECK(Chassis.chassis_mode == ZERO_FORCE);
        CheckZero();
    }
}

static void HandoffOutputValidation(void)
{
    PrepareHandoff();
    ++clock_ms;
    falling_to_down();
    CHECK(Chassis.chassis_mode == NORMAL);
    /* The mode has changed, but this cycle still sends recovery commands. */
    bad_mapping = 1;
    VMC_translate();
    CHECK(Chassis.chassis_mode == ZERO_FORCE);
    CHECK(mapping_calls == 2U);
    CheckZero();
}

/* Run first: the detector's static enable history is still power-on OFFLINE. */
static void PowerOnAlreadyOnline(void)
{
    Chassis.chassis_enable = ONLINE;
    Chassis.chassis_mode = NORMAL;
    SetPhi(LEFT_Leg, 2.1f, 0.25f);
    SetPhi(RIGHT_Leg, 2.1f, 0.28f);
    falling_down_detect();
    CHECK(Chassis.chassis_mode == FALLING_DOWN);
    falling_down();
    CHECK(chassis_recovery.phase[LEFT_Leg] == CHASSIS_RECOVERY_RETRACT);
    CHECK(chassis_recovery.phase[RIGHT_Leg] == CHASSIS_RECOVERY_RETRACT);
    CHECK(Chassis.leg_situation[LEFT_Leg].vmc.Tp == 0.0f);
    CHECK(Chassis.leg_situation[RIGHT_Leg].vmc.Tp == 0.0f);
}

static void StartupPoseRouting(void)
{
    const float lower = SPIN_RETRACT_PHI0_MIN_RAD;
    const float upper = SPIN_RETRACT_PHI0_MAX_RAD;
    const float target = SPIN_ALIGN_TARGET_PHI0_RAD;
    const struct { float left, right, pitch; uint8_t left_ready, right_ready; } cases[] = {
        {2.1f, 2.1f, 0.0f, 1U, 1U},
        {1.2f, 2.1f, 0.0f, 1U, 1U},
        {2.1f, 1.2f, 0.0f, 1U, 1U},
        {target, target, 0.0f, 1U, 1U},
        {2.5f, 2.78f, 0.0f, 1U, 1U},
        {2.78f, upper, 1.0f, 1U, 1U},
        {2.78f, -1.0f, 0.0f, 1U, 0U},
        {-1.0f, 2.78f, 0.0f, 0U, 1U},
        {lower, target, 1.0f, 1U, 1U},
        {target, upper, -1.0f, 1U, 1U},
        {lower - 0.0001f, target, 0.0f, 0U, 1U},
        {target, upper + 0.0001f, 0.0f, 1U, 0U},
        {lower, upper, 0.3f, 1U, 1U},
        {upper, lower, -0.3f, 1U, 1U},
        {2.1f, 2.1f, 0.3001f, 1U, 1U},
        {2.1f, 2.1f, -0.3001f, 1U, 1U},
        {lower - 0.0001f, upper + 0.0001f, 0.0f, 0U, 0U},
        {2.1f + 2.0f * LEG_MOTION_PI, 2.1f, 0.0f, 0U, 1U},
        {2.1f, 2.1f - 2.0f * LEG_MOTION_PI, 0.0f, 1U, 0U},
        {1.2f, -0.57f, 0.0f, 1U, 0U},
        {-0.57f, 1.2f, 0.0f, 0U, 1U},
        {lower, 2.1f, 0.3001f, 1U, 1U},
        {lower - 0.0001f, 2.1f, 0.3001f, 0U, 1U},
        {2.1f, 2.1f, NAN, 1U, 1U},
        {2.1f, 2.1f, INFINITY, 1U, 1U},
        {2.1f, 2.1f, -INFINITY, 1U, 1U},
    };
    unsigned int index, leg;
    for (index = 0U; index < sizeof(cases) / sizeof(cases[0]); ++index)
    {
        Reset();
        clock_ms = 123U;
        SetPhi(LEFT_Leg, target, 0.25f);
        SetPhi(RIGHT_Leg, target, 0.28f);
        /* Preserve raw boundaries and equivalent turns without trigonometric rounding. */
        Chassis.leg_situation[LEFT_Leg].vmc.phi0 = cases[index].left;
        Chassis.leg_situation[RIGHT_Leg].vmc.phi0 = cases[index].right;
        Chassis.body_state.theta = cases[index].pitch;
        falling_down_detect();
        CHECK(Chassis.chassis_mode == FALLING_DOWN);
        CHECK(chassis_recovery.result == CHASSIS_RECOVERY_IDLE);
        falling_down();
        CHECK(chassis_recovery.phase[0] == (cases[index].left_ready && cases[index].right_ready ?
              CHASSIS_RECOVERY_RETRACT : CHASSIS_RECOVERY_SWING));
        CHECK(chassis_recovery.phase[1] == chassis_recovery.phase[0]);
        for (leg = 0U; leg < 2U; ++leg)
        {
            int ready = leg == 0U ? cases[index].left_ready : cases[index].right_ready;
            int retract = cases[index].left_ready && cases[index].right_ready;
            CHECK(chassis_recovery.motion[leg].command.angle_control ==
                  (ready ? LEG_MOTION_FREE : LEG_MOTION_POSITION));
            CHECK(chassis_recovery.sweep_arrived[leg] == (ready && !retract));
            CHECK(chassis_recovery.motion[leg].start_tick == clock_ms);
            NEAR(chassis_recovery.motion[leg].start_length_m, leg == 0U ? 0.25f : 0.28f);
            if (ready) CHECK(Chassis.leg_situation[leg].vmc.Tp == 0.0f);
        }
    }
}

static void NormalStartupToNormal(void)
{
    const float starts[][2] = {{2.1f, 2.1f}, {1.2f, 2.1f},
                              {SPIN_ALIGN_TARGET_PHI0_RAD, SPIN_ALIGN_TARGET_PHI0_RAD}};
    unsigned int scenario;
    for (scenario = 0U; scenario < 6U; ++scenario)
    {
        unsigned int late = scenario % 2U;
        unsigned int early = 1U - late;
        uint32_t align_tick;
        Reset();
        SetPhi(LEFT_Leg, starts[scenario / 2U][0], 0.30f);
        SetPhi(RIGHT_Leg, starts[scenario / 2U][1], 0.28f);
        falling_down_detect();
        CHECK(Chassis.chassis_mode == FALLING_DOWN);
        falling_down();
        CHECK(chassis_recovery.phase[0] == CHASSIS_RECOVERY_RETRACT);
        CHECK(chassis_recovery.phase[1] == CHASSIS_RECOVERY_RETRACT);
        SetPhi(LEFT_Leg, -0.2f, SPIN_RETRACT_LENGTH);
        SetPhi(RIGHT_Leg, 2.1f, SPIN_RETRACT_LENGTH);
        clock_ms = SPIN_RETRACT_TIME_MS - 1U;
        falling_down_detect();
        falling_down();
        CHECK(chassis_recovery.phase[0] == CHASSIS_RECOVERY_RETRACT);
        CHECK(chassis_recovery.phase[1] == CHASSIS_RECOVERY_RETRACT);
        CHECK(chassis_recovery.motion[0].start_tick == 0U);
        CHECK(chassis_recovery.motion[1].start_tick == 0U);
        CHECK(Chassis.leg_situation[0].vmc.Tp == 0.0f);
        CHECK(Chassis.leg_situation[1].vmc.Tp == 0.0f);
        Chassis.leg_situation[late].vmc.L0 = 0.20f;
        ++clock_ms;
        falling_down();
        CHECK(chassis_recovery.phase[early] == CHASSIS_RECOVERY_WAIT_ALIGN);
        CHECK(chassis_recovery.phase[late] == CHASSIS_RECOVERY_RETRACT);
        clock_ms += 50U;
        SetPhi(LEFT_Leg, 1.3f, SPIN_RETRACT_LENGTH);
        SetPhi(RIGHT_Leg, 1.9f, SPIN_RETRACT_LENGTH);
        falling_down();
        CHECK(chassis_recovery.phase[0] == CHASSIS_RECOVERY_WAIT_ALIGN);
        CHECK(chassis_recovery.phase[1] == CHASSIS_RECOVERY_WAIT_ALIGN);
        CHECK(Chassis.leg_situation[0].vmc.Tp == 0.0f);
        CHECK(Chassis.leg_situation[1].vmc.Tp == 0.0f);
        align_tick = ++clock_ms;
        falling_down();
        CHECK(chassis_recovery.phase[0] == CHASSIS_RECOVERY_ALIGN);
        CHECK(chassis_recovery.phase[1] == CHASSIS_RECOVERY_ALIGN);
        CHECK(chassis_recovery.motion[0].start_tick == align_tick);
        CHECK(chassis_recovery.motion[1].start_tick == align_tick);
        CHECK(chassis_recovery.motion[0].command.direction == LEG_MOTION_POSITIVE);
        CHECK(chassis_recovery.motion[1].command.direction == LEG_MOTION_NEGATIVE);
        NEAR(chassis_recovery.motion[0].start_angle_rad, 1.3f - LEG_MOTION_PI * 0.5f);
        NEAR(chassis_recovery.motion[1].start_angle_rad, 1.9f - LEG_MOTION_PI * 0.5f);
        clock_ms = align_tick + SPIN_ALIGN_TIME_MS;
        SetPhi(early, SPIN_ALIGN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
        SetPhi(late, SPIN_ALIGN_TARGET_PHI0_RAD + 0.051f, SPIN_RETRACT_LENGTH);
        falling_down_detect();
        falling_down();
        CHECK(Chassis.chassis_mode == FALLING_DOWN);
        CHECK(chassis_recovery.phase[early] == CHASSIS_RECOVERY_HOLD);
        CHECK(chassis_recovery.phase[late] == CHASSIS_RECOVERY_ALIGN);
        SetPhi(late, SPIN_ALIGN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
        ++clock_ms;
        falling_down();
        CHECK(Chassis.chassis_mode == FALLING_TO_NORMAL);
        SetPhi(LEFT_Leg, SPIN_ALIGN_TARGET_PHI0_RAD + 0.051f, SPIN_RETRACT_LENGTH);
        ++clock_ms;
        falling_to_down();
        CHECK(Chassis.chassis_mode == FALLING_TO_NORMAL);
        SetPhi(LEFT_Leg, SPIN_ALIGN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
        SetPhi(RIGHT_Leg, SPIN_ALIGN_TARGET_PHI0_RAD - 0.051f, SPIN_RETRACT_LENGTH);
        ++clock_ms;
        falling_to_down();
        CHECK(Chassis.chassis_mode == FALLING_TO_NORMAL);
        SetPhi(RIGHT_Leg, SPIN_ALIGN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
        Chassis.body_state.theta = SPIN_HANDOFF_PITCH_MAX_RAD;
        ++clock_ms;
        falling_to_down();
        CHECK(Chassis.chassis_mode == FALLING_TO_NORMAL);
        Chassis.body_state.theta = 0.0f;
        Chassis.leg_situation[late].vmc.d_L0 = 0.06f;
        ++clock_ms;
        falling_to_down();
        CHECK(Chassis.chassis_mode == FALLING_TO_NORMAL);
        Chassis.leg_situation[late].vmc.d_L0 = 0.0f;
        Chassis.leg_situation[late].vmc.L0 += 0.021f;
        ++clock_ms;
        falling_to_down();
        CHECK(Chassis.chassis_mode == FALLING_TO_NORMAL);
        Chassis.leg_situation[late].vmc.L0 = SPIN_RETRACT_LENGTH;
        ++clock_ms;
        falling_to_down();
        CHECK(Chassis.chassis_mode == NORMAL);
        CHECK(chassis_recovery.result == CHASSIS_RECOVERY_DONE);
        /* Retraction range applies only inside recovery, never ongoing NORMAL operation. */
        SetPhi(LEFT_Leg, 1.2f, 0.26f);
        SetPhi(RIGHT_Leg, 1.3f, 0.26f);
        falling_down_detect();
        CHECK(Chassis.chassis_mode == NORMAL);
        Chassis.chassis_enable = OFFLINE;
        falling_down_detect();
        Chassis.chassis_enable = ONLINE;
        ++clock_ms;
        falling_down_detect();
        CHECK(Chassis.chassis_mode == FALLING_DOWN);
        falling_down();
        CHECK(chassis_recovery.motion[0].start_tick == clock_ms);
        CHECK(chassis_recovery.motion[1].start_tick == clock_ms);
        NEAR(chassis_recovery.motion[0].start_length_m, 0.26f);
        NEAR(chassis_recovery.motion[1].start_length_m, 0.26f);
    }
}

static void EnableEdges(void)
{
    Reset();
    SetPhi(LEFT_Leg, SPIN_ALIGN_TARGET_PHI0_RAD, 0.13f);
    SetPhi(RIGHT_Leg, SPIN_ALIGN_TARGET_PHI0_RAD, 0.13f);
    falling_down_detect();
    CHECK(Chassis.chassis_mode == FALLING_DOWN);
    falling_down();
    clock_ms = SPIN_RETRACT_TIME_MS;
    falling_down();
    CHECK(chassis_recovery.phase[0] == CHASSIS_RECOVERY_WAIT_ALIGN);
    CHECK(chassis_recovery.phase[1] == CHASSIS_RECOVERY_WAIT_ALIGN);
    ++clock_ms;
    falling_down();
    NEAR(chassis_recovery.motion[0].target_angle_rad - chassis_recovery.motion[0].start_angle_rad, 0.0f);
    NEAR(chassis_recovery.motion[1].target_angle_rad - chassis_recovery.motion[1].start_angle_rad, 0.0f);
    clock_ms += SPIN_ALIGN_TIME_MS - 1U;
    falling_down();
    CHECK(Chassis.chassis_mode == FALLING_DOWN);
    ++clock_ms;
    falling_down();
    CHECK(Chassis.chassis_mode == FALLING_TO_NORMAL);
    ++clock_ms;
    falling_to_down();
    CHECK(Chassis.chassis_mode == NORMAL);
    Chassis.body_state.theta = 0.5f;
    falling_down_detect();
    CHECK(Chassis.chassis_mode == ZERO_FORCE);
    falling_down_detect();
    CHECK(Chassis.chassis_mode == ZERO_FORCE);
    Chassis.chassis_enable = OFFLINE;
    falling_down_detect();
    Chassis.chassis_enable = ONLINE;
    falling_down_detect();
    CHECK(Chassis.chassis_mode == FALLING_DOWN);
    falling_down();
    CHECK(chassis_recovery.motion[LEFT_Leg].result == LEG_MOTION_RUNNING);
}

static void NormalReleaseAndDisable(void)
{
    Reset();
    SetPhi(LEFT_Leg, -0.57f, 0.30f);
    SetPhi(RIGHT_Leg, -0.57f, 0.35f);
    falling_down();
    clock_ms = SPIN_SWING_ANGLE_TIME_MS;
    SetPhi(LEFT_Leg, SPIN_TARGET_PHI0_RAD, 0.30f);
    falling_down();
    CHECK(chassis_recovery.phase[LEFT_Leg] == CHASSIS_RECOVERY_SWING);
    CHECK(chassis_recovery.phase[RIGHT_Leg] == CHASSIS_RECOVERY_SWING);
    CHECK(Chassis.leg_situation[LEFT_Leg].vmc.Tp == 0.0f);
    CHECK(chassis_recovery.sweep_arrived[LEFT_Leg] == 1U);
    NEAR(chassis_recovery.motion[LEFT_Leg].reference_length_m, 0.30f);
    ++clock_ms;
    SetPhi(RIGHT_Leg, SPIN_TARGET_PHI0_RAD, 0.35f);
    falling_down();
    CHECK(chassis_recovery.phase[LEFT_Leg] == CHASSIS_RECOVERY_RETRACT);
    CHECK(chassis_recovery.phase[RIGHT_Leg] == CHASSIS_RECOVERY_RETRACT);
    CHECK(chassis_recovery.motion[LEFT_Leg].start_tick == clock_ms);
    CHECK(chassis_recovery.motion[RIGHT_Leg].start_tick == clock_ms);
    CHECK(Chassis.leg_situation[LEFT_Leg].vmc.Tp == 0.0f);
    CHECK(Chassis.leg_situation[RIGHT_Leg].vmc.Tp == 0.0f);
    CHECK(Chassis.Wheel_Motor[LEFT_Wheel].wheel_T == 0.0f);
    CHECK(Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T == 0.0f);
    clock_ms += SPIN_RETRACT_TIME_MS / 2U;
    SetPhi(LEFT_Leg, 2.0f, 0.30f);
    SetPhi(RIGHT_Leg, -3.0f, 0.35f);
    falling_down();
    VMC_translate();
    CHECK(mapping_calls == 2U);
    CHECK(Chassis.leg_situation[LEFT_Leg].vmc.Tp == 0.0f);
    CHECK(Chassis.leg_situation[RIGHT_Leg].vmc.Tp == 0.0f);
    CHECK(fabsf(Chassis.leg_situation[LEFT_Leg].vmc.torque_set[0]) > 1.0f);
    CHECK(fabsf(Chassis.leg_situation[RIGHT_Leg].vmc.torque_set[0]) > 1.0f);
    Chassis.chassis_enable = OFFLINE;
    falling_down();
    VMC_translate();
    CHECK(mapping_calls == 2U);
    CheckZero();
    CHECK(chassis_recovery.result == CHASSIS_RECOVERY_IDLE);
    Chassis.chassis_enable = ONLINE;
    ++clock_ms;
    falling_down();
    CHECK(chassis_recovery.motion[LEFT_Leg].command.angle_control == LEG_MOTION_FREE);
    CHECK(chassis_recovery.motion[RIGHT_Leg].command.angle_control == LEG_MOTION_POSITION);
    CHECK(chassis_recovery.motion[LEFT_Leg].start_tick == clock_ms);
    CHECK(chassis_recovery.motion[RIGHT_Leg].start_tick == clock_ms);
}

int main(void)
{
    PowerOnAlreadyOnline();
    StartupPoseRouting();
    NormalStartupToNormal();
    NormalReleaseAndDisable();
    WaitingThenCommonRetraction();
    VerticalAndEquivalentTurns();
    FailureOutputs();
    FailureOrdering();
    SharedDeadline();
    HandoffGuards();
    HandoffOutputValidation();
    EnableEdges();
    printf("Recovery integration verification passed (%u assertions).\n", checks);
    return 0;
}
