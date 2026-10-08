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

static void IndependentPhases(void)
{
    float held_angle;
    Reset();
    SetPhi(LEFT_Leg, 1.2f, SPIN_SCAN_LENGTH);
    SetPhi(RIGHT_Leg, -0.57f, SPIN_SCAN_LENGTH);
    falling_down();
    held_angle = chassis_recovery.motion[LEFT_Leg].target_angle_rad;
    CHECK(chassis_recovery.phase[LEFT_Leg] == CHASSIS_RECOVERY_RETRACT);
    CHECK(chassis_recovery.phase[RIGHT_Leg] == CHASSIS_RECOVERY_SWING);
    clock_ms = SPIN_RETRACT_TIME_MS;
    SetPhi(LEFT_Leg, 1.2f, SPIN_RETRACT_LENGTH);
    falling_down();
    CHECK(chassis_recovery.phase[LEFT_Leg] == CHASSIS_RECOVERY_WAIT_ALIGN);
    CHECK(Chassis.chassis_mode == FALLING_DOWN);
    clock_ms = SPIN_RAMP_TIME_MS;
    SetPhi(LEFT_Leg, 1.4f, SPIN_RETRACT_LENGTH);
    SetPhi(RIGHT_Leg, SPIN_TARGET_PHI0_RAD, SPIN_SCAN_LENGTH);
    falling_down();
    CHECK(chassis_recovery.phase[LEFT_Leg] == CHASSIS_RECOVERY_WAIT_ALIGN);
    CHECK(chassis_recovery.phase[RIGHT_Leg] == CHASSIS_RECOVERY_RETRACT);
    NEAR(chassis_recovery.motion[LEFT_Leg].target_angle_rad, held_angle);
    NEAR(chassis_recovery.motion[LEFT_Leg].reference_length_m, SPIN_RETRACT_LENGTH);
    CHECK(Chassis.leg_situation[LEFT_Leg].vmc.Tp < -5.0f);
    clock_ms += SPIN_RETRACT_TIME_MS;
    SetPhi(RIGHT_Leg, SPIN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
    falling_down();
    CHECK(chassis_recovery.phase[RIGHT_Leg] == CHASSIS_RECOVERY_WAIT_ALIGN);
    CHECK(Chassis.chassis_mode == FALLING_DOWN);
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
            Chassis.body_state.theta = 0.25f;
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

static void EnableEdges(void)
{
    Reset();
    SetLeg(LEFT_Leg, 0.0f, 0.13f);
    SetLeg(RIGHT_Leg, 0.0f, 0.13f);
    falling_down_detect();
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

int main(void)
{
    IndependentPhases();
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
