/* Replay feedback through the actual recovery API and portable controllers. */
#include "chassis_recovery.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned int checks;
#define CHECK(condition) do { \
    ++checks; \
    if (!(condition)) { \
        fprintf(stderr, "RECOVERY API FAIL line %d: %s\n", __LINE__, #condition); \
        exit(1); \
    } \
} while (0)
#define NEAR(actual, expected) CHECK(fabsf((actual) - (expected)) < 0.0001f)

static ChassisRecovery_Input Input(float phi0, float length)
{
    ChassisRecovery_Input input = {0};
    unsigned int leg;
    input.enabled = 1U;
    for (leg = 0U; leg < 2U; ++leg)
    {
        input.leg[leg].phi0_rad = phi0;
        input.leg[leg].length_m = length;
    }
    return input;
}

static void Zero(const ChassisRecovery_Output *output)
{
    unsigned int leg;
    for (leg = 0U; leg < 2U; ++leg)
    {
        NEAR(output->leg[leg].F0, 0.0f);
        NEAR(output->leg[leg].Tp, 0.0f);
    }
}

static uint32_t WaitAlign(ChassisRecovery_Context *context,
                          ChassisRecovery_Input *input, ChassisRecovery_Output *output,
                          uint32_t start)
{
    *input = Input(1.2f, SPIN_RETRACT_LENGTH);
    CHECK(ChassisRecovery_Run(context, input, start, output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context->phase[0] == CHASSIS_RECOVERY_RETRACT);
    CHECK(ChassisRecovery_Run(context, input, start + SPIN_RETRACT_TIME_MS, output) ==
          CHASSIS_RECOVERY_RUNNING);
    CHECK(context->phase[0] == CHASSIS_RECOVERY_WAIT_ALIGN);
    CHECK(context->phase[1] == CHASSIS_RECOVERY_WAIT_ALIGN);
    return start + SPIN_RETRACT_TIME_MS;
}

static uint32_t Handoff(ChassisRecovery_Context *context,
                        ChassisRecovery_Input *input, ChassisRecovery_Output *output,
                        uint32_t start)
{
    uint32_t now = WaitAlign(context, input, output, start) + 1U;
    CHECK(ChassisRecovery_Run(context, input, now, output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context->phase[0] == CHASSIS_RECOVERY_ALIGN);
    CHECK(context->phase[1] == CHASSIS_RECOVERY_ALIGN);
    *input = Input(SPIN_ALIGN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
    now += SPIN_ALIGN_TIME_MS;
    CHECK(ChassisRecovery_Run(context, input, now, output) == CHASSIS_RECOVERY_HANDOFF);
    CHECK(context->handoff_tick == now);
    return now;
}

static float SweepLengthReference(float start_length, uint32_t elapsed, uint32_t duration)
{
    float fraction = duration == 0U || elapsed >= duration ?
                     1.0f : (float)elapsed / (float)duration;
    return start_length + (SPIN_SCAN_LENGTH - start_length) * fraction;
}

static void SweepTargetTrajectory(void)
{
    ChassisRecovery_Context context = {0};
    ChassisRecovery_Input input = Input(-0.57f, SPIN_RETRACT_LENGTH);
    ChassisRecovery_Output output;
    const float travel = SPIN_TARGET_PHI0_RAD - 2.0f * LEG_MOTION_PI + 0.57f;
    float retract_start_length;
    unsigned int step;
    uint32_t now;

    CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
    NEAR(context.motion[0].target_angle_rad - context.motion[0].start_angle_rad, travel);
    for (step = 1U; step <= 40U; ++step)
    {
        float fraction = (float)step / 40.0f;
        float angle = -0.57f + travel * fraction;
        now = SPIN_SWING_ANGLE_TIME_MS * step / 40U;
        input = Input(atan2f(sinf(angle), cosf(angle)),
                      SweepLengthReference(SPIN_RETRACT_LENGTH, now, SPIN_SWING_LENGTH_TIME_MS));
        CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
        CHECK(context.phase[0] == (step == 40U ? CHASSIS_RECOVERY_RETRACT : CHASSIS_RECOVERY_SWING));
        CHECK(context.phase[1] == context.phase[0]);
        NEAR(context.motion[0].reference_length_m, input.leg[0].length_m);
        NEAR(context.motion[0].reference_angle_rad, context.motion[0].actual_angle_rad);
        /* Sweep uses gravity compensation; retraction releases Tp immediately. */
        if (step == 40U)
        {
            CHECK(output.leg[0].Tp == 0.0f && output.leg[1].Tp == 0.0f);
            CHECK(context.motion[0].command.angle_control == LEG_MOTION_FREE);
            CHECK(context.motion[0].start_tick == context.motion[1].start_tick);
        }
        else
        {
            NEAR(output.leg[0].Tp, LEG_MOTION_LEG_WEIGHT_N * input.leg[0].length_m * 0.5f *
                                   sinf(input.leg[0].phi0_rad - LEG_MOTION_PI * 0.5f));
        }
        NEAR(output.leg[0].F0, -LEG_MOTION_LOWER_LEG_WEIGHT_N *
                               cosf(input.leg[0].phi0_rad - LEG_MOTION_PI * 0.5f));
    }
    NEAR(context.motion[0].command.target_phi0_rad, SPIN_TARGET_PHI0_RAD);
    retract_start_length = context.motion[0].start_length_m;
    for (step = 1U; step <= 10U; ++step)
    {
        float fraction = (float)step / 10.0f;
        input = Input(SPIN_TARGET_PHI0_RAD,
                      retract_start_length + (SPIN_RETRACT_LENGTH - retract_start_length) * fraction);
        now = SPIN_SWING_ANGLE_TIME_MS + SPIN_RETRACT_TIME_MS * step / 10U;
        CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
        CHECK(context.phase[0] == (step == 10U ? CHASSIS_RECOVERY_WAIT_ALIGN : CHASSIS_RECOVERY_RETRACT));
        NEAR(context.motion[0].reference_length_m, input.leg[0].length_m);
        NEAR(context.motion[0].command.target_phi0_rad, SPIN_TARGET_PHI0_RAD);
    }
    now = SPIN_SWING_ANGLE_TIME_MS + SPIN_RETRACT_TIME_MS + 1U;
    CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.motion[0].command.direction == LEG_MOTION_NEGATIVE);
    CHECK(context.motion[0].start_tick == now && context.motion[1].start_tick == now);
    for (step = 1U; step <= 10U; ++step)
    {
        float fraction = (float)step / 10.0f;
        input = Input(SPIN_TARGET_PHI0_RAD + (SPIN_ALIGN_TARGET_PHI0_RAD - SPIN_TARGET_PHI0_RAD) * fraction,
                      SPIN_RETRACT_LENGTH);
        CHECK(ChassisRecovery_Run(&context, &input, now + SPIN_ALIGN_TIME_MS * step / 10U,
                                  &output) == (step == 10U ? CHASSIS_RECOVERY_HANDOFF : CHASSIS_RECOVERY_RUNNING));
        NEAR(context.motion[0].reference_length_m, SPIN_RETRACT_LENGTH);
        NEAR(context.motion[0].reference_angle_rad, context.motion[0].actual_angle_rad);
    }
    now += SPIN_ALIGN_TIME_MS;
    CHECK(ChassisRecovery_Run(&context, &input, now + 1U, &output) == CHASSIS_RECOVERY_DONE);
    CHECK(ChassisRecovery_Run(&context, &input, now + SPIN_SCAN_TIMEOUT_MS, &output) == CHASSIS_RECOVERY_DONE);
    input.leg[1].length_m = NAN;
    CHECK(ChassisRecovery_Run(&context, &input, now + SPIN_SCAN_TIMEOUT_MS + 1U, &output) == CHASSIS_RECOVERY_FAULT);
    Zero(&output);
}

static void EarlyWindowQuery(void)
{
    const float lower = SPIN_RETRACT_PHI0_MIN_RAD;
    const float upper = SPIN_RETRACT_PHI0_MAX_RAD;
    const float angles[] = {lower, lower + 0.0001f, 1.2f, SPIN_ALIGN_TARGET_PHI0_RAD,
                            2.1f, upper - 0.0001f, upper};
    unsigned int index;
    int turn;
    for (index = 0U; index < sizeof(angles) / sizeof(angles[0]); ++index)
    {
        CHECK(ChassisRecovery_InEarlyWindow(angles[index]));
        for (turn = -2; turn <= 2; ++turn)
        {
            if (turn != 0)
                CHECK(!ChassisRecovery_InEarlyWindow(angles[index] + 2.0f * LEG_MOTION_PI * (float)turn));
        }
    }
    CHECK(!ChassisRecovery_InEarlyWindow(lower - 0.0001f));
    CHECK(!ChassisRecovery_InEarlyWindow(upper + 0.0001f));
    CHECK(!ChassisRecovery_InEarlyWindow(NAN));
    CHECK(!ChassisRecovery_InEarlyWindow(INFINITY));
    CHECK(!ChassisRecovery_InEarlyWindow(-INFINITY));
}

static void SweepArrivalAndEarlyWindow(void)
{
    ChassisRecovery_Context context = {0};
    ChassisRecovery_Input input = Input(-0.57f, 0.20f);
    ChassisRecovery_Output output;
    float angles[] = {SPIN_RETRACT_PHI0_MIN_RAD - 0.0001f,
                      SPIN_RETRACT_PHI0_MIN_RAD,
                      SPIN_RETRACT_PHI0_MAX_RAD,
                      SPIN_RETRACT_PHI0_MAX_RAD + 0.0001f};
    unsigned int index;
    CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
    input = Input(SPIN_TARGET_PHI0_RAD + 0.01f, 0.20f);
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_SWING_ANGLE_TIME_MS - 1U, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING); /* At angle, but ramp still active. */
    input = Input(SPIN_TARGET_PHI0_RAD + 0.1f, SPIN_SCAN_LENGTH);
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_SWING_ANGLE_TIME_MS, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.motion[0].result == LEG_MOTION_DONE); /* Generic DONE is too loose. */
    CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING);
    input = Input(SPIN_TARGET_PHI0_RAD + SPIN_ANGLE_WINDOW + 0.00001f, 0.20f);
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_SWING_ANGLE_TIME_MS + 1U, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING);
    input = Input(SPIN_TARGET_PHI0_RAD + SPIN_ANGLE_WINDOW - 0.00001f, 0.20f);
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_SWING_ANGLE_TIME_MS + 2U, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_RETRACT); /* Full extension is not required. */
    NEAR(context.motion[0].command.target_phi0_rad, input.leg[0].phi0_rad);
    NEAR(context.motion[0].start_length_m, 0.20f);
    for (index = 0U; index < sizeof(angles) / sizeof(angles[0]); ++index)
    {
        ChassisRecovery_Reset(&context);
        input = Input(angles[index], 0.22f);
        input.pitch_rad = 1.0f; /* Retraction range is independent of body pitch. */
        CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
        CHECK(context.phase[0] == ((index == 1U || index == 2U) ? CHASSIS_RECOVERY_RETRACT : CHASSIS_RECOVERY_SWING));
    }
    ChassisRecovery_Reset(&context);
    input = Input(SPIN_RETRACT_PHI0_MAX_RAD + 0.1f, 0.22f);
    input.pitch_rad = NAN; /* Invalid pitch cannot prevent range entry during SWING. */
    CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
    input = Input(1.2f, 0.23f);
    CHECK(ChassisRecovery_Run(&context, &input, 100U, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_RETRACT); /* Early window bypasses the ramp. */
    NEAR(context.motion[0].command.target_phi0_rad, 1.2f);
    input = Input(1.9f, 0.20f);
    CHECK(ChassisRecovery_Run(&context, &input, 200U, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_RETRACT);
    NEAR(context.motion[0].command.target_phi0_rad, 1.2f);
}

static void ActualArrivalGates(void)
{
    ChassisRecovery_Context context = {0};
    ChassisRecovery_Input input = Input(-0.57f, 0.20f);
    ChassisRecovery_Output output;
    uint32_t now;
    CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
    input = Input(SPIN_TARGET_PHI0_RAD, 0.20f);
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_SWING_ANGLE_TIME_MS, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_RETRACT); /* Angle alone permits normal retraction. */
    NEAR(context.motion[0].start_length_m, 0.20f);

    ChassisRecovery_Reset(&context);
    input = Input(-0.57f, SPIN_SCAN_LENGTH);
    CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
    input = Input(0.5f + 2.0f * LEG_MOTION_PI, SPIN_SCAN_LENGTH);
    CHECK(ChassisRecovery_Run(&context, &input, 1000U, &output) == CHASSIS_RECOVERY_RUNNING);
    input = Input(1.8f + 2.0f * LEG_MOTION_PI, SPIN_SCAN_LENGTH);
    CHECK(ChassisRecovery_Run(&context, &input, 2000U, &output) == CHASSIS_RECOVERY_RUNNING);
    input = Input(SPIN_TARGET_PHI0_RAD + 0.01f, SPIN_SCAN_LENGTH);
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_SWING_ANGLE_TIME_MS, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING); /* Same wrapped pose, wrong continuous route. */
    NEAR(context.motion[0].actual_angle_rad - context.motion[0].target_angle_rad, 2.0f * LEG_MOTION_PI + 0.01f);

    ChassisRecovery_Reset(&context);
    now = WaitAlign(&context, &input, &output, 0U) + 1U;
    CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
    input = Input(SPIN_ALIGN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH + 0.04f);
    now += SPIN_ALIGN_TIME_MS;
    CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_ALIGN);
    CHECK(context.motion[0].result == LEG_MOTION_RUNNING); /* ALIGN still requires length arrival. */
    input = Input(SPIN_ALIGN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
    CHECK(ChassisRecovery_Run(&context, &input, now + 1U, &output) == CHASSIS_RECOVERY_HANDOFF);
}

static void SynchronizedRangeRetraction(void)
{
    ChassisRecovery_Context context = {0};
    ChassisRecovery_Input input = Input(-0.57f, SPIN_SCAN_LENGTH);
    ChassisRecovery_Output output;
    uint32_t now;
    input.leg[0].phi0_rad = 1.2f;
    CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
    input.leg[0].length_m = SPIN_RETRACT_LENGTH;
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_RETRACT_TIME_MS, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING);
    CHECK(context.phase[1] == CHASSIS_RECOVERY_SWING);
    input.leg[0].phi0_rad = 1.4f;
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_RETRACT_TIME_MS + 1U, &output) == CHASSIS_RECOVERY_RUNNING);
    NEAR(context.motion[0].target_angle_rad, context.motion[0].actual_angle_rad);
    NEAR(context.motion[0].reference_length_m, SPIN_SCAN_LENGTH);
    CHECK(output.leg[0].Tp == 0.0f);
    input.leg[1].phi0_rad = SPIN_TARGET_PHI0_RAD;
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_SWING_ANGLE_TIME_MS, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[1] == CHASSIS_RECOVERY_RETRACT);
    now = SPIN_SWING_ANGLE_TIME_MS + SPIN_RETRACT_TIME_MS;
    CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[1] == CHASSIS_RECOVERY_RETRACT); /* Ramp over; actual length still long. */
    CHECK(context.phase[0] == CHASSIS_RECOVERY_WAIT_ALIGN);
    input.leg[1].length_m = SPIN_RETRACT_LENGTH;
    CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_WAIT_ALIGN && context.phase[1] == CHASSIS_RECOVERY_WAIT_ALIGN);
    input.leg[0].phi0_rad = 1.3f; /* Capture live feedback, not the old retract target. */
    CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_ALIGN && context.phase[1] == CHASSIS_RECOVERY_ALIGN);
    CHECK(context.motion[0].start_tick == now && context.motion[1].start_tick == now);
    CHECK(context.motion[0].command.direction == LEG_MOTION_POSITIVE);
    CHECK(context.motion[1].command.direction == LEG_MOTION_NEGATIVE);
    NEAR(context.motion[0].start_angle_rad, 1.3f - LEG_MOTION_PI * 0.5f);
    input = Input(SPIN_ALIGN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
    CHECK(ChassisRecovery_Run(&context, &input, now + SPIN_ALIGN_TIME_MS - 1U, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_ALIGN && context.phase[1] == CHASSIS_RECOVERY_ALIGN);
    input.leg[1].phi0_rad += 0.1f;
    CHECK(ChassisRecovery_Run(&context, &input, now + SPIN_ALIGN_TIME_MS, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_HOLD);
    CHECK(context.motion[1].result == LEG_MOTION_DONE);
    CHECK(context.phase[1] == CHASSIS_RECOVERY_ALIGN);
    CHECK(context.motion[0].command.direction == LEG_MOTION_POSITIVE);
    CHECK(context.motion[1].command.direction == LEG_MOTION_NEGATIVE);
    input.leg[1].phi0_rad = SPIN_ALIGN_TARGET_PHI0_RAD + SPIN_ANGLE_WINDOW + 0.00001f;
    CHECK(ChassisRecovery_Run(&context, &input, now + SPIN_ALIGN_TIME_MS + 1U, &output) == CHASSIS_RECOVERY_RUNNING);
    input.leg[1].phi0_rad = SPIN_ALIGN_TARGET_PHI0_RAD + SPIN_ANGLE_WINDOW - 0.00001f;
    CHECK(ChassisRecovery_Run(&context, &input, now + SPIN_ALIGN_TIME_MS + 2U, &output) == CHASSIS_RECOVERY_HANDOFF);
    CHECK(ChassisRecovery_Run(&context, &input, now + SPIN_ALIGN_TIME_MS + 3U, &output) == CHASSIS_RECOVERY_DONE);
}

static void AlignmentRoutes(void)
{
    const float target = SPIN_ALIGN_TARGET_PHI0_RAD;
    const float starts[] = {target - 0.5f, target + 1.1f, target, -3.0f,
                            target + 2.0f * LEG_MOTION_PI,
                            target - LEG_MOTION_PI, target + LEG_MOTION_PI};
    const float travels[] = {0.5f, -1.1f, 0.0f, target + 3.0f - 2.0f * LEG_MOTION_PI,
                             0.0f, -LEG_MOTION_PI, -LEG_MOTION_PI};
    unsigned int index, step;
    for (index = 0U; index < sizeof(starts) / sizeof(starts[0]); ++index)
    {
        ChassisRecovery_Context context = {0};
        ChassisRecovery_Input input;
        ChassisRecovery_Output output;
        uint32_t now = WaitAlign(&context, &input, &output, 0U) + 1U;
        input = Input(starts[index], SPIN_RETRACT_LENGTH);
        CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
        NEAR(context.motion[0].target_angle_rad - context.motion[0].start_angle_rad, travels[index]);
        if (travels[index] != 0.0f)
        {
            CHECK(context.motion[0].command.direction == (travels[index] < 0.0f ? LEG_MOTION_NEGATIVE : LEG_MOTION_POSITIVE));
        }
        for (step = 1U; step <= 10U; ++step)
        {
            float phi0 = starts[index] + travels[index] * (float)step / 10.0f;
            input = Input(atan2f(sinf(phi0), cosf(phi0)), SPIN_RETRACT_LENGTH);
            CHECK(ChassisRecovery_Run(&context, &input, now + SPIN_ALIGN_TIME_MS * step / 10U,
                                      &output) == (step == 10U ? CHASSIS_RECOVERY_HANDOFF : CHASSIS_RECOVERY_RUNNING));
        }
    }
}

static void ResetAndDisable(void)
{
    ChassisRecovery_Context context = {0};
    ChassisRecovery_Input input;
    ChassisRecovery_Output output;
    LegMotion_Context saved[2];
    uint32_t now = WaitAlign(&context, &input, &output, 100U) + 1U;
    CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
    memcpy(saved, context.motion, sizeof(saved));
    ChassisRecovery_Reset(&context);
    CHECK(context.result == CHASSIS_RECOVERY_IDLE);
    CHECK(memcmp(saved, context.motion, sizeof(saved)) == 0);
    input = Input(1.4f, 0.22f);
    CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_RUNNING);
    NEAR(context.motion[0].reference_length_m, 0.22f);
    NEAR(context.motion[0].actual_angle_rad, 1.4f - LEG_MOTION_PI * 0.5f);
    input.leg[0].angular_velocity_rad_s = NAN;
    CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_FAULT);
    memcpy(saved, context.motion, sizeof(saved));
    input.enabled = 0U;
    CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_IDLE);
    Zero(&output);
    CHECK(memcmp(saved, context.motion, sizeof(saved)) == 0);
    input = Input(-0.57f, 0.25f);
    CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING);
    CHECK(context.motion[0].start_tick == now);
    NEAR(context.motion[0].reference_length_m, 0.25f);
}

static void DeadlinesAndWrap(void)
{
    ChassisRecovery_Context context = {0};
    ChassisRecovery_Input input = Input(-0.57f, SPIN_SCAN_LENGTH);
    ChassisRecovery_Output output;
    uint32_t now;
    CHECK(ChassisRecovery_Run(&context, &input, 100U, &output) == CHASSIS_RECOVERY_RUNNING);
    input = Input(SPIN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
    now = 100U + SPIN_SCAN_TIMEOUT_MS - SPIN_RETRACT_TIME_MS - 100U;
    CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.motion[0].command.timeout_ms == SPIN_RETRACT_TIME_MS + 100U);
    now += SPIN_RETRACT_TIME_MS;
    CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_WAIT_ALIGN);
    CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.motion[0].command.timeout_ms == 99U);
    CHECK(context.motion[0].start_tick == now);
    CHECK(ChassisRecovery_Run(&context, &input, 100U + SPIN_SCAN_TIMEOUT_MS, &output) == CHASSIS_RECOVERY_FAULT);
    Zero(&output);
    ChassisRecovery_Reset(&context);
    now = Handoff(&context, &input, &output, 0U);
    input.pitch_rad = SPIN_HANDOFF_PITCH_MAX_RAD;
    CHECK(ChassisRecovery_Run(&context, &input, now + SPIN_HANDOFF_TIMEOUT_MS - 1U, &output) == CHASSIS_RECOVERY_HANDOFF);
    CHECK(ChassisRecovery_Run(&context, &input, now + SPIN_HANDOFF_TIMEOUT_MS, &output) == CHASSIS_RECOVERY_FAULT);
    Zero(&output);
    ChassisRecovery_Reset(&context);
    now = Handoff(&context, &input, &output, UINT32_MAX - 200U);
    CHECK(ChassisRecovery_Run(&context, &input, now + SPIN_HANDOFF_TIMEOUT_MS, &output) == CHASSIS_RECOVERY_DONE);
    ChassisRecovery_Reset(&context);
    input = Input(-0.57f, 0.20f);
    now = UINT32_MAX - 100U;
    CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(ChassisRecovery_Run(&context, &input, now + SPIN_SCAN_TIMEOUT_MS, &output) == CHASSIS_RECOVERY_FAULT);
    Zero(&output);
}

static void HandoffGates(void)
{
    unsigned int gate;
    for (gate = 0U; gate < 10U; ++gate)
    {
        ChassisRecovery_Context context = {0};
        ChassisRecovery_Input input;
        ChassisRecovery_Output output;
        uint32_t now = Handoff(&context, &input, &output, 0U);
        switch (gate)
        {
            case 0U: input.pitch_rad = SPIN_HANDOFF_PITCH_MAX_RAD; break;
            case 1U: input.pitch_rad = -SPIN_HANDOFF_PITCH_MAX_RAD; break;
            case 2U: input.pitch_rad = NAN; break;
            case 3U: input.leg[0].phi0_rad = 2.5001f; break;
            case 4U: input.leg[1].phi0_rad -= 2.0f * LEG_MOTION_PI; break;
            case 5U: input.leg[0].length_m += SPIN_LENGTH_TOLERANCE + 0.00001f; break;
            case 6U: input.leg[0].length_velocity_m_s = 0.05f; break;
            case 7U: input.leg[1].length_velocity_m_s = -0.05f; break;
            case 8U: input.leg[0].phi0_rad += SPIN_ANGLE_WINDOW + 0.00001f; break;
            default: input.leg[1].phi0_rad -= SPIN_ANGLE_WINDOW + 0.00001f; break;
        }
        CHECK(ChassisRecovery_Run(&context, &input, now + 1U, &output) == CHASSIS_RECOVERY_HANDOFF);
        input = Input(SPIN_ALIGN_TARGET_PHI0_RAD, SPIN_RETRACT_LENGTH);
        input.pitch_rad = SPIN_HANDOFF_PITCH_MAX_RAD - 0.001f;
        input.leg[0].phi0_rad += SPIN_ANGLE_WINDOW - 0.00001f;
        input.leg[1].length_velocity_m_s = 0.049f;
        CHECK(ChassisRecovery_Run(&context, &input, now + 2U, &output) == CHASSIS_RECOVERY_DONE);
    }
}

static void FailureOrderingAndLatching(void)
{
    unsigned int stage, failed_leg;
    for (stage = 0U; stage < 8U; ++stage)
    {
        for (failed_leg = 0U; failed_leg < 2U; ++failed_leg)
        {
            ChassisRecovery_Context context = {0};
            ChassisRecovery_Input input = Input(-0.57f, 0.20f);
            ChassisRecovery_Output output;
            LegMotion_Context saved_right;
            ChassisRecovery_Phase saved_phase;
            uint32_t now = 0U;
            if (stage < 3U)
            {
                if (stage != 0U) input = Input(1.2f, 0.20f);
                CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
                if (stage == 2U)
                {
                    input.leg[0].length_m = SPIN_RETRACT_LENGTH;
                    now = SPIN_RETRACT_TIME_MS;
                    CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
                    CHECK(context.phase[0] == CHASSIS_RECOVERY_WAIT_ALIGN);
                }
            }
            else if (stage < 6U)
            {
                now = WaitAlign(&context, &input, &output, 0U);
                if (stage >= 4U)
                {
                    CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_RUNNING);
                }
                if (stage == 5U)
                {
                    input.leg[0].phi0_rad = SPIN_ALIGN_TARGET_PHI0_RAD;
                    now += SPIN_ALIGN_TIME_MS;
                    CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
                    CHECK(context.phase[0] == CHASSIS_RECOVERY_HOLD);
                }
            }
            else
            {
                now = Handoff(&context, &input, &output, 0U);
                if (stage == 7U)
                {
                    CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_DONE);
                }
            }
            memcpy(&saved_right, &context.motion[1], sizeof(saved_right));
            saved_phase = context.phase[1];
            input.leg[failed_leg].angular_velocity_rad_s = NAN;
            CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_FAULT);
            Zero(&output);
            if (failed_leg == 0U)
            {
                CHECK(memcmp(&saved_right, &context.motion[1], sizeof(saved_right)) == 0);
                CHECK(context.phase[1] == saved_phase);
            }
            input.leg[failed_leg].angular_velocity_rad_s = 0.0f;
            CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_FAULT);
            Zero(&output);
            ChassisRecovery_Reset(&context);
            CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_RUNNING);
        }
    }
}

static void InvalidPointers(void)
{
    ChassisRecovery_Context context = {0};
    ChassisRecovery_Input input = Input(1.2f, 0.20f);
    ChassisRecovery_Output output;
    CHECK(ChassisRecovery_Run(NULL, &input, 0U, &output) == CHASSIS_RECOVERY_FAULT);
    Zero(&output);
    CHECK(ChassisRecovery_Run(&context, NULL, 0U, &output) == CHASSIS_RECOVERY_FAULT);
    Zero(&output);
    CHECK(ChassisRecovery_Run(&context, &input, 0U, NULL) == CHASSIS_RECOVERY_FAULT);
    ChassisRecovery_Reset(NULL);
}

static void SynchronizedRetraction(void)
{
    unsigned int first;
    for (first = 0U; first < 2U; ++first)
    {
        unsigned int other = 1U - first;
        ChassisRecovery_Context context = {0};
        ChassisRecovery_Input input = Input(-0.57f, 0.25f);
        ChassisRecovery_Output output;
        uint32_t origin = first == 0U ? 100U : UINT32_MAX - 200U;
        uint32_t now = origin + SPIN_SWING_ANGLE_TIME_MS;
        CHECK(ChassisRecovery_Run(&context, &input, origin, &output) == CHASSIS_RECOVERY_RUNNING);
        input.leg[first].phi0_rad = SPIN_TARGET_PHI0_RAD + 0.01f; /* Outside raw range; angle tolerance permits readiness. */
        CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
        CHECK(context.phase[first] == CHASSIS_RECOVERY_SWING && context.phase[other] == CHASSIS_RECOVERY_SWING);
        CHECK(context.sweep_arrived[first] == 1U && context.sweep_arrived[other] == 0U);
        CHECK(context.motion[first].start_tick == now);
        NEAR(context.motion[first].reference_length_m, 0.25f);
        CHECK(output.leg[first].Tp == 0.0f);
        /* Readiness is latched while the partner must still satisfy its own strict tolerance. */
        input.leg[first].phi0_rad += 0.1f;
        input.leg[other].phi0_rad = SPIN_TARGET_PHI0_RAD + 0.1f;
        CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_RUNNING);
        CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING && context.phase[1] == CHASSIS_RECOVERY_SWING);
        input.leg[other].phi0_rad = SPIN_TARGET_PHI0_RAD + SPIN_ANGLE_WINDOW + 0.00001f;
        CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_RUNNING);
        CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING && context.phase[1] == CHASSIS_RECOVERY_SWING);
        input.leg[other].phi0_rad = SPIN_TARGET_PHI0_RAD + SPIN_ANGLE_WINDOW - 0.00001f;
        input.leg[first].length_m = 0.27f;
        input.leg[other].length_m = 0.31f;
        CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_RUNNING);
        CHECK(context.phase[0] == CHASSIS_RECOVERY_RETRACT && context.phase[1] == CHASSIS_RECOVERY_RETRACT);
        CHECK(context.motion[0].start_tick == now && context.motion[1].start_tick == now);
        CHECK(context.motion[0].command.angle_control == LEG_MOTION_FREE &&
              context.motion[1].command.angle_control == LEG_MOTION_FREE);
        NEAR(context.motion[first].start_length_m, 0.27f);
        NEAR(context.motion[other].start_length_m, 0.31f);
        CHECK(output.leg[0].Tp == 0.0f && output.leg[1].Tp == 0.0f);
        {
            uint32_t start = now;
            input.leg[first].phi0_rad = 2.0f;
            input.leg[other].phi0_rad = -3.0f;
            input.leg[first].angular_velocity_rad_s = -3.0f;
            input.leg[other].angular_velocity_rad_s = 2.0f;
            now = start + SPIN_RETRACT_TIME_MS / 2U;
            CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
            CHECK(context.motion[0].start_tick == start && context.motion[1].start_tick == start);
            NEAR(context.motion[first].reference_length_m, (0.27f + SPIN_RETRACT_LENGTH) * 0.5f);
            NEAR(context.motion[other].reference_length_m, (0.31f + SPIN_RETRACT_LENGTH) * 0.5f);
            CHECK(output.leg[0].Tp == 0.0f && output.leg[1].Tp == 0.0f);
            input.leg[first].phi0_rad = 1.0f;
            input.leg[first].length_m = SPIN_RETRACT_LENGTH;
            input.leg[other].length_m = SPIN_RETRACT_LENGTH + SPIN_LENGTH_TOLERANCE + 0.00001f;
            now = start + SPIN_RETRACT_TIME_MS - 1U;
            CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
            CHECK(context.phase[first] == CHASSIS_RECOVERY_RETRACT);
            CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_RUNNING);
            CHECK(context.phase[first] == CHASSIS_RECOVERY_WAIT_ALIGN);
            CHECK(context.phase[other] == CHASSIS_RECOVERY_RETRACT);
            input.leg[first].phi0_rad = 0.2f;
            CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_RUNNING);
            CHECK(output.leg[0].Tp == 0.0f && output.leg[1].Tp == 0.0f);
            NEAR(context.motion[first].reference_angle_rad, context.motion[first].actual_angle_rad);
            input.leg[other].length_m = SPIN_RETRACT_LENGTH;
            CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_RUNNING);
            CHECK(context.phase[0] == CHASSIS_RECOVERY_WAIT_ALIGN && context.phase[1] == CHASSIS_RECOVERY_WAIT_ALIGN);
            CHECK(output.leg[0].Tp == 0.0f && output.leg[1].Tp == 0.0f);
            input.leg[first].phi0_rad = 1.1f;
            input.leg[other].phi0_rad = 2.9f;
            CHECK(ChassisRecovery_Run(&context, &input, ++now, &output) == CHASSIS_RECOVERY_RUNNING);
            CHECK(context.phase[0] == CHASSIS_RECOVERY_ALIGN && context.phase[1] == CHASSIS_RECOVERY_ALIGN);
            CHECK(context.motion[0].start_tick == now && context.motion[1].start_tick == now);
            CHECK(context.motion[first].command.direction == LEG_MOTION_POSITIVE);
            CHECK(context.motion[other].command.direction == LEG_MOTION_NEGATIVE);
            CHECK(context.motion[0].command.angle_control == LEG_MOTION_POSITION &&
                  context.motion[1].command.angle_control == LEG_MOTION_POSITION);
            NEAR(context.motion[first].start_angle_rad, 1.1f - LEG_MOTION_PI * 0.5f);
            NEAR(context.motion[other].start_angle_rad, 2.9f - LEG_MOTION_PI * 0.5f);
            CHECK(fabsf(output.leg[first].Tp) > 0.1f && fabsf(output.leg[other].Tp) > 0.1f);
        }
    }
}

static void RangeWaitingBothSides(void)
{
    unsigned int early, same_cycle;
    for (early = 0U; early < 2U; ++early)
    {
        for (same_cycle = 0U; same_cycle < 2U; ++same_cycle)
        {
            unsigned int normal = 1U - early;
            ChassisRecovery_Context context = {0};
            ChassisRecovery_Input input = Input(-0.57f, 0.25f);
            ChassisRecovery_Output output;
            uint32_t now = SPIN_SWING_ANGLE_TIME_MS;
            CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
            if (same_cycle == 0U)
            {
                input.leg[early].phi0_rad = 1.2f;
                CHECK(ChassisRecovery_Run(&context, &input, 100U, &output) == CHASSIS_RECOVERY_RUNNING);
                CHECK(context.phase[early] == CHASSIS_RECOVERY_SWING);
                CHECK(context.phase[normal] == CHASSIS_RECOVERY_SWING);
                CHECK(context.sweep_arrived[early] == 1U);
                NEAR(context.motion[early].reference_length_m, 0.25f);
                CHECK(output.leg[early].Tp == 0.0f);
            }
            else
            {
                input.leg[normal].phi0_rad = SPIN_TARGET_PHI0_RAD;
                CHECK(ChassisRecovery_Run(&context, &input, now++, &output) == CHASSIS_RECOVERY_RUNNING);
                CHECK(context.phase[normal] == CHASSIS_RECOVERY_SWING);
            }
            input.leg[early].phi0_rad = same_cycle ? 1.2f : -0.1f; /* Ready leg may drift outside the range. */
            input.leg[normal].phi0_rad = SPIN_TARGET_PHI0_RAD;
            CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
            CHECK(context.phase[early] == CHASSIS_RECOVERY_RETRACT);
            CHECK(context.phase[normal] == CHASSIS_RECOVERY_RETRACT);
            CHECK(context.motion[normal].start_tick == now);
            CHECK(context.motion[early].start_tick == now);
            CHECK(context.sweep_arrived[0] == 0U && context.sweep_arrived[1] == 0U);
            CHECK(output.leg[0].Tp == 0.0f && output.leg[1].Tp == 0.0f);
        }
    }
}

static void SweepBarrierFaults(void)
{
    unsigned int failed, field;
    for (failed = 0U; failed < 2U; ++failed)
    {
        for (field = 0U; field < 5U; ++field)
        {
            ChassisRecovery_Context context = {0};
            ChassisRecovery_Input input = Input(-0.57f, 0.25f);
            ChassisRecovery_Output output;
            LegMotion_Context saved;
            CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
            saved = context.motion[1];
            input = Input(SPIN_TARGET_PHI0_RAD, 0.25f);
            switch (field)
            {
                case 0U: input.leg[failed].phi0_rad = NAN; break;
                case 1U: input.leg[failed].angular_velocity_rad_s = NAN; break;
                case 2U: input.leg[failed].length_m = 0.0f; break;
                case 3U: input.leg[failed].length_velocity_m_s = NAN; break;
                default: input.leg[failed].phi0_rad = -0.57f + LEG_MOTION_PI; break;
            }
            CHECK(ChassisRecovery_Run(&context, &input, SPIN_SWING_ANGLE_TIME_MS, &output) == CHASSIS_RECOVERY_FAULT);
            Zero(&output);
            if (failed == 0U)
            {
                CHECK(memcmp(&saved, &context.motion[1], sizeof(saved)) == 0);
                CHECK(context.phase[1] == CHASSIS_RECOVERY_SWING);
            }
        }
    }
    {
        ChassisRecovery_Context context = {0};
        ChassisRecovery_Input input = Input(-0.57f, 0.25f);
        ChassisRecovery_Output output;
        CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
        input.leg[0].phi0_rad = SPIN_TARGET_PHI0_RAD;
        CHECK(ChassisRecovery_Run(&context, &input, SPIN_SCAN_TIMEOUT_MS - 1U, &output) == CHASSIS_RECOVERY_RUNNING);
        CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING);
        CHECK(ChassisRecovery_Run(&context, &input, SPIN_SCAN_TIMEOUT_MS, &output) == CHASSIS_RECOVERY_FAULT);
        Zero(&output);
    }
}

static void UnifiedRangeLatching(void)
{
    ChassisRecovery_Context context = {0};
    ChassisRecovery_Input input;
    ChassisRecovery_Output output;
    const float pitches[] = {0.0f, 0.3001f, -0.3001f, 1.0f, -1.0f, NAN, INFINITY, -INFINITY};
    unsigned int index;
    for (index = 0U; index < sizeof(pitches) / sizeof(pitches[0]); ++index)
    {
        ChassisRecovery_Reset(&context);
        input = Input(SPIN_RETRACT_PHI0_MIN_RAD, 0.25f);
        input.leg[1].phi0_rad = SPIN_RETRACT_PHI0_MAX_RAD;
        input.leg[1].length_m = 0.30f;
        input.pitch_rad = pitches[index];
        CHECK(ChassisRecovery_Run(&context, &input, 100U, &output) == CHASSIS_RECOVERY_RUNNING);
        CHECK(context.phase[0] == CHASSIS_RECOVERY_RETRACT && context.phase[1] == CHASSIS_RECOVERY_RETRACT);
        CHECK(context.motion[0].start_tick == 100U && context.motion[1].start_tick == 100U);
        NEAR(context.motion[0].start_length_m, 0.25f);
        NEAR(context.motion[1].start_length_m, 0.30f);
        CHECK(output.leg[0].Tp == 0.0f && output.leg[1].Tp == 0.0f);
    }
    ChassisRecovery_Reset(&context);
    input = Input(-0.57f, 0.25f);
    input.leg[1].phi0_rad = SPIN_RETRACT_PHI0_MAX_RAD + 0.1f;
    input.pitch_rad = NAN;
    CHECK(ChassisRecovery_Run(&context, &input, 100U, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING && context.phase[1] == CHASSIS_RECOVERY_SWING);
    input.leg[0].phi0_rad = SPIN_RETRACT_PHI0_MIN_RAD;
    CHECK(ChassisRecovery_Run(&context, &input, 150U, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING && context.phase[1] == CHASSIS_RECOVERY_SWING);
    CHECK(context.sweep_arrived[0] == 1U && context.sweep_arrived[1] == 0U);
    NEAR(context.motion[0].reference_length_m, 0.25f);
    CHECK(context.motion[0].start_tick == 150U && context.motion[1].start_tick == 100U);
    CHECK(output.leg[0].Tp == 0.0f);
    input.leg[0].phi0_rad = SPIN_RETRACT_PHI0_MIN_RAD - 0.1f;
    input.leg[1].phi0_rad = SPIN_RETRACT_PHI0_MAX_RAD;
    CHECK(ChassisRecovery_Run(&context, &input, 200U, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_RETRACT && context.phase[1] == CHASSIS_RECOVERY_RETRACT);
    CHECK(context.motion[0].start_tick == 200U && context.motion[1].start_tick == 200U);
    CHECK(context.sweep_arrived[0] == 0U && context.sweep_arrived[1] == 0U);
    CHECK(output.leg[0].Tp == 0.0f && output.leg[1].Tp == 0.0f);
    input.leg[0].length_m = SPIN_RETRACT_LENGTH;
    CHECK(ChassisRecovery_Run(&context, &input, 200U + SPIN_RETRACT_TIME_MS, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_WAIT_ALIGN && context.phase[1] == CHASSIS_RECOVERY_RETRACT);
    CHECK(output.leg[0].Tp == 0.0f);
    input.leg[1].length_m = SPIN_RETRACT_LENGTH;
    CHECK(ChassisRecovery_Run(&context, &input, 201U + SPIN_RETRACT_TIME_MS, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_WAIT_ALIGN && context.phase[1] == CHASSIS_RECOVERY_WAIT_ALIGN);
    CHECK(output.leg[0].Tp == 0.0f && output.leg[1].Tp == 0.0f);
    CHECK(ChassisRecovery_Run(&context, &input, 202U + SPIN_RETRACT_TIME_MS, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_ALIGN && context.phase[1] == CHASSIS_RECOVERY_ALIGN);
    CHECK(context.motion[0].start_tick == context.motion[1].start_tick);

    ChassisRecovery_Reset(&context);
    input = Input(2.1f, 0.25f);
    CHECK(ChassisRecovery_Run(&context, &input, 300U, &output) == CHASSIS_RECOVERY_RUNNING);
    input.leg[0].phi0_rad = -0.57f;
    input.leg[1].phi0_rad = 2.6f;
    input.pitch_rad = 0.4f; /* Leaving the range cannot restart or revert retraction. */
    CHECK(ChassisRecovery_Run(&context, &input, 301U, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_RETRACT && context.phase[1] == CHASSIS_RECOVERY_RETRACT);
    CHECK(context.motion[0].start_tick == 300U && context.motion[1].start_tick == 300U);
    CHECK(output.leg[0].Tp == 0.0f && output.leg[1].Tp == 0.0f);

    ChassisRecovery_Reset(&context);
    input = Input(2.1f, 0.25f);
    CHECK(ChassisRecovery_Run(&context, &input, UINT32_MAX - 100U, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(ChassisRecovery_Run(&context, &input, UINT32_MAX - 100U + SPIN_SCAN_TIMEOUT_MS,
                              &output) == CHASSIS_RECOVERY_FAULT);
    Zero(&output);
}

static void RangeStartFaultOrdering(void)
{
    unsigned int failed, field;
    for (failed = 0U; failed < 2U; ++failed)
    {
        for (field = 0U; field < 4U; ++field)
        {
            ChassisRecovery_Context context = {0};
            ChassisRecovery_Input input = Input(2.1f, 0.25f);
            ChassisRecovery_Output output;
            LegMotion_Context right_before = context.motion[1];
            switch (field)
            {
                case 0U: input.leg[failed].phi0_rad = NAN; break;
                case 1U: input.leg[failed].angular_velocity_rad_s = NAN; break;
                case 2U: input.leg[failed].length_m = 0.0f; break;
                default: input.leg[failed].length_velocity_m_s = NAN; break;
            }
            CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_FAULT);
            Zero(&output);
            if (failed == 0U)
            {
                CHECK(memcmp(&right_before, &context.motion[1], sizeof(right_before)) == 0);
                CHECK(context.phase[1] == CHASSIS_RECOVERY_SWING);
            }
            else
            {
                CHECK(context.motion[0].command.angle_control == LEG_MOTION_FREE);
                CHECK(context.motion[0].result == (field == 0U ? LEG_MOTION_DONE : LEG_MOTION_RUNNING));
            }
        }
    }
}

static void NearTargetStartRepro(void)
{
    ChassisRecovery_Context context = {0};
    ChassisRecovery_Input input = Input(2.78f, SPIN_SCAN_LENGTH);
    ChassisRecovery_Output output;
    CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_RETRACT);
    CHECK(context.phase[1] == CHASSIS_RECOVERY_RETRACT);
    CHECK(output.leg[0].Tp == 0.0f && output.leg[1].Tp == 0.0f);
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_SWING_ANGLE_TIME_MS, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_RETRACT && context.phase[1] == CHASSIS_RECOVERY_RETRACT);
    CHECK(output.leg[0].Tp == 0.0f && output.leg[1].Tp == 0.0f);
}

static void SynchronizedWindowWaitRepro(void)
{
    ChassisRecovery_Context context = {0};
    ChassisRecovery_Input input = Input(1.6f, 0.25f);
    ChassisRecovery_Output output;
    input.leg[1].phi0_rad = -1.0f;
    CHECK(ChassisRecovery_Run(&context, &input, 10U, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING);
    CHECK(context.phase[1] == CHASSIS_RECOVERY_SWING);
    CHECK(context.motion[0].command.angle_control == LEG_MOTION_FREE);
    NEAR(context.motion[0].reference_length_m, 0.25f);
    CHECK(output.leg[0].Tp == 0.0f);
}

static void ReadyLengthHoldAndReset(void)
{
    const float poses[] = {SPIN_RETRACT_PHI0_MIN_RAD, SPIN_ALIGN_TARGET_PHI0_RAD,
                           2.5f, 2.78f, SPIN_RETRACT_PHI0_MAX_RAD};
    unsigned int ready, index;
    for (ready = 0U; ready < 2U; ++ready)
    {
        for (index = 0U; index < sizeof(poses) / sizeof(poses[0]); ++index)
        {
            unsigned int other = 1U - ready;
            ChassisRecovery_Context context = {0};
            ChassisRecovery_Input input = Input(-1.0f, 0.31f);
            ChassisRecovery_Output output;
            uint32_t origin = UINT32_MAX - 100U;
            input.leg[ready].phi0_rad = poses[index];
            input.leg[ready].length_m = 0.25f;
            CHECK(ChassisRecovery_Run(&context, &input, origin, &output) == CHASSIS_RECOVERY_RUNNING);
            CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING && context.phase[1] == CHASSIS_RECOVERY_SWING);
            CHECK(context.sweep_arrived[ready] == 1U && context.sweep_arrived[other] == 0U);
            CHECK(context.motion[ready].command.angle_control == LEG_MOTION_FREE);
            CHECK(context.motion[ready].command.angle_duration_ms == 0U);
            CHECK(context.motion[ready].command.length_duration_ms == 0U);
            NEAR(context.motion[ready].command.length_m, 0.25f);
            CHECK(output.leg[ready].Tp == 0.0f);
            /* Drift outside the raw range and change actual length: reference stays captured. */
            input.leg[ready].phi0_rad = poses[index] + 1.0f;
            input.leg[ready].angular_velocity_rad_s = 3.0f;
            input.leg[ready].length_m = 0.24f;
            CHECK(ChassisRecovery_Run(&context, &input, origin + 100U, &output) == CHASSIS_RECOVERY_RUNNING);
            CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING && context.phase[1] == CHASSIS_RECOVERY_SWING);
            CHECK(context.motion[ready].start_tick == origin);
            NEAR(context.motion[ready].reference_length_m, 0.25f);
            CHECK(output.leg[ready].Tp == 0.0f);
            input.leg[other].phi0_rad = SPIN_RETRACT_PHI0_MIN_RAD;
            CHECK(ChassisRecovery_Run(&context, &input, origin + 101U, &output) == CHASSIS_RECOVERY_RUNNING);
            CHECK(context.phase[0] == CHASSIS_RECOVERY_RETRACT && context.phase[1] == CHASSIS_RECOVERY_RETRACT);
            CHECK(context.motion[0].start_tick == origin + 101U && context.motion[1].start_tick == origin + 101U);
            CHECK(context.sweep_arrived[0] == 0U && context.sweep_arrived[1] == 0U);
            NEAR(context.motion[ready].start_length_m, 0.24f);
            NEAR(context.motion[other].start_length_m, 0.31f);
            CHECK(output.leg[0].Tp == 0.0f && output.leg[1].Tp == 0.0f);
            input.enabled = 0U;
            CHECK(ChassisRecovery_Run(&context, &input, origin + 102U, &output) == CHASSIS_RECOVERY_IDLE);
            Zero(&output);
            CHECK(context.sweep_arrived[0] == 0U && context.sweep_arrived[1] == 0U);
            input.enabled = 1U;
            input.leg[other].phi0_rad = -1.0f;
            input.leg[ready].phi0_rad = poses[index];
            input.leg[ready].length_m = 0.27f;
            CHECK(ChassisRecovery_Run(&context, &input, origin + 103U, &output) == CHASSIS_RECOVERY_RUNNING);
            CHECK(context.sweep_arrived[ready] == 1U && context.sweep_arrived[other] == 0U);
            NEAR(context.motion[ready].reference_length_m, 0.27f);
            CHECK(context.motion[ready].start_tick == origin + 103U);
            CHECK(ChassisRecovery_Run(&context, &input, origin + 103U + SPIN_SCAN_TIMEOUT_MS, &output) == CHASSIS_RECOVERY_FAULT);
            Zero(&output);
            ChassisRecovery_Reset(&context);
            CHECK(context.sweep_arrived[0] == 0U && context.sweep_arrived[1] == 0U);
        }
    }
}

static void WaitingFaultBeforeRetraction(void)
{
    unsigned int failed, field;
    for (failed = 0U; failed < 2U; ++failed)
    {
        for (field = 0U; field < 5U; ++field)
        {
            ChassisRecovery_Context context = {0};
            ChassisRecovery_Input input = Input(-1.0f, 0.25f);
            ChassisRecovery_Output output;
            LegMotion_Context right_before;
            uint8_t right_ready;
            input.leg[0].phi0_rad = 2.78f;
            CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
            right_before = context.motion[1];
            right_ready = context.sweep_arrived[1];
            input.leg[1].phi0_rad = SPIN_RETRACT_PHI0_MIN_RAD;
            switch (field)
            {
                case 0U: input.leg[failed].phi0_rad = NAN; break;
                case 1U: input.leg[failed].angular_velocity_rad_s = NAN; break;
                case 2U: input.leg[failed].length_m = 0.0f; break;
                case 3U: input.leg[failed].length_velocity_m_s = NAN; break;
                default:
                    input.leg[failed].phi0_rad = (failed == 0U ? 2.78f : -1.0f) + LEG_MOTION_PI;
                    break;
            }
            CHECK(ChassisRecovery_Run(&context, &input, 100U, &output) == CHASSIS_RECOVERY_FAULT);
            CHECK(context.motion[failed].result == LEG_MOTION_INVALID);
            Zero(&output);
            if (failed == 0U)
            {
                CHECK(memcmp(&right_before, &context.motion[1], sizeof(right_before)) == 0);
                CHECK(context.sweep_arrived[1] == right_ready);
                CHECK(context.phase[1] == CHASSIS_RECOVERY_SWING);
            }
        }
    }
}

int main(void)
{
    ReadyLengthHoldAndReset();
    WaitingFaultBeforeRetraction();
    SynchronizedWindowWaitRepro();
    NearTargetStartRepro();
    UnifiedRangeLatching();
    RangeStartFaultOrdering();
    EarlyWindowQuery();
    SynchronizedRetraction();
    RangeWaitingBothSides();
    SweepBarrierFaults();
    SweepTargetTrajectory();
    SweepArrivalAndEarlyWindow();
    ActualArrivalGates();
    SynchronizedRangeRetraction();
    AlignmentRoutes();
    ResetAndDisable();
    DeadlinesAndWrap();
    HandoffGates();
    FailureOrderingAndLatching();
    InvalidPointers();
    printf("ChassisRecovery API verification passed (%u assertions).\n", checks);
    return 0;
}
