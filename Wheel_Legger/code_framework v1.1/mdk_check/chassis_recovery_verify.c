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

static void NormalTrajectory(void)
{
    ChassisRecovery_Context context = {0};
    ChassisRecovery_Input input = Input(-0.57f, SPIN_RETRACT_LENGTH);
    ChassisRecovery_Output output;
    const float travel = SPIN_TARGET_PHI0_RAD - 2.0f * LEG_MOTION_PI + 0.57f;
    unsigned int step;
    uint32_t now;

    CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
    NEAR(context.motion[0].target_angle_rad - context.motion[0].start_angle_rad, travel);
    for (step = 1U; step <= 40U; ++step)
    {
        float fraction = (float)step / 40.0f;
        float angle = -0.57f + travel * fraction;
        input = Input(atan2f(sinf(angle), cosf(angle)),
                      SPIN_RETRACT_LENGTH + (SPIN_SCAN_LENGTH - SPIN_RETRACT_LENGTH) * fraction);
        now = SPIN_RAMP_TIME_MS * step / 40U;
        CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
        CHECK(context.phase[0] == (step == 40U ? CHASSIS_RECOVERY_RETRACT : CHASSIS_RECOVERY_SWING));
        CHECK(context.phase[1] == context.phase[0]);
        NEAR(context.motion[0].reference_length_m, input.leg[0].length_m);
        NEAR(context.motion[0].reference_angle_rad, context.motion[0].actual_angle_rad);
        /* With exact tracking and zero measured rates, only gravity remains. */
        NEAR(output.leg[0].Tp, LEG_MOTION_LEG_WEIGHT_N * input.leg[0].length_m * 0.5f *
                               sinf(input.leg[0].phi0_rad - LEG_MOTION_PI * 0.5f));
        NEAR(output.leg[0].F0, -LEG_MOTION_LOWER_LEG_WEIGHT_N *
                               cosf(input.leg[0].phi0_rad - LEG_MOTION_PI * 0.5f));
    }
    NEAR(context.motion[0].command.target_phi0_rad, SPIN_TARGET_PHI0_RAD);
    for (step = 1U; step <= 10U; ++step)
    {
        float fraction = (float)step / 10.0f;
        input = Input(SPIN_TARGET_PHI0_RAD,
                      SPIN_SCAN_LENGTH + (SPIN_RETRACT_LENGTH - SPIN_SCAN_LENGTH) * fraction);
        now = SPIN_RAMP_TIME_MS + SPIN_RETRACT_TIME_MS * step / 10U;
        CHECK(ChassisRecovery_Run(&context, &input, now, &output) == CHASSIS_RECOVERY_RUNNING);
        CHECK(context.phase[0] == (step == 10U ? CHASSIS_RECOVERY_WAIT_ALIGN : CHASSIS_RECOVERY_RETRACT));
        NEAR(context.motion[0].reference_length_m, input.leg[0].length_m);
        NEAR(context.motion[0].command.target_phi0_rad, SPIN_TARGET_PHI0_RAD);
    }
    now = SPIN_RAMP_TIME_MS + SPIN_RETRACT_TIME_MS + 1U;
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

static void SweepArrivalAndEarlyWindow(void)
{
    ChassisRecovery_Context context = {0};
    ChassisRecovery_Input input = Input(-0.57f, 0.20f);
    ChassisRecovery_Output output;
    float angles[] = {LEG_MOTION_PI * 0.5f - SPIN_EARLY_ANGLE - 0.0001f,
                      LEG_MOTION_PI * 0.5f - SPIN_EARLY_ANGLE,
                      LEG_MOTION_PI * 0.5f,
                      LEG_MOTION_PI * 0.5f + 0.0001f};
    unsigned int index;
    CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
    input = Input(SPIN_TARGET_PHI0_RAD, 0.20f);
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_RAMP_TIME_MS - 1U, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING); /* At angle, but ramp still active. */
    input = Input(SPIN_TARGET_PHI0_RAD + 0.1f, SPIN_SCAN_LENGTH);
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_RAMP_TIME_MS, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.motion[0].result == LEG_MOTION_DONE); /* Generic DONE is too loose. */
    CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING);
    input = Input(SPIN_TARGET_PHI0_RAD + SPIN_ANGLE_WINDOW + 0.00001f, 0.20f);
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_RAMP_TIME_MS + 1U, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING);
    input = Input(SPIN_TARGET_PHI0_RAD + SPIN_ANGLE_WINDOW - 0.00001f, 0.20f);
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_RAMP_TIME_MS + 2U, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_RETRACT); /* Full extension is not required. */
    NEAR(context.motion[0].command.target_phi0_rad, input.leg[0].phi0_rad);
    NEAR(context.motion[0].start_length_m, 0.20f);
    for (index = 0U; index < sizeof(angles) / sizeof(angles[0]); ++index)
    {
        ChassisRecovery_Reset(&context);
        input = Input(angles[index], 0.22f);
        CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
        CHECK(context.phase[0] == ((index == 1U || index == 2U) ? CHASSIS_RECOVERY_RETRACT : CHASSIS_RECOVERY_SWING));
    }
    ChassisRecovery_Reset(&context);
    input = Input(1.8f, 0.22f);
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
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_RAMP_TIME_MS, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_RETRACT); /* Angle alone permits normal retraction. */
    NEAR(context.motion[0].start_length_m, 0.20f);

    ChassisRecovery_Reset(&context);
    input = Input(-0.57f, SPIN_SCAN_LENGTH);
    CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
    input = Input(0.5f, SPIN_SCAN_LENGTH);
    CHECK(ChassisRecovery_Run(&context, &input, 1000U, &output) == CHASSIS_RECOVERY_RUNNING);
    input = Input(1.8f, SPIN_SCAN_LENGTH);
    CHECK(ChassisRecovery_Run(&context, &input, 2000U, &output) == CHASSIS_RECOVERY_RUNNING);
    input = Input(SPIN_TARGET_PHI0_RAD, SPIN_SCAN_LENGTH);
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_RAMP_TIME_MS, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING); /* Same wrapped pose, wrong continuous route. */
    NEAR(context.motion[0].actual_angle_rad - context.motion[0].target_angle_rad, 2.0f * LEG_MOTION_PI);

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

static void IndependentRetraction(void)
{
    ChassisRecovery_Context context = {0};
    ChassisRecovery_Input input = Input(-0.57f, SPIN_SCAN_LENGTH);
    ChassisRecovery_Output output;
    float held;
    uint32_t now;
    input.leg[0].phi0_rad = 1.2f;
    CHECK(ChassisRecovery_Run(&context, &input, 0U, &output) == CHASSIS_RECOVERY_RUNNING);
    held = context.motion[0].target_angle_rad;
    input.leg[0].length_m = SPIN_RETRACT_LENGTH;
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_RETRACT_TIME_MS, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_WAIT_ALIGN);
    CHECK(context.phase[1] == CHASSIS_RECOVERY_SWING);
    input.leg[0].phi0_rad = 1.4f;
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_RETRACT_TIME_MS + 1U, &output) == CHASSIS_RECOVERY_RUNNING);
    NEAR(context.motion[0].target_angle_rad, held);
    NEAR(context.motion[0].reference_length_m, SPIN_RETRACT_LENGTH);
    CHECK(output.leg[0].Tp < -5.0f);
    input.leg[1].phi0_rad = SPIN_TARGET_PHI0_RAD;
    CHECK(ChassisRecovery_Run(&context, &input, SPIN_RAMP_TIME_MS, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[1] == CHASSIS_RECOVERY_RETRACT);
    now = SPIN_RAMP_TIME_MS + SPIN_RETRACT_TIME_MS;
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
    const float starts[] = {1.2f, 2.8f, 1.7f, -3.0f, 1.7f + 2.0f * LEG_MOTION_PI,
                            1.7f - LEG_MOTION_PI, 1.7f + LEG_MOTION_PI};
    const float travels[] = {0.5f, -1.1f, 0.0f, 4.7f - 2.0f * LEG_MOTION_PI, 0.0f,
                             -LEG_MOTION_PI, -LEG_MOTION_PI};
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
    input.pitch_rad = 0.2f;
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
            case 0U: input.pitch_rad = 0.2f; break;
            case 1U: input.pitch_rad = -0.2f; break;
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
        input.pitch_rad = 0.199f;
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
                if (stage != 0U) input.leg[0].phi0_rad = 1.2f;
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

int main(void)
{
    NormalTrajectory();
    SweepArrivalAndEarlyWindow();
    ActualArrivalGates();
    IndependentRetraction();
    AlignmentRoutes();
    ResetAndDisable();
    DeadlinesAndWrap();
    HandoffGates();
    FailureOrderingAndLatching();
    InvalidPointers();
    printf("ChassisRecovery API verification passed (%u assertions).\n", checks);
    return 0;
}
