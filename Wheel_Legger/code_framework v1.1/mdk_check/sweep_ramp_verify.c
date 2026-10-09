#include "chassis_recovery.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned int checks;
#define CHECK(test) do { ++checks; if (!(test)) { \
    fprintf(stderr, "SWEEP RAMP FAIL line %d: %s\n", __LINE__, #test); exit(1); \
} } while (0)
#define NEAR(a, b) CHECK(fabsf((a) - (b)) < 0.00001f)

static float LengthAt(float start, uint32_t elapsed, uint32_t duration_ms)
{
    float part = duration_ms == 0U || elapsed >= duration_ms ?
                 1.0f : (float)elapsed / (float)duration_ms;
    return start + (SPIN_SCAN_LENGTH - start) * part;
}

static void SweepIntoRetractRange(void)
{
    ChassisRecovery_Context context = {0};
    ChassisRecovery_Input input = {0};
    ChassisRecovery_Output output;
    const float start_angle = SPIN_RETRACT_PHI0_MAX_RAD + 0.1f;
    const float start_lengths[] = {0.18f, 0.23f};
    float captured[2];
    uint32_t elapsed = 0U;
    uint32_t angle_time = SPIN_SWING_ANGLE_TIME_MS;
    uint32_t length_time = SPIN_SWING_LENGTH_TIME_MS;
    uint32_t origin = 100U;
    unsigned int leg, step;
    input.enabled = 1U;
    input.pitch_rad = NAN;
    for (leg = 0U; leg < 2U; ++leg)
    {
        input.leg[leg].phi0_rad = start_angle;
        input.leg[leg].length_m = start_lengths[leg];
    }
    CHECK(ChassisRecovery_Run(&context, &input, origin, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_SWING && context.phase[1] == CHASSIS_RECOVERY_SWING);
    for (step = 1U; step <= 100U; ++step)
    {
        float angle = angle_time == 0U ? SPIN_RETRACT_PHI0_MAX_RAD :
                      start_angle + (SPIN_RETRACT_PHI0_MAX_RAD - 0.1f - start_angle) *
                                    (float)step / 100.0f;
        int inside = angle >= SPIN_RETRACT_PHI0_MIN_RAD && angle <= SPIN_RETRACT_PHI0_MAX_RAD;
        elapsed = angle_time == 0U ? 1U : angle_time * step / 100U;
        for (leg = 0U; leg < 2U; ++leg)
        {
            input.leg[leg].phi0_rad = angle;
            input.leg[leg].length_m = LengthAt(start_lengths[leg], elapsed, length_time);
        }
        CHECK(ChassisRecovery_Run(&context, &input, origin + elapsed, &output) == CHASSIS_RECOVERY_RUNNING);
        for (leg = 0U; leg < 2U; ++leg)
        {
            CHECK(context.phase[leg] == (inside ? CHASSIS_RECOVERY_RETRACT : CHASSIS_RECOVERY_SWING));
            NEAR(context.motion[leg].reference_length_m, input.leg[leg].length_m);
            if (inside)
            {
                CHECK(output.leg[leg].Tp == 0.0f);
                CHECK(context.motion[leg].command.angle_control == LEG_MOTION_FREE);
                CHECK(context.motion[leg].start_tick == origin + elapsed);
                NEAR(context.motion[leg].start_length_m, input.leg[leg].length_m);
                captured[leg] = input.leg[leg].length_m;
                if (length_time > elapsed) CHECK(captured[leg] < SPIN_SCAN_LENGTH);
            }
            else
            {
                NEAR(context.motion[leg].reference_angle_rad, start_angle - LEG_MOTION_PI * 0.5f +
                     (SPIN_TARGET_PHI0_RAD - start_angle) * (float)step / 100.0f);
                CHECK(context.motion[leg].start_tick == origin);
            }
        }
        if (inside) break;
    }
    CHECK(step <= 100U);
    CHECK(elapsed < angle_time || angle_time == 0U);
    origin += elapsed;
    for (leg = 0U; leg < 2U; ++leg)
    {
        input.leg[leg].phi0_rad = SPIN_RETRACT_PHI0_MAX_RAD + 0.2f;
        input.leg[leg].length_m = (captured[leg] + SPIN_RETRACT_LENGTH) * 0.5f;
    }
    CHECK(ChassisRecovery_Run(&context, &input, origin + SPIN_RETRACT_TIME_MS / 2U, &output) == CHASSIS_RECOVERY_RUNNING);
    for (leg = 0U; leg < 2U; ++leg)
    {
        CHECK(context.phase[leg] == CHASSIS_RECOVERY_RETRACT);
        CHECK(context.motion[leg].start_tick == origin);
        NEAR(context.motion[leg].reference_length_m, input.leg[leg].length_m);
        CHECK(output.leg[leg].Tp == 0.0f);
        input.leg[leg].length_m = SPIN_RETRACT_LENGTH;
    }
    origin += SPIN_RETRACT_TIME_MS;
    CHECK(ChassisRecovery_Run(&context, &input, origin, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_WAIT_ALIGN && context.phase[1] == CHASSIS_RECOVERY_WAIT_ALIGN);
    CHECK(output.leg[0].Tp == 0.0f && output.leg[1].Tp == 0.0f);
    CHECK(ChassisRecovery_Run(&context, &input, ++origin, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_ALIGN && context.phase[1] == CHASSIS_RECOVERY_ALIGN);
    CHECK(context.motion[0].start_tick == origin && context.motion[1].start_tick == origin);
}

int main(void)
{
    ChassisRecovery_Context context = {0};
    ChassisRecovery_Input input = {0};
    ChassisRecovery_Output output;
    const float starts[] = {-0.57f, -0.70f};
    const float lengths[] = {0.18f, 0.23f};
    uint32_t origin = 100U;
    uint32_t elapsed = 0U;
    uint32_t length_time = SPIN_SWING_LENGTH_TIME_MS;
    unsigned int leg, step;
    input.enabled = 1U;
    input.pitch_rad = 0.3001f; /* Both raw leg angles start outside the retract range. */
    for (leg = 0U; leg < 2U; ++leg)
    {
        input.leg[leg].phi0_rad = starts[leg];
        input.leg[leg].length_m = lengths[leg];
    }
    CHECK(ChassisRecovery_Run(&context, &input, origin, &output) == CHASSIS_RECOVERY_RUNNING);
    for (leg = 0U; leg < 2U; ++leg)
    {
        CHECK(context.phase[leg] == CHASSIS_RECOVERY_SWING);
        CHECK(context.motion[leg].command.angle_duration_ms == SPIN_SWING_ANGLE_TIME_MS);
        CHECK(context.motion[leg].command.length_duration_ms == SPIN_SWING_LENGTH_TIME_MS);
        CHECK(context.motion[leg].start_tick == origin);
        NEAR(context.motion[leg].reference_length_m, LengthAt(lengths[leg], 0U, length_time));
    }
    /* The final angle sample must trigger retraction, regardless of length progress. */
    for (step = 1U; step <= 10U; ++step)
    {
        float part = (float)step / 10.0f;
        if (SPIN_SWING_ANGLE_TIME_MS == 0U)
        {
            step = 10U;
            part = 1.0f;
            elapsed = 1U;
        }
        else
        {
            elapsed = SPIN_SWING_ANGLE_TIME_MS * step / 10U;
        }
        for (leg = 0U; leg < 2U; ++leg)
        {
            input.leg[leg].phi0_rad = starts[leg] +
                (SPIN_TARGET_PHI0_RAD - 2.0f * LEG_MOTION_PI - starts[leg]) * part;
            input.leg[leg].length_m = LengthAt(lengths[leg], elapsed, length_time);
        }
        CHECK(ChassisRecovery_Run(&context, &input, origin + elapsed, &output) == CHASSIS_RECOVERY_RUNNING);
        for (leg = 0U; leg < 2U; ++leg)
        {
            if (step < 10U)
            {
                CHECK(context.phase[leg] == CHASSIS_RECOVERY_SWING);
                NEAR(context.motion[leg].reference_angle_rad,
                     input.leg[leg].phi0_rad - LEG_MOTION_PI * 0.5f);
                NEAR(context.motion[leg].reference_length_m, input.leg[leg].length_m);
            }
            else
            {
                CHECK(context.phase[leg] == CHASSIS_RECOVERY_RETRACT);
                CHECK(context.motion[leg].command.angle_control == LEG_MOTION_FREE);
                CHECK(context.motion[leg].command.angle_duration_ms == 0U);
                CHECK(context.motion[leg].command.length_duration_ms == SPIN_RETRACT_TIME_MS);
                CHECK(context.motion[leg].start_tick == origin + elapsed);
                NEAR(context.motion[leg].start_length_m, input.leg[leg].length_m);
                CHECK(output.leg[leg].Tp == 0.0f);
                if (length_time > elapsed)
                    CHECK(context.motion[leg].start_length_m < SPIN_SCAN_LENGTH - 0.01f);
            }
        }
    }
    origin += elapsed + SPIN_RETRACT_TIME_MS;
    input.pitch_rad = 0.0f;
    for (leg = 0U; leg < 2U; ++leg)
    {
        input.leg[leg].phi0_rad = leg == 0U ? 1.3f : 1.9f;
        input.leg[leg].length_m = SPIN_RETRACT_LENGTH;
    }
    CHECK(ChassisRecovery_Run(&context, &input, origin, &output) == CHASSIS_RECOVERY_RUNNING);
    CHECK(context.phase[0] == CHASSIS_RECOVERY_WAIT_ALIGN && context.phase[1] == CHASSIS_RECOVERY_WAIT_ALIGN);
    CHECK(output.leg[0].Tp == 0.0f && output.leg[1].Tp == 0.0f);
    CHECK(ChassisRecovery_Run(&context, &input, ++origin, &output) == CHASSIS_RECOVERY_RUNNING);
    for (leg = 0U; leg < 2U; ++leg)
    {
        CHECK(context.phase[leg] == CHASSIS_RECOVERY_ALIGN);
        CHECK(context.motion[leg].start_tick == origin);
        CHECK(context.motion[leg].command.angle_duration_ms == SPIN_ALIGN_TIME_MS);
        CHECK(context.motion[leg].command.length_duration_ms == SPIN_ALIGN_TIME_MS);
    }
    CHECK(context.motion[0].command.direction == LEG_MOTION_POSITIVE);
    CHECK(context.motion[1].command.direction == LEG_MOTION_NEGATIVE);
    for (leg = 0U; leg < 2U; ++leg) input.leg[leg].phi0_rad = SPIN_ALIGN_TARGET_PHI0_RAD;
    origin += SPIN_ALIGN_TIME_MS;
    CHECK(ChassisRecovery_Run(&context, &input, origin, &output) == CHASSIS_RECOVERY_HANDOFF);
    CHECK(ChassisRecovery_Run(&context, &input, origin + 1U, &output) == CHASSIS_RECOVERY_DONE);
    SweepIntoRetractRange();
    printf("Sweep ramp verification passed (angle=%u ms, length=%u ms, %u assertions).\n",
           (unsigned int)SPIN_SWING_ANGLE_TIME_MS, (unsigned int)SPIN_SWING_LENGTH_TIME_MS, checks);
    return 0;
}
