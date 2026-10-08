#include "leg_motion.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned int checks;

#define CHECK(condition) do { \
    ++checks; \
    if (!(condition)) { \
        fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
        exit(1); \
    } \
} while (0)

#define NEAR(actual, expected) CHECK(fabsf((actual) - (expected)) < 0.0001f)

static LegMotion_Feedback Feedback(float phi0_rad, float length_m)
{
    LegMotion_Feedback feedback;
    feedback.phi0_rad = atan2f(sinf(phi0_rad), cosf(phi0_rad));
    feedback.angular_velocity_rad_s = 0.0f;
    feedback.length_m = length_m;
    feedback.length_velocity_m_s = 0.0f;
    return feedback;
}

static LegMotion_Command Command(LegMotion_Direction direction,
                                  float target_phi0_rad, uint32_t duration_ms,
                                  float length_m, uint32_t timeout_ms)
{
    LegMotion_Command command;
    command.direction = direction;
    command.target_phi0_rad = target_phi0_rad;
    command.duration_ms = duration_ms;
    command.length_m = length_m;
    command.timeout_ms = timeout_ms;
    return command;
}

static void AbsolutePath(float initial, float target,
                         LegMotion_Direction direction, float travel)
{
    LegMotion_Context context = {0};
    LegMotion_Output output;
    LegMotion_Command command = Command(direction, target, 4000U, 0.30f, 6000U);
    LegMotion_Feedback feedback = Feedback(initial, 0.13f);
    float origin;
    unsigned int step;

    CHECK(LegMotion_Run(&context, &command, &feedback, 100U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    origin = context.start_angle_rad;
    NEAR(context.target_angle_rad - origin, travel);
    NEAR(context.reference_angle_rad, origin);
    NEAR(context.reference_length_m, 0.13f);
    for (step = 1U; step <= 16U; ++step)
    {
        float fraction = (float)step / 16.0f;
        LegMotion_Result expected = LEG_MOTION_RUNNING;
        if (step == 16U)
        {
            expected = LEG_MOTION_DONE;
        }
        feedback = Feedback(initial + travel * fraction,
                            0.13f + 0.17f * fraction);
        CHECK(LegMotion_Run(&context, NULL, &feedback, 100U + step * 250U,
                            0U, &output) == expected);
        NEAR(context.actual_angle_rad, origin + travel * fraction);
        NEAR(context.reference_angle_rad, origin + travel * fraction);
        NEAR(context.reference_length_m, feedback.length_m);
    }
    NEAR(context.actual_angle_rad, context.target_angle_rad);
    CHECK(isfinite(output.F0) && isfinite(output.Tp));
}

static void SamePose(LegMotion_Direction direction, float turns)
{
    LegMotion_Context context = {0};
    LegMotion_Output output;
    LegMotion_Feedback feedback = Feedback(1.2f, 0.20f);
    LegMotion_Command command = Command(direction,
                                        1.2f + turns * 2.0f * LEG_MOTION_PI,
                                        1000U, 0.20f, 2000U);

    CHECK(LegMotion_Run(&context, &command, &feedback, 0U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    NEAR(context.target_angle_rad, context.start_angle_rad);
    CHECK(LegMotion_Run(&context, NULL, &feedback, 1000U, 0U, &output) ==
          LEG_MOTION_DONE);
    NEAR(context.actual_angle_rad, context.target_angle_rad);
}

static void EquivalentPoseRoundoff(void)
{
    const float initial[2] = {-0.42908f, 1.2f};
    const float turns[2] = {3.0f, 10.0f};
    unsigned int sample;
    int direction, turn_sign;
    for (sample = 0U; sample < 2U; ++sample)
    {
        for (direction = -1; direction <= 1; direction += 2)
        {
            for (turn_sign = -1; turn_sign <= 1; turn_sign += 2)
            {
                LegMotion_Context context = {0};
                LegMotion_Output output;
                LegMotion_Feedback feedback = Feedback(initial[sample], 0.20f);
                LegMotion_Command command;
                feedback.phi0_rad = initial[sample];
                command = Command((LegMotion_Direction)direction,
                    feedback.phi0_rad + (float)turn_sign * turns[sample] *
                        2.0f * LEG_MOTION_PI, 1000U, 0.20f, 2000U);
                CHECK(LegMotion_Run(&context, &command, &feedback, 0U, 1U, &output) ==
                      LEG_MOTION_RUNNING);
                CHECK(context.target_angle_rad == context.start_angle_rad);
                CHECK(LegMotion_Run(&context, NULL, &feedback, 1000U, 0U, &output) ==
                      LEG_MOTION_DONE);
            }
        }
    }
}

static void SmallDistinctTarget(float turns, float offset,
                                LegMotion_Direction direction)
{
    LegMotion_Context context = {0};
    LegMotion_Output output;
    LegMotion_Feedback feedback = Feedback(1.2f, 0.20f);
    LegMotion_Command command = Command(direction,
        1.2f + turns * 2.0f * LEG_MOTION_PI + offset, 1000U, 0.20f, 2000U);
    float travel;
    CHECK(LegMotion_Run(&context, &command, &feedback, 0U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    travel = context.target_angle_rad - context.start_angle_rad;
    if ((float)direction * offset > 0.0f)
    {
        CHECK(fabsf(travel) > fabsf(offset) * 0.5f);
        CHECK(fabsf(travel) < fabsf(offset) * 1.5f);
    }
    else
    {
        CHECK(fabsf(travel) > 6.0f);
    }
    CHECK((float)direction * travel > 0.0f);
}

static void CompletionAndHolding(void)
{
    LegMotion_Context context = {0};
    LegMotion_Output output;
    LegMotion_Command command = Command(LEG_MOTION_POSITIVE,
                                        3.0f, 1000U, 0.20f, 1500U);
    LegMotion_Feedback feedback = Feedback(LEG_MOTION_PI * 0.5f, 0.20f);
    float target;

    CHECK(LegMotion_Run(&context, &command, &feedback, 0U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    /* Elapsed ramp time alone cannot finish an unperformed movement. */
    CHECK(LegMotion_Run(&context, NULL, &feedback, 1000U, 0U, &output) ==
          LEG_MOTION_RUNNING);
    CHECK(output.Tp == LEG_MOTION_TORQUE_MAX);
    CHECK(LegMotion_Run(&context, NULL, &feedback, 1500U, 0U, &output) ==
          LEG_MOTION_TIMEOUT);
    NEAR(output.F0, 0.0f);
    NEAR(output.Tp, 0.0f);
    CHECK(LegMotion_Run(&context, NULL, &feedback, 1600U, 0U, &output) ==
          LEG_MOTION_TIMEOUT);

    command = Command(LEG_MOTION_NEGATIVE, feedback.phi0_rad,
                       1000U, 0.20f, 2000U);
    CHECK(LegMotion_Run(&context, &command, &feedback, 2000U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    CHECK(LegMotion_Run(&context, NULL, &feedback, 2500U, 0U, &output) ==
          LEG_MOTION_RUNNING);
    CHECK(LegMotion_Run(&context, NULL, &feedback, 3000U, 0U, &output) ==
          LEG_MOTION_DONE);
    target = context.target_angle_rad;

    /* Perturb a finished leg: it must hold the old angle and short length. */
    feedback = Feedback(LEG_MOTION_PI * 0.5f + 0.5f, 0.21f);
    CHECK(LegMotion_Run(&context, NULL, &feedback, 5000U, 0U, &output) ==
          LEG_MOTION_DONE);
    NEAR(context.target_angle_rad, target);
    NEAR(context.reference_length_m, 0.20f);
    CHECK(output.Tp < -5.0f);
    CHECK(output.F0 < -3.0f);

    feedback.phi0_rad = NAN;
    CHECK(LegMotion_Run(&context, NULL, &feedback, 5050U, 0U, &output) ==
          LEG_MOTION_INVALID);
    NEAR(output.F0, 0.0f);
    NEAR(output.Tp, 0.0f);
    feedback = Feedback(LEG_MOTION_PI * 0.5f + 0.5f, 0.21f);
    command = Command(LEG_MOTION_POSITIVE, feedback.phi0_rad + 1.5f,
                       500U, 0.30f, 1500U);
    CHECK(LegMotion_Run(&context, &command, &feedback, 5100U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    NEAR(context.reference_angle_rad, context.actual_angle_rad);
    NEAR(context.reference_length_m, 0.21f);
    NEAR(context.target_angle_rad - context.start_angle_rad, 1.5f);
}

static void LatchingAndIndependentLegs(void)
{
    LegMotion_Context left = {0};
    LegMotion_Context right = {0};
    LegMotion_Output output;
    LegMotion_Command command = Command(LEG_MOTION_POSITIVE,
                                        LEG_MOTION_PI, 1000U, 0.30f, 2000U);
    LegMotion_Feedback feedback = Feedback(LEG_MOTION_PI * 0.5f, 0.10f);

    CHECK(LegMotion_Run(&left, &command, &feedback, 10U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    command.target_phi0_rad = feedback.phi0_rad;
    command.length_m = 0.13f;
    CHECK(LegMotion_Run(&right, &command, &feedback, 20U, 1U, &output) ==
          LEG_MOTION_RUNNING);

    feedback = Feedback(LEG_MOTION_PI * 0.75f, 0.20f);
    CHECK(LegMotion_Run(&left, &command, &feedback, 510U, 0U, &output) ==
          LEG_MOTION_RUNNING);
    NEAR(left.reference_angle_rad, LEG_MOTION_PI * 0.25f);
    NEAR(left.reference_length_m, 0.20f);
    NEAR(left.command.target_phi0_rad, LEG_MOTION_PI);
    NEAR(left.command.length_m, 0.30f);
    NEAR(right.reference_length_m, 0.10f);

    feedback = Feedback(LEG_MOTION_PI * 0.5f, 0.13f);
    CHECK(LegMotion_Run(&right, NULL, &feedback, 1020U, 0U, &output) ==
          LEG_MOTION_DONE);
    NEAR(right.reference_length_m, 0.13f);
    NEAR(left.reference_length_m, 0.20f);

    /* Restarting the exact same command must reset the origin and clock. */
    CHECK(LegMotion_Run(&right, &command, &feedback, 1100U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    CHECK(right.start_tick == 1100U);
    feedback.length_m = 0.16f;
    CHECK(LegMotion_Run(&right, &command, &feedback, 1200U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    CHECK(right.start_tick == 1200U);
    NEAR(right.reference_length_m, 0.16f);
}

static void ClockAndInstantTargets(void)
{
    LegMotion_Context context = {0};
    LegMotion_Output output;
    LegMotion_Command command = Command(LEG_MOTION_POSITIVE,
                                        LEG_MOTION_PI * 0.5f,
                                        1000U, 0.30f, 2000U);
    LegMotion_Feedback feedback = Feedback(LEG_MOTION_PI * 0.5f, 0.10f);
    uint32_t origin = UINT32_MAX - 499U;

    CHECK(LegMotion_Run(&context, &command, &feedback, origin, 1U, &output) ==
          LEG_MOTION_RUNNING);
    feedback.length_m = 0.20f;
    CHECK(LegMotion_Run(&context, NULL, &feedback, origin + 500U, 0U, &output) ==
          LEG_MOTION_RUNNING);
    NEAR(context.reference_length_m, 0.20f);
    feedback.length_m = 0.30f;
    CHECK(LegMotion_Run(&context, NULL, &feedback, origin + 1000U, 0U, &output) ==
          LEG_MOTION_DONE);
    /* DONE must hold even if the elapsed uint32 counter later wraps again. */
    CHECK(LegMotion_Run(&context, NULL, &feedback, origin, 0U, &output) ==
          LEG_MOTION_DONE);
    NEAR(context.reference_length_m, 0.30f);

    command = Command(LEG_MOTION_POSITIVE, LEG_MOTION_PI, 0U, 0.50f, 500U);
    CHECK(LegMotion_Run(&context, &command, &feedback, 0U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    NEAR(context.reference_length_m, 0.50f);
    CHECK(output.F0 == LEG_MOTION_FORCE_MAX);
    CHECK(output.Tp == LEG_MOTION_TORQUE_MAX);
    feedback = Feedback(LEG_MOTION_PI, 0.50f);
    CHECK(LegMotion_Run(&context, NULL, &feedback, 500U, 0U, &output) ==
          LEG_MOTION_DONE); /* Arrival at the deadline wins over timeout. */
}

static void InvalidInputs(void)
{
    LegMotion_Context context = {0};
    LegMotion_Output output = {1.0f, 1.0f};
    LegMotion_Command good = Command(LEG_MOTION_POSITIVE,
                                     2.0f, 100U, 0.20f, 200U);
    LegMotion_Command command;
    LegMotion_Feedback feedback = Feedback(LEG_MOTION_PI * 0.5f, 0.20f);
    unsigned int field;

    CHECK(LegMotion_Run(&context, NULL, NULL, 0U, 0U, &output) ==
          LEG_MOTION_IDLE);
    NEAR(output.F0, 0.0f);
    NEAR(output.Tp, 0.0f);
    CHECK(LegMotion_Run(NULL, &good, &feedback, 0U, 1U, &output) ==
          LEG_MOTION_INVALID);
    CHECK(LegMotion_Run(&context, &good, &feedback, 0U, 1U, NULL) ==
          LEG_MOTION_INVALID);
    CHECK(LegMotion_Run(&context, NULL, &feedback, 0U, 1U, &output) ==
          LEG_MOTION_INVALID);
    CHECK(LegMotion_Run(&context, &good, NULL, 0U, 1U, &output) ==
          LEG_MOTION_INVALID);

    for (field = 0U; field < 8U; ++field)
    {
        command = good;
        switch (field)
        {
            case 0U: command.direction = (LegMotion_Direction)0; break;
            case 1U: command.target_phi0_rad = NAN; break;
            case 2U: command.target_phi0_rad = INFINITY; break;
            case 3U: command.target_phi0_rad = -INFINITY; break;
            case 4U: command.length_m = 0.0f; break;
            case 5U: command.length_m = -0.10f; break;
            case 6U: command.length_m = INFINITY; break;
            default: command.timeout_ms = 0U; break;
        }
        CHECK(LegMotion_Run(&context, &command, &feedback, 0U, 1U, &output) ==
              LEG_MOTION_INVALID);
        NEAR(output.F0, 0.0f);
        NEAR(output.Tp, 0.0f);
    }
    for (field = 0U; field < 5U; ++field)
    {
        feedback = Feedback(LEG_MOTION_PI * 0.5f, 0.20f);
        CHECK(LegMotion_Run(&context, &good, &feedback, 0U, 1U, &output) ==
              LEG_MOTION_RUNNING);
        switch (field)
        {
            case 0U: feedback.phi0_rad = NAN; break;
            case 1U: feedback.angular_velocity_rad_s = INFINITY; break;
            case 2U: feedback.length_m = NAN; break;
            case 3U: feedback.length_m = 0.0f; break;
            default: feedback.length_velocity_m_s = NAN; break;
        }
        CHECK(LegMotion_Run(&context, NULL, &feedback, 50U, 0U, &output) ==
              LEG_MOTION_INVALID);
        NEAR(output.F0, 0.0f);
        NEAR(output.Tp, 0.0f);
        feedback = Feedback(LEG_MOTION_PI * 0.5f, 0.20f);
        CHECK(LegMotion_Run(&context, NULL, &feedback, 60U, 0U, &output) ==
              LEG_MOTION_INVALID);
    }

    CHECK(LegMotion_Run(&context, &good, &feedback, 0U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    feedback = Feedback(LEG_MOTION_PI * 1.5f, 0.20f);
    CHECK(LegMotion_Run(&context, NULL, &feedback, 50U, 0U, &output) ==
          LEG_MOTION_INVALID);
    NEAR(output.F0, 0.0f);
    NEAR(output.Tp, 0.0f);
}

int main(void)
{
    /* Both routes across the VMC phi0 boundary at +/-pi. */
    AbsolutePath(3.0f, -3.0f, LEG_MOTION_POSITIVE, 2.0f * LEG_MOTION_PI - 6.0f);
    AbsolutePath(3.0f, -3.0f, LEG_MOTION_NEGATIVE, -6.0f);
    AbsolutePath(-3.0f, 3.0f, LEG_MOTION_NEGATIVE, 6.0f - 2.0f * LEG_MOTION_PI);
    AbsolutePath(-3.0f, 3.0f, LEG_MOTION_POSITIVE, 6.0f);
    /* The internal vertical-zero coordinate wraps at a different phi0. */
    AbsolutePath(-1.8f, -1.3f, LEG_MOTION_POSITIVE, 0.5f);
    AbsolutePath(-1.3f, -1.8f, LEG_MOTION_NEGATIVE, -0.5f);
    AbsolutePath(LEG_MOTION_PI, -LEG_MOTION_PI, LEG_MOTION_NEGATIVE, 0.0f);
    AbsolutePath(-LEG_MOTION_PI, LEG_MOTION_PI, LEG_MOTION_POSITIVE, 0.0f);
    SamePose(LEG_MOTION_POSITIVE, 0.0f);
    SamePose(LEG_MOTION_NEGATIVE, 0.0f);
    SamePose(LEG_MOTION_POSITIVE, 1.0f);
    SamePose(LEG_MOTION_NEGATIVE, 1.0f);
    SamePose(LEG_MOTION_POSITIVE, -1.0f);
    SamePose(LEG_MOTION_NEGATIVE, -1.0f);
    EquivalentPoseRoundoff();
    SmallDistinctTarget(0.0f, 0.00001f, LEG_MOTION_POSITIVE);
    SmallDistinctTarget(0.0f, -0.00001f, LEG_MOTION_NEGATIVE);
    SmallDistinctTarget(10.0f, 0.0001f, LEG_MOTION_POSITIVE);
    SmallDistinctTarget(10.0f, 0.0001f, LEG_MOTION_NEGATIVE);
    SmallDistinctTarget(10.0f, -0.0001f, LEG_MOTION_POSITIVE);
    SmallDistinctTarget(10.0f, -0.0001f, LEG_MOTION_NEGATIVE);
    CompletionAndHolding();
    LatchingAndIndependentLegs();
    ClockAndInstantTargets();
    InvalidInputs();
    printf("LegMotion verification passed (%u assertions).\n", checks);
    return 0;
}
