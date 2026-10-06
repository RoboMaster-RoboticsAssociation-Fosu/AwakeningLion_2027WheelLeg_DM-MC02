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

static LegMotion_Feedback Feedback(float angle_deg, float length_m)
{
    float phi0 = (angle_deg + 90.0f) * LEG_MOTION_DEG_TO_RAD;
    LegMotion_Feedback feedback = {
        atan2f(sinf(phi0), cosf(phi0)), 0.0f, length_m, 0.0f
    };
    return feedback;
}

static LegMotion_Command Command(LegMotion_AngleMode mode,
                                  LegMotion_Direction direction,
                                  float angle_deg, uint32_t duration_ms,
                                  float length_m, uint32_t timeout_ms)
{
    LegMotion_Command command = {
        direction, mode, angle_deg, duration_ms, length_m, timeout_ms
    };
    return command;
}

static void FullTurns(LegMotion_Direction direction, float turns)
{
    LegMotion_Context context = {0};
    LegMotion_Output output;
    LegMotion_Command command = Command(LEG_MOTION_RELATIVE, direction,
                                        turns * 360.0f, 4000U, 0.30f, 6000U);
    LegMotion_Feedback feedback = Feedback(170.0f, 0.13f);
    float travel = (float)direction * turns * 360.0f;
    unsigned int step;

    CHECK(LegMotion_Run(&context, &command, &feedback, 100U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    for (step = 1U; step <= 16U; ++step)
    {
        float fraction = (float)step / 16.0f;
        feedback = Feedback(170.0f + travel * fraction,
                            0.13f + 0.17f * fraction);
        CHECK(LegMotion_Run(&context, NULL, &feedback, 100U + step * 250U,
                            0U, &output) ==
              (step == 16U ? LEG_MOTION_DONE : LEG_MOTION_RUNNING));
        NEAR(context.reference_length_m, feedback.length_m);
        NEAR(context.actual_angle_rad,
             (170.0f + travel * fraction) * LEG_MOTION_DEG_TO_RAD);
    }
    NEAR(context.actual_angle_rad, context.target_angle_rad);
    CHECK(isfinite(output.F0) && isfinite(output.Tp));
}

static void AbsolutePath(float initial, float target,
                         LegMotion_Direction direction, float travel)
{
    LegMotion_Context context = {0};
    LegMotion_Output output;
    LegMotion_Command command = Command(LEG_MOTION_ABSOLUTE, direction,
                                        target, 1000U, 0.20f, 2000U);
    LegMotion_Feedback feedback = Feedback(initial, 0.20f);
    float origin;
    unsigned int step;

    CHECK(LegMotion_Run(&context, &command, &feedback, 0U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    origin = context.start_angle_rad;
    NEAR(context.target_angle_rad - origin, travel * LEG_MOTION_DEG_TO_RAD);
    for (step = 1U; step <= 8U; ++step)
    {
        feedback = Feedback(initial + travel * (float)step / 8.0f, 0.20f);
        CHECK(LegMotion_Run(&context, NULL, &feedback, step * 125U, 0U, &output) ==
              (step == 8U ? LEG_MOTION_DONE : LEG_MOTION_RUNNING));
    }
    NEAR(context.actual_angle_rad, context.target_angle_rad);
}

static void CompletionAndHolding(void)
{
    LegMotion_Context context = {0};
    LegMotion_Output output;
    LegMotion_Command command = Command(LEG_MOTION_RELATIVE, LEG_MOTION_POSITIVE,
                                        360.0f, 1000U, 0.20f, 1500U);
    LegMotion_Feedback feedback = Feedback(0.0f, 0.20f);
    float target;

    CHECK(LegMotion_Run(&context, &command, &feedback, 0U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    /* An identical wrapped pose must not complete an unperformed full turn. */
    CHECK(LegMotion_Run(&context, NULL, &feedback, 1000U, 0U, &output) ==
          LEG_MOTION_RUNNING);
    CHECK(output.Tp == LEG_MOTION_TORQUE_MAX);
    CHECK(LegMotion_Run(&context, NULL, &feedback, 1500U, 0U, &output) ==
          LEG_MOTION_TIMEOUT);
    NEAR(output.F0, 0.0f);
    NEAR(output.Tp, 0.0f);
    CHECK(LegMotion_Run(&context, NULL, &feedback, 1600U, 0U, &output) ==
          LEG_MOTION_TIMEOUT);

    command = Command(LEG_MOTION_RELATIVE, LEG_MOTION_NEGATIVE,
                       0.0f, 1000U, 0.20f, 2000U);
    CHECK(LegMotion_Run(&context, &command, &feedback, 2000U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    CHECK(LegMotion_Run(&context, NULL, &feedback, 2500U, 0U, &output) ==
          LEG_MOTION_RUNNING);
    CHECK(LegMotion_Run(&context, NULL, &feedback, 3000U, 0U, &output) ==
          LEG_MOTION_DONE);
    target = context.target_angle_rad;

    /* Perturb a finished leg: it must hold the old angle and short length. */
    feedback = Feedback(30.0f, 0.21f);
    CHECK(LegMotion_Run(&context, NULL, &feedback, 5000U, 0U, &output) ==
          LEG_MOTION_DONE);
    NEAR(context.target_angle_rad, target);
    NEAR(context.reference_length_m, 0.20f);
    CHECK(output.Tp < -5.0f);
    CHECK(output.F0 < -3.0f);

    command = Command(LEG_MOTION_RELATIVE, LEG_MOTION_POSITIVE,
                       90.0f, 500U, 0.30f, 1500U);
    CHECK(LegMotion_Run(&context, &command, &feedback, 5100U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    NEAR(context.reference_angle_rad, context.actual_angle_rad);
    NEAR(context.reference_length_m, 0.21f);
    NEAR(context.target_angle_rad - context.start_angle_rad,
         90.0f * LEG_MOTION_DEG_TO_RAD);
}

static void LatchingAndIndependentLegs(void)
{
    LegMotion_Context left = {0};
    LegMotion_Context right = {0};
    LegMotion_Output output;
    LegMotion_Command command = Command(LEG_MOTION_RELATIVE, LEG_MOTION_POSITIVE,
                                        90.0f, 1000U, 0.30f, 2000U);
    LegMotion_Feedback feedback = Feedback(0.0f, 0.10f);

    CHECK(LegMotion_Run(&left, &command, &feedback, 10U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    command.angle_deg = 0.0f;
    command.length_m = 0.13f;
    CHECK(LegMotion_Run(&right, &command, &feedback, 20U, 1U, &output) ==
          LEG_MOTION_RUNNING);

    feedback = Feedback(45.0f, 0.20f);
    CHECK(LegMotion_Run(&left, &command, &feedback, 510U, 0U, &output) ==
          LEG_MOTION_RUNNING);
    NEAR(left.reference_angle_rad, 45.0f * LEG_MOTION_DEG_TO_RAD);
    NEAR(left.reference_length_m, 0.20f);
    NEAR(left.command.angle_deg, 90.0f);
    NEAR(left.command.length_m, 0.30f);
    NEAR(right.reference_length_m, 0.10f);

    feedback = Feedback(0.0f, 0.13f);
    CHECK(LegMotion_Run(&right, NULL, &feedback, 1020U, 0U, &output) ==
          LEG_MOTION_DONE);
    NEAR(right.reference_length_m, 0.13f);
    NEAR(left.reference_length_m, 0.20f);

    /* Restarting the exact same command must reset the origin and clock. */
    CHECK(LegMotion_Run(&right, &command, &feedback, 1100U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    CHECK(right.start_tick == 1100U);
}

static void ClockAndInstantTargets(void)
{
    LegMotion_Context context = {0};
    LegMotion_Output output;
    LegMotion_Command command = Command(LEG_MOTION_RELATIVE, LEG_MOTION_POSITIVE,
                                        0.0f, 1000U, 0.30f, 2000U);
    LegMotion_Feedback feedback = Feedback(0.0f, 0.10f);
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

    command = Command(LEG_MOTION_RELATIVE, LEG_MOTION_POSITIVE,
                       90.0f, 0U, 0.50f, 500U);
    CHECK(LegMotion_Run(&context, &command, &feedback, 0U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    NEAR(context.reference_length_m, 0.50f);
    CHECK(output.F0 == LEG_MOTION_FORCE_MAX);
    CHECK(output.Tp == LEG_MOTION_TORQUE_MAX);
    feedback = Feedback(90.0f, 0.50f);
    CHECK(LegMotion_Run(&context, NULL, &feedback, 500U, 0U, &output) ==
          LEG_MOTION_DONE); /* Arrival at the deadline wins over timeout. */
}

static void InvalidInputs(void)
{
    LegMotion_Context context = {0};
    LegMotion_Output output = {1.0f, 1.0f};
    LegMotion_Command good = Command(LEG_MOTION_RELATIVE, LEG_MOTION_POSITIVE,
                                     30.0f, 100U, 0.20f, 200U);
    LegMotion_Command command;
    LegMotion_Feedback feedback = Feedback(0.0f, 0.20f);
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

    for (field = 0U; field < 6U; ++field)
    {
        command = good;
        switch (field)
        {
            case 0U: command.direction = (LegMotion_Direction)0; break;
            case 1U: command.angle_mode = (LegMotion_AngleMode)9; break;
            case 2U: command.angle_deg = -1.0f; break;
            case 3U: command.angle_deg = INFINITY; break;
            case 4U: command.length_m = 0.0f; break;
            default: command.timeout_ms = 0U; break;
        }
        CHECK(LegMotion_Run(&context, &command, &feedback, 0U, 1U, &output) ==
              LEG_MOTION_INVALID);
        NEAR(output.F0, 0.0f);
        NEAR(output.Tp, 0.0f);
    }
    for (field = 0U; field < 4U; ++field)
    {
        feedback = Feedback(0.0f, 0.20f);
        CHECK(LegMotion_Run(&context, &good, &feedback, 0U, 1U, &output) ==
              LEG_MOTION_RUNNING);
        switch (field)
        {
            case 0U: feedback.phi0_rad = NAN; break;
            case 1U: feedback.angular_velocity_rad_s = INFINITY; break;
            case 2U: feedback.length_m = NAN; break;
            default: feedback.length_velocity_m_s = NAN; break;
        }
        CHECK(LegMotion_Run(&context, NULL, &feedback, 50U, 0U, &output) ==
              LEG_MOTION_INVALID);
        NEAR(output.F0, 0.0f);
        NEAR(output.Tp, 0.0f);
        feedback = Feedback(0.0f, 0.20f);
        CHECK(LegMotion_Run(&context, NULL, &feedback, 60U, 0U, &output) ==
              LEG_MOTION_INVALID);
    }

    CHECK(LegMotion_Run(&context, &good, &feedback, 0U, 1U, &output) ==
          LEG_MOTION_RUNNING);
    feedback = Feedback(180.0f, 0.20f);
    CHECK(LegMotion_Run(&context, NULL, &feedback, 50U, 0U, &output) ==
          LEG_MOTION_INVALID);
    NEAR(output.F0, 0.0f);
    NEAR(output.Tp, 0.0f);
}

int main(void)
{
    FullTurns(LEG_MOTION_POSITIVE, 1.0f);
    FullTurns(LEG_MOTION_NEGATIVE, 1.0f);
    FullTurns(LEG_MOTION_POSITIVE, 2.0f);
    FullTurns(LEG_MOTION_NEGATIVE, 2.0f);
    AbsolutePath(170.0f, -170.0f, LEG_MOTION_POSITIVE, 20.0f);
    AbsolutePath(170.0f, -170.0f, LEG_MOTION_NEGATIVE, -340.0f);
    AbsolutePath(-170.0f, 170.0f, LEG_MOTION_NEGATIVE, -20.0f);
    AbsolutePath(-170.0f, 170.0f, LEG_MOTION_POSITIVE, 340.0f);
    AbsolutePath(39.143f, 39.143f, LEG_MOTION_NEGATIVE, 0.0f);
    CompletionAndHolding();
    LatchingAndIndependentLegs();
    ClockAndInstantTargets();
    InvalidInputs();
    printf("LegMotion verification passed (%u assertions).\n", checks);
    return 0;
}
