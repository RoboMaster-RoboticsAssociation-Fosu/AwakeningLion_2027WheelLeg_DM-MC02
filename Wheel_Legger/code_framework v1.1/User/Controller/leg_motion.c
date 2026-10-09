#include "leg_motion.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#define LEG_MOTION_TWO_PI (2.0f * LEG_MOTION_PI) /* 一整圈的弧度值，用于角度归一化及跨圈展开 */
#define LEG_MOTION_SAME_POSE_EPSILON_RAD 0.000001f /* 同姿态判定的基础容差（rad），也用于半圈采样歧义检查 */

static float LegMotion_WrapSigned(float angle)
{
    angle = fmodf(angle, LEG_MOTION_TWO_PI);
    if (angle > LEG_MOTION_PI)
    {
        angle -= LEG_MOTION_TWO_PI;
    }
    else if (angle < -LEG_MOTION_PI)
    {
        angle += LEG_MOTION_TWO_PI;
    }
    return angle;
}

static float LegMotion_Limit(float value, float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }
    if (value < -maximum)
    {
        return -maximum;
    }
    return value;
}

static uint8_t LegMotion_FeedbackValid(const LegMotion_Feedback *feedback)
{
    return feedback != NULL &&
           isfinite(feedback->phi0_rad) &&
           isfinite(feedback->angular_velocity_rad_s) &&
           isfinite(feedback->length_m) && feedback->length_m > 0.0f &&
           isfinite(feedback->length_velocity_m_s);
}

static uint8_t LegMotion_CommandValid(const LegMotion_Command *command)
{
    return command != NULL &&
           (command->angle_control == LEG_MOTION_POSITION ||
            command->angle_control == LEG_MOTION_FREE) &&
           (command->direction == LEG_MOTION_NEGATIVE ||
            command->direction == LEG_MOTION_POSITIVE) &&
           isfinite(command->target_phi0_rad) &&
           isfinite(command->length_m) && command->length_m > 0.0f &&
           command->timeout_ms > 0U;
}

static LegMotion_Result LegMotion_Invalid(LegMotion_Context *context)
{
    if (context != NULL)
    {
        context->result = LEG_MOTION_INVALID;
    }
    return LEG_MOTION_INVALID;
}

LegMotion_Result LegMotion_Run(LegMotion_Context *context,
                               const LegMotion_Command *command,
                               const LegMotion_Feedback *feedback,
                               uint32_t now_ms,
                               uint8_t start,
                               LegMotion_Output *output)
{
    float angle;
    float delta;
    float angle_fraction;
    float length_fraction;
    float angle_error;
    float length_error;
    float F0;
    float Tp;
    uint32_t elapsed;

    if (output == NULL)
    {
        return LegMotion_Invalid(context);
    }
    output->F0 = 0.0f;
    output->Tp = 0.0f;
    if (context == NULL)
    {
        return LEG_MOTION_INVALID;
    }
    if (start == 0U && (context->result == LEG_MOTION_IDLE ||
                        context->result == LEG_MOTION_TIMEOUT ||
                        context->result == LEG_MOTION_INVALID))
    {
        return context->result;
    }
    if (!LegMotion_FeedbackValid(feedback))
    {
        return LegMotion_Invalid(context);
    }

    angle = LegMotion_WrapSigned(feedback->phi0_rad - LEG_MOTION_PI * 0.5f);
    if (start != 0U)
    {
        if (!LegMotion_CommandValid(command))
        {
            return LegMotion_Invalid(context);
        }
        delta = 0.0f;
        if (command->angle_control == LEG_MOTION_POSITION)
        {
            float target = LegMotion_WrapSigned(command->target_phi0_rad -
                                                 LEG_MOTION_PI * 0.5f);
            /* Whole-turn offsets lose precision as the input magnitude grows.
             * Apply this allowance only to the initial same-pose decision. */
            float pose_epsilon = LEG_MOTION_SAME_POSE_EPSILON_RAD +
                2.0f * FLT_EPSILON * fabsf(command->target_phi0_rad) +
                2.0f * FLT_EPSILON * fabsf(feedback->phi0_rad);
            delta = LegMotion_WrapSigned(target - angle);
            if (fabsf(delta) <= pose_epsilon)
            {
                delta = 0.0f;
            }
            else if (command->direction == LEG_MOTION_POSITIVE && delta < 0.0f)
            {
                delta += LEG_MOTION_TWO_PI;
            }
            else if (command->direction == LEG_MOTION_NEGATIVE && delta > 0.0f)
            {
                delta -= LEG_MOTION_TWO_PI;
            }
        }
        if (!isfinite(delta) || !isfinite(angle + delta))
        {
            return LegMotion_Invalid(context);
        }
        context->command = *command;
        context->start_tick = now_ms;
        context->last_angle_rad = angle;
        context->actual_angle_rad = angle;
        context->start_angle_rad = angle;
        context->target_angle_rad = angle + delta;
        context->reference_angle_rad = angle;
        context->start_length_m = feedback->length_m;
        context->reference_length_m = feedback->length_m;
        context->result = LEG_MOTION_RUNNING;
    }
    else
    {
        delta = LegMotion_WrapSigned(angle - context->last_angle_rad);
        /* A half-turn between samples cannot be unwrapped unambiguously. */
        if (fabsf(delta) >= LEG_MOTION_PI - LEG_MOTION_SAME_POSE_EPSILON_RAD)
        {
            return LegMotion_Invalid(context);
        }
        context->actual_angle_rad += delta;
        context->last_angle_rad = angle;
    }

    elapsed = (uint32_t)(now_ms - context->start_tick);
    angle_fraction = (context->result == LEG_MOTION_DONE ||
                      context->command.angle_duration_ms == 0U ||
                      elapsed >= context->command.angle_duration_ms) ?
                     1.0f : (float)elapsed / (float)context->command.angle_duration_ms;
    length_fraction = (context->result == LEG_MOTION_DONE ||
                       context->command.length_duration_ms == 0U ||
                       elapsed >= context->command.length_duration_ms) ?
                      1.0f : (float)elapsed / (float)context->command.length_duration_ms;
    if (context->command.angle_control == LEG_MOTION_FREE)
    {
        /* 自由摆角只更新诊断角度，不建立固定角度目标。 */
        context->target_angle_rad = context->actual_angle_rad;
        context->reference_angle_rad = context->actual_angle_rad;
    }
    else
    {
        context->reference_angle_rad = context->start_angle_rad +
            (context->target_angle_rad - context->start_angle_rad) * angle_fraction;
    }
    context->reference_length_m = context->start_length_m +
        (context->command.length_m - context->start_length_m) * length_fraction;
    if (!isfinite(context->actual_angle_rad) ||
        !isfinite(context->reference_angle_rad) ||
        !isfinite(context->reference_length_m))
    {
        return LegMotion_Invalid(context);
    }

    angle_error = context->reference_angle_rad - context->actual_angle_rad;
    length_error = context->reference_length_m - feedback->length_m;
    if (context->result == LEG_MOTION_RUNNING)
    {
        if (length_fraction == 1.0f &&
            (context->command.angle_control == LEG_MOTION_FREE ||
             (angle_fraction == 1.0f &&
              fabsf(angle_error) < LEG_MOTION_ANGLE_TOLERANCE_RAD)) &&
            fabsf(length_error) < LEG_MOTION_LENGTH_TOLERANCE_M)
        {
            context->result = LEG_MOTION_DONE;
        }
        else if (elapsed >= context->command.timeout_ms)
        {
            context->result = LEG_MOTION_TIMEOUT;
            return context->result;
        }
    }

    /* Preserve the existing physical-coordinate PD and gravity approximation.
     * Use the wrapped physical angle for gravity, continuous error for motion. */
    Tp = 0.0f;
    if (context->command.angle_control == LEG_MOTION_POSITION)
    {
        Tp = LEG_MOTION_ANGLE_KP * angle_error -
             LEG_MOTION_ANGLE_KD * feedback->angular_velocity_rad_s +
             LEG_MOTION_LEG_WEIGHT_N * feedback->length_m * 0.5f * sinf(angle);
    }
    F0 = LEG_MOTION_LENGTH_KP * length_error -
         LEG_MOTION_LENGTH_KD * feedback->length_velocity_m_s -
         LEG_MOTION_LOWER_LEG_WEIGHT_N * cosf(angle);
    if (!isfinite(Tp) || !isfinite(F0))
    {
        return LegMotion_Invalid(context);
    }
    output->Tp = LegMotion_Limit(Tp, LEG_MOTION_TORQUE_MAX);
    output->F0 = LegMotion_Limit(F0, LEG_MOTION_FORCE_MAX);
    return context->result;
}
