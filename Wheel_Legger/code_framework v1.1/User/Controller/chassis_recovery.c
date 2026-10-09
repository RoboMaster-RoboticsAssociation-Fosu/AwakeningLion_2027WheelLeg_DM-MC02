#include "chassis_recovery.h"

#include <math.h>
#include <stddef.h>

static void ChassisRecovery_ZeroOutput(ChassisRecovery_Output *output)
{
    if (output != NULL)
    {
        output->leg[CHASSIS_RECOVERY_LEFT].F0 = 0.0f;
        output->leg[CHASSIS_RECOVERY_LEFT].Tp = 0.0f;
        output->leg[CHASSIS_RECOVERY_RIGHT].F0 = 0.0f;
        output->leg[CHASSIS_RECOVERY_RIGHT].Tp = 0.0f;
    }
}

static ChassisRecovery_Result ChassisRecovery_Fail(ChassisRecovery_Context *context,
                                                  ChassisRecovery_Output *output)
{
    ChassisRecovery_ZeroOutput(output);
    if (context != NULL)
    {
        context->result = CHASSIS_RECOVERY_FAULT;
    }
    return CHASSIS_RECOVERY_FAULT;
}

void ChassisRecovery_Reset(ChassisRecovery_Context *context)
{
    if (context == NULL)
    {
        return;
    }
    context->phase[CHASSIS_RECOVERY_LEFT] = CHASSIS_RECOVERY_SWING;
    context->phase[CHASSIS_RECOVERY_RIGHT] = CHASSIS_RECOVERY_SWING;
    context->result = CHASSIS_RECOVERY_IDLE;
    context->entry_tick = 0U;
    context->handoff_tick = 0U;
    context->sweep_arrived[CHASSIS_RECOVERY_LEFT] = 0U;
    context->sweep_arrived[CHASSIS_RECOVERY_RIGHT] = 0U;
}

uint8_t ChassisRecovery_InEarlyWindow(float phi0)
{
    return isfinite(phi0) &&
           phi0 >= SPIN_RETRACT_PHI0_MIN_RAD &&
           phi0 <= SPIN_RETRACT_PHI0_MAX_RAD;
}

static uint8_t ChassisRecovery_AngleArrived(const LegMotion_Context *motion,
                                            uint32_t now_ms)
{
    return (uint32_t)(now_ms - motion->start_tick) >= motion->command.angle_duration_ms &&
           fabsf(motion->target_angle_rad - motion->actual_angle_rad) < SPIN_ANGLE_WINDOW;
}

/* 只读推算本拍连续角，双方决定收腿前不推进任何单腿上下文。 */
static uint8_t ChassisRecovery_SweepReady(const LegMotion_Context *motion,
                                          const LegMotion_Feedback *feedback,
                                          uint32_t now_ms)
{
    float angle;
    float delta;
    float actual;

    if ((motion->result != LEG_MOTION_RUNNING && motion->result != LEG_MOTION_DONE) ||
        motion->command.angle_control != LEG_MOTION_POSITION ||
        (uint32_t)(now_ms - motion->start_tick) < motion->command.angle_duration_ms ||
        !isfinite(feedback->phi0_rad) || !isfinite(feedback->angular_velocity_rad_s) ||
        !isfinite(feedback->length_m) || feedback->length_m <= 0.0f ||
        !isfinite(feedback->length_velocity_m_s))
    {
        return 0U;
    }
    angle = remainderf(feedback->phi0_rad - LEG_MOTION_PI * 0.5f,
                       2.0f * LEG_MOTION_PI);
    delta = remainderf(angle - motion->last_angle_rad, 2.0f * LEG_MOTION_PI);
    /* 与单腿连续角展开相同，半圈跳变交由实际运行报错。 */
    if (fabsf(delta) >= LEG_MOTION_PI - 0.000001f)
    {
        return 0U;
    }
    actual = motion->actual_angle_rad + delta;
    return isfinite(actual) && fabsf(motion->target_angle_rad - actual) < SPIN_ANGLE_WINDOW;
}

static LegMotion_Direction ChassisRecovery_AlignDirection(float phi0)
{
    float delta = remainderf(SPIN_ALIGN_TARGET_PHI0_RAD - phi0, 2.0f * LEG_MOTION_PI);

    /* 半圈无较短方向，沿用扫腿方向；同姿态由单腿模块处理。 */
    if (fabsf(fabsf(delta) - LEG_MOTION_PI) < 0.000001f)
    {
        return SPIN_SWEEP_DIR;
    }
    if (delta < 0.0f)
    {
        return LEG_MOTION_NEGATIVE;
    }
    return LEG_MOTION_POSITIVE;
}

static LegMotion_Result ChassisRecovery_RunLeg(ChassisRecovery_Context *context,
                                               uint8_t leg,
                                               const LegMotion_Feedback *feedback,
                                               uint8_t start,
                                               uint8_t arrived,
                                               uint8_t start_retract,
                                               uint8_t start_align,
                                               uint32_t now_ms,
                                               LegMotion_Output *output)
{
    LegMotion_Command command;
    LegMotion_Result result;

    if (context->phase[leg] == CHASSIS_RECOVERY_SWING)
    {
        /* 先用原动作验证本拍反馈，避免重新启动 FREE 掩盖采样故障。 */
        if (start == 0U &&
            ((arrived != 0U && context->sweep_arrived[leg] == 0U) || start_retract != 0U))
        {
            result = LegMotion_Run(&context->motion[leg], NULL, feedback,
                                   now_ms, 0U, output);
            if (result == LEG_MOTION_TIMEOUT || result == LEG_MOTION_INVALID)
            {
                return result;
            }
        }
        /* 只推进当前腿；左腿失败前不修改右腿的就绪状态。 */
        if (arrived != 0U && context->sweep_arrived[leg] == 0U)
        {
            context->sweep_arrived[leg] = 1U;
            start = 1U;
        }
    }

    /* 两腿上一拍都收完，才各自捕获反馈并同拍启动第二段。 */
    if (context->phase[leg] == CHASSIS_RECOVERY_WAIT_ALIGN && start_align != 0U)
    {
        context->phase[leg] = CHASSIS_RECOVERY_ALIGN;
        start = 1U;
    }
    if (context->phase[leg] == CHASSIS_RECOVERY_SWING &&
        start_retract != 0U)
    {
        context->phase[leg] = CHASSIS_RECOVERY_RETRACT;
        context->sweep_arrived[leg] = 0U;
        start = 1U;
    }
    if (context->phase[leg] == CHASSIS_RECOVERY_WAIT_ALIGN ||
        context->phase[leg] == CHASSIS_RECOVERY_HOLD)
    {
        return LegMotion_Run(&context->motion[leg], NULL, feedback,
                             now_ms, 0U, output);
    }

    command.angle_control = LEG_MOTION_POSITION;
    command.direction = SPIN_SWEEP_DIR;
    command.timeout_ms = SPIN_SCAN_TIMEOUT_MS - (uint32_t)(now_ms - context->entry_tick);
    switch (context->phase[leg])
    {
        case CHASSIS_RECOVERY_SWING:
        {
            if (context->sweep_arrived[leg] != 0U)
            {
                /* 已就绪：摆角自由，仅在进入等待时捕获实际腿长。 */
                command.angle_control = LEG_MOTION_FREE;
                command.target_phi0_rad = feedback->phi0_rad;
                command.angle_duration_ms = 0U;
                command.length_duration_ms = 0U;
                command.length_m = feedback->length_m;
            }
            else
            {
                command.target_phi0_rad = SPIN_TARGET_PHI0_RAD;
                command.angle_duration_ms = SPIN_SWING_ANGLE_TIME_MS;
                command.length_duration_ms = SPIN_SWING_LENGTH_TIME_MS;
                command.length_m = SPIN_SCAN_LENGTH;
            }
        }break;
        case CHASSIS_RECOVERY_RETRACT:
        {
            command.angle_control = LEG_MOTION_FREE;
            command.target_phi0_rad = feedback->phi0_rad;
            command.angle_duration_ms = 0U;
            command.length_duration_ms = SPIN_RETRACT_TIME_MS;
            command.length_m = SPIN_RETRACT_LENGTH;
        }break;
        case CHASSIS_RECOVERY_ALIGN:
        {
            command.target_phi0_rad = SPIN_ALIGN_TARGET_PHI0_RAD;
            command.angle_duration_ms = SPIN_ALIGN_TIME_MS;
            command.length_duration_ms = SPIN_ALIGN_TIME_MS;
            command.length_m = SPIN_RETRACT_LENGTH;
            if (start != 0U)
            {
                command.direction = ChassisRecovery_AlignDirection(feedback->phi0_rad);
            }
        }break;
        default:
        {
            return LEG_MOTION_INVALID;
        }
    }
    result = LegMotion_Run(&context->motion[leg], &command, feedback,
                           now_ms, start, output);
    if (result == LEG_MOTION_TIMEOUT || result == LEG_MOTION_INVALID)
    {
        return result;
    }

    if (context->phase[leg] == CHASSIS_RECOVERY_RETRACT && result == LEG_MOTION_DONE)
    {
        context->phase[leg] = CHASSIS_RECOVERY_WAIT_ALIGN;
    }
    else if (context->phase[leg] == CHASSIS_RECOVERY_ALIGN && result == LEG_MOTION_DONE &&
             ChassisRecovery_AngleArrived(&context->motion[leg], now_ms))
    {
        context->phase[leg] = CHASSIS_RECOVERY_HOLD;
    }
    return result;
}

static uint8_t ChassisRecovery_HandoffReady(const ChassisRecovery_Input *input)
{
    uint8_t leg;

    if (!isfinite(input->pitch_rad) || fabsf(input->pitch_rad) >= SPIN_HANDOFF_PITCH_MAX_RAD)
    {
        return 0U;
    }
    for (leg = 0U; leg < 2U; ++leg)
    {
        const LegMotion_Feedback *feedback = &input->leg[leg];

        if (!isfinite(feedback->phi0_rad) ||
            fabsf(remainderf(SPIN_ALIGN_TARGET_PHI0_RAD - feedback->phi0_rad,
                             2.0f * LEG_MOTION_PI)) >= SPIN_ANGLE_WINDOW ||
            feedback->phi0_rad < SPIN_RETRACT_PHI0_MIN_RAD ||
            feedback->phi0_rad > SPIN_RETRACT_PHI0_MAX_RAD ||
            fabsf(SPIN_RETRACT_LENGTH - feedback->length_m) >= SPIN_LENGTH_TOLERANCE ||
            fabsf(feedback->length_velocity_m_s) >= 0.05f)
        {
            return 0U;
        }
    }
    return 1U;
}

ChassisRecovery_Result ChassisRecovery_Run(ChassisRecovery_Context *context,
                                          const ChassisRecovery_Input *input,
                                          uint32_t now_ms,
                                          ChassisRecovery_Output *output)
{
    uint8_t first_cycle = 0U;
    uint8_t start_align;
    uint8_t retract_left, retract_right;
    uint8_t ready_left, ready_right;
    uint8_t arrived_left, arrived_right;
    LegMotion_Result result;

    ChassisRecovery_ZeroOutput(output);
    if (context == NULL || input == NULL || output == NULL)
    {
        return ChassisRecovery_Fail(context, output);
    }
    if (input->enabled == 0U)
    {
        ChassisRecovery_Reset(context);
        return CHASSIS_RECOVERY_IDLE;
    }
    if (context->result == CHASSIS_RECOVERY_FAULT)
    {
        return CHASSIS_RECOVERY_FAULT;
    }
    if (context->result == CHASSIS_RECOVERY_IDLE)
    {
        ChassisRecovery_Reset(context);
        context->entry_tick = now_ms;
        context->result = CHASSIS_RECOVERY_RUNNING;
        first_cycle = 1U;
    }

    switch (context->result)
    {
        case CHASSIS_RECOVERY_RUNNING:
        {
            /* 所有动作和等待共用总截止时间，第二段不重新计时。 */
            if ((uint32_t)(now_ms - context->entry_tick) >= SPIN_SCAN_TIMEOUT_MS)
            {
                return ChassisRecovery_Fail(context, output);
            }

            start_align = context->phase[CHASSIS_RECOVERY_LEFT] == CHASSIS_RECOVERY_WAIT_ALIGN &&
                          context->phase[CHASSIS_RECOVERY_RIGHT] == CHASSIS_RECOVERY_WAIT_ALIGN;

            /* 同拍只读计算双方就绪，再按左、右顺序锁存和推进。 */
            arrived_left = context->phase[CHASSIS_RECOVERY_LEFT] == CHASSIS_RECOVERY_SWING &&
                           (context->sweep_arrived[CHASSIS_RECOVERY_LEFT] != 0U ||
                            ChassisRecovery_InEarlyWindow(input->leg[CHASSIS_RECOVERY_LEFT].phi0_rad) ||
                            (first_cycle == 0U &&
                             ChassisRecovery_SweepReady(&context->motion[CHASSIS_RECOVERY_LEFT],
                                                       &input->leg[CHASSIS_RECOVERY_LEFT], now_ms)));
            arrived_right = context->phase[CHASSIS_RECOVERY_RIGHT] == CHASSIS_RECOVERY_SWING &&
                            (context->sweep_arrived[CHASSIS_RECOVERY_RIGHT] != 0U ||
                             ChassisRecovery_InEarlyWindow(input->leg[CHASSIS_RECOVERY_RIGHT].phi0_rad) ||
                             (first_cycle == 0U &&
                              ChassisRecovery_SweepReady(&context->motion[CHASSIS_RECOVERY_RIGHT],
                                                        &input->leg[CHASSIS_RECOVERY_RIGHT], now_ms)));
            ready_left = arrived_left ||
                         context->phase[CHASSIS_RECOVERY_LEFT] == CHASSIS_RECOVERY_RETRACT ||
                         context->phase[CHASSIS_RECOVERY_LEFT] == CHASSIS_RECOVERY_WAIT_ALIGN;
            ready_right = arrived_right ||
                          context->phase[CHASSIS_RECOVERY_RIGHT] == CHASSIS_RECOVERY_RETRACT ||
                          context->phase[CHASSIS_RECOVERY_RIGHT] == CHASSIS_RECOVERY_WAIT_ALIGN;
            retract_left = ready_left && ready_right;
            retract_right = retract_left;

            /* 左腿失败立即返回，不继续推进右腿。 */
            result = ChassisRecovery_RunLeg(context, CHASSIS_RECOVERY_LEFT,
                                            &input->leg[CHASSIS_RECOVERY_LEFT],
                                            first_cycle, arrived_left, retract_left, start_align, now_ms,
                                            &output->leg[CHASSIS_RECOVERY_LEFT]);
            if (result == LEG_MOTION_TIMEOUT || result == LEG_MOTION_INVALID)
            {
                return ChassisRecovery_Fail(context, output);
            }

            /* 右腿失败时同时清除左腿本周期的输出。 */
            result = ChassisRecovery_RunLeg(context, CHASSIS_RECOVERY_RIGHT,
                                            &input->leg[CHASSIS_RECOVERY_RIGHT],
                                            first_cycle, arrived_right, retract_right, start_align, now_ms,
                                            &output->leg[CHASSIS_RECOVERY_RIGHT]);
            if (result == LEG_MOTION_TIMEOUT || result == LEG_MOTION_INVALID)
            {
                return ChassisRecovery_Fail(context, output);
            }
            if (context->phase[CHASSIS_RECOVERY_LEFT] == CHASSIS_RECOVERY_HOLD &&
                context->phase[CHASSIS_RECOVERY_RIGHT] == CHASSIS_RECOVERY_HOLD)
            {
                context->handoff_tick = now_ms;
                context->result = CHASSIS_RECOVERY_HANDOFF;
            }
            /* 本拍只进入交接，下一拍才检查是否恢复平衡。 */
        }break;
        case CHASSIS_RECOVERY_HANDOFF:
        case CHASSIS_RECOVERY_DONE:
        {
            result = ChassisRecovery_RunLeg(context, CHASSIS_RECOVERY_LEFT,
                                            &input->leg[CHASSIS_RECOVERY_LEFT],
                                            0U, 0U, 0U, 0U, now_ms,
                                            &output->leg[CHASSIS_RECOVERY_LEFT]);
            if (result != LEG_MOTION_DONE)
            {
                return ChassisRecovery_Fail(context, output);
            }
            result = ChassisRecovery_RunLeg(context, CHASSIS_RECOVERY_RIGHT,
                                            &input->leg[CHASSIS_RECOVERY_RIGHT],
                                            0U, 0U, 0U, 0U, now_ms,
                                            &output->leg[CHASSIS_RECOVERY_RIGHT]);
            if (result != LEG_MOTION_DONE)
            {
                return ChassisRecovery_Fail(context, output);
            }
            if (context->result == CHASSIS_RECOVERY_DONE)
            {
                return CHASSIS_RECOVERY_DONE;
            }

            /* 就绪优先；未满足交接条件时再检查超时。 */
            if (ChassisRecovery_HandoffReady(input))
            {
                context->result = CHASSIS_RECOVERY_DONE;
            }
            else if ((uint32_t)(now_ms - context->handoff_tick) >= SPIN_HANDOFF_TIMEOUT_MS)
            {
                return ChassisRecovery_Fail(context, output);
            }
        }break;
        default:
        {
            return ChassisRecovery_Fail(context, output);
        }
    }
    return context->result;
}
