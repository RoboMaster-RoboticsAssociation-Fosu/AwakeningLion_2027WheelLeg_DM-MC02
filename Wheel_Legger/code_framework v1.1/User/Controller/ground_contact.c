#include "ground_contact.h"
#include "chassis_task.h"

#include <math.h>
#include <string.h>

/* Watch 可写的离地控制请求：0只观测，1允许离地控制；仅 OFFLINE 采纳，ONLINE 锁存。
   实际采纳值看 Chassis.ground.control_enabled，切勿将它与遥控电机使能混淆。 */
volatile uint8_t ground_control_enable = 1U;

/* OFFLINE 抱车观测入口：运行程序后将此值设1，并保持 ground_control_enable=0。
   0沿用 OFFLINE 复位，1持续观测；最终电机发送仍为零。开关用法与模型限制见头文件。 */
volatile uint8_t ground_offline_debug_enable = 0U;

/* 与 INS 的一阶低通相同；直接复用本拍 dt，不建立另一套计时器。 */
static float GroundContact_Filter(float value, float previous, float dt)
{
    return previous + dt / (GROUND_FILTER_TIME_S + dt) * (value - previous);
}

/* 无效数据只中断确认和滤波历史，保留最近一次确认的离地状态。 */
static void GroundContact_Invalidate(uint8_t leg)
{
    Chassis.ground.valid[leg] = 0U;
    Chassis.ground.initialized[leg] = 0U;
    Chassis.ground.off_time_s[leg] = 0.0f;
    Chassis.ground.land_time_s[leg] = 0.0f;
}

void GroundContact_Reset(void)
{
    uint8_t enabled = Chassis.ground.control_enabled;
    memset(&Chassis.ground, 0, sizeof(Chassis.ground));
    Chassis.ground.control_enabled = enabled;
}

void GroundContact_Update(float dt, float accel_z, const float torque_fb[2][2])
{
    uint8_t leg;
    const float mass = GROUND_LEG_MASS_KG + GROUND_WHEEL_MASS_KG;

    if (Chassis.chassis_enable == OFFLINE)
    {
        Chassis.ground.control_enabled = (ground_control_enable != 0U);
        if (ground_offline_debug_enable == 0U)
        {
            GroundContact_Reset();
            return;
        }
        /* OFFLINE 只观测：不要求 NORMAL，但保留在线、使能与几何有效性检查。
           电机发送仍由 OFFLINE 分支清零，不将调试观测变成控制使能。 */
    }
    else if (Chassis.chassis_mode != NORMAL)
    {
        GroundContact_Reset();
        return;
    }

    /* 与当前机体重力前馈同源，避免另存一份 16 kg 后参数漂移。 */
    Chassis.ground.Fn_static = 0.5f * body_mg + mass * GROUND_GRAVITY_M_S2;
    for (leg = 0U; leg < 2U; ++leg)
    {
        vmc_leg_t *vmc = &Chassis.leg_situation[leg].vmc;
        float F0, Tp, c, s, accel_wheel, force;
        uint8_t first_sample;
        /* 右腿在角度端和命令端已镜像；反馈力矩也只在这里取反一次。 */
        float direction = (leg == RIGHT_Leg) ? -1.0f : 1.0f;

        if (!isfinite(dt) || dt <= 0.0f || dt > GROUND_DT_MAX_S ||
            !isfinite(accel_z) || !Chassis.all_motors_online ||
            !isfinite(vmc->theta) || !isfinite(vmc->d_theta) ||
            !isfinite(vmc->d_L0) || !isfinite(vmc->dd_L0) || !isfinite(vmc->dd_theta) ||
            !VMC_Force_Calc(vmc, direction * torque_fb[leg][0],
                            direction * torque_fb[leg][1], &F0, &Tp))
        {
            GroundContact_Invalidate(leg);
            continue;
        }

        first_sample = (Chassis.ground.initialized[leg] == 0U);
        if (first_sample)
        {
            /* VMC 原始差分可能带着进入 NORMAL 前的运动，首帧从零建立导数滤波。 */
            Chassis.ground.dd_L0[leg] = 0.0f;
            Chassis.ground.dd_theta[leg] = 0.0f;
        }
        else
        {
            Chassis.ground.dd_L0[leg] = GroundContact_Filter(
                vmc->dd_L0, Chassis.ground.dd_L0[leg], dt);
            Chassis.ground.dd_theta[leg] = GroundContact_Filter(
                vmc->dd_theta, Chassis.ground.dd_theta[leg], dt);
        }

        c = arm_cos_f32(vmc->theta);
        s = arm_sin_f32(vmc->theta);
        /* z_wheel = z_body - L0*cos(theta)，展开二阶导得到轮轴竖直加速度。 */
        accel_wheel = accel_z - Chassis.ground.dd_L0[leg] * c
                    + 2.0f * vmc->d_L0 * vmc->d_theta * s
                    + vmc->L0 * Chassis.ground.dd_theta[leg] * s
                    + vmc->L0 * vmc->d_theta * vmc->d_theta * c;
        /* Tp 与 phi0 共轭，竖直投影为 -Tp*sin(theta)/L0。
           MotionAccel_n 已去重力，所以质量项需加 g；腿轮采用参考集总近似。
           气弹簧已拆除，不加弹簧力，也不扣除电机真实承担的 Roll/PD 控制力。 */
        force = F0 * c - Tp * s / vmc->L0
              + mass * (GROUND_GRAVITY_M_S2 + accel_wheel);
        if (!isfinite(force))
        {
            GroundContact_Invalidate(leg);
            continue;
        }

        Chassis.ground.F0_feedback[leg] = F0;
        Chassis.ground.Tp_feedback[leg] = Tp;
        Chassis.ground.Fn_raw[leg] = force;
        Chassis.ground.Fn[leg] = first_sample ? force :
            GroundContact_Filter(force, Chassis.ground.Fn[leg], dt);
        Chassis.ground.Fn_ratio[leg] = Chassis.ground.Fn[leg] / Chassis.ground.Fn_static;
        Chassis.ground.valid[leg] = 1U;
        Chassis.ground.initialized[leg] = 1U;
        if (first_sample) continue;

        /* 两阈值提供滞回；按实际秒数累计，短时卸载不立即切换控制。 */
        if (Chassis.ground.off_flag[leg] == 0U)
        {
            Chassis.ground.land_time_s[leg] = 0.0f;
            if (Chassis.ground.Fn_ratio[leg] <= GROUND_OFF_RATIO)
            {
                Chassis.ground.off_time_s[leg] += dt;
                if (Chassis.ground.off_time_s[leg] + 1.0e-7f >= GROUND_OFF_CONFIRM_S)
                {
                    Chassis.ground.off_flag[leg] = 1U;
                    Chassis.ground.off_time_s[leg] = 0.0f;
                }
            }
            else Chassis.ground.off_time_s[leg] = 0.0f;
        }
        else
        {
            Chassis.ground.off_time_s[leg] = 0.0f;
            if (Chassis.ground.Fn_ratio[leg] >= GROUND_LAND_RATIO)
            {
                Chassis.ground.land_time_s[leg] += dt;
                if (Chassis.ground.land_time_s[leg] + 1.0e-7f >= GROUND_LAND_CONFIRM_S)
                {
                    Chassis.ground.off_flag[leg] = 0U;
                    Chassis.ground.land_time_s[leg] = 0.0f;
                }
            }
            else Chassis.ground.land_time_s[leg] = 0.0f;
        }
    }
    Chassis.ground.all_off_flag = Chassis.ground.off_flag[LEFT_Leg] &&
                                  Chassis.ground.off_flag[RIGHT_Leg];
}
