#ifndef LEG_MOTION_H
#define LEG_MOTION_H /* 头文件保护，防止单腿动作接口被重复包含 */

#include <stdint.h>

#define LEG_MOTION_PI 3.14159265358979323846f /* 圆周率 pi，半圈对应的弧度值 */
#define LEG_MOTION_DEG_TO_RAD (LEG_MOTION_PI / 180.0f) /* 角度转弧度的换算系数 */
#define LEG_MOTION_RAD_TO_DEG (180.0f / LEG_MOTION_PI) /* 弧度转角度的换算系数 */

/* Rate-form PD gains; F0 is a force in N, Tp is a torque in N*m. */
#define LEG_MOTION_ANGLE_KP 200.0f /* 摆腿角度比例增益，用角度误差计算虚拟摆腿力矩 */
#define LEG_MOTION_ANGLE_KD 2.0f /* 摆腿角速度阻尼增益，用反馈角速度抑制摆动 */
#define LEG_MOTION_LENGTH_KP 2000.0f /* 腿长比例增益，用长度误差计算虚拟伸缩力 */
#define LEG_MOTION_LENGTH_KD 100.0f /* 腿长变化率阻尼增益，用反馈伸缩速度抑制振荡 */
#define LEG_MOTION_TORQUE_MAX 12.0f /* 虚拟摆腿力矩 Tp 的正负限幅绝对值（N·m） */
#define LEG_MOTION_FORCE_MAX 40.0f /* 虚拟伸缩力 F0 的正负限幅绝对值（N） */
#define LEG_MOTION_LEG_WEIGHT_N 14.33f /* 整条腿的重力估计（N），用于摆腿重力补偿 */
#define LEG_MOTION_LOWER_LEG_WEIGHT_N 10.71f /* 小腿的重力估计（N），用于伸缩方向重力补偿 */
#define LEG_MOTION_ANGLE_TOLERANCE_RAD 0.25f /* 单腿动作完成时允许的连续角度误差（rad） */
#define LEG_MOTION_LENGTH_TOLERANCE_M 0.02f /* 单腿动作完成时允许的腿长误差（m） */

typedef enum
{
    LEG_MOTION_NEGATIVE = -1, /* Decreasing phi0 on either mirrored leg. */
    LEG_MOTION_POSITIVE = 1
} LegMotion_Direction;

typedef enum
{
    LEG_MOTION_POSITION = 0, /* 控制摆角位置，默认模式。 */
    LEG_MOTION_FREE = 1      /* 放开摆角，Tp 恒为零，只控制腿长。 */
} LegMotion_AngleControl;

typedef enum
{
    LEG_MOTION_IDLE = 0,
    LEG_MOTION_RUNNING,
    LEG_MOTION_DONE,
    LEG_MOTION_TIMEOUT,
    LEG_MOTION_INVALID
} LegMotion_Result;

typedef struct
{
    LegMotion_Direction direction;
    float target_phi0_rad; /* POSITION target in VMC radians; FREE ignores the target. */
    uint32_t angle_duration_ms; /* POSITION angle-reference ramp time; FREE ignores it. */
    uint32_t length_duration_ms; /* Independent length-reference ramp time. */
    float length_m;
    uint32_t timeout_ms; /* Total deadline, measured from this command's start. */
    LegMotion_AngleControl angle_control; /* Latched at start; set explicitly. */
} LegMotion_Command;

typedef struct
{
    float phi0_rad;             /* VMC body-relative phi0; vertical = pi/2. */
    float angular_velocity_rad_s;
    float length_m;
    float length_velocity_m_s;
} LegMotion_Feedback;

typedef struct
{
    float F0;
    float Tp;
} LegMotion_Output;

/* Zero-initialize once, then keep a separate context for each leg.
 * Fields are public for debugger inspection; callers must not edit an active
 * context. All angle fields below use vertical = 0, in continuous radians.
 * In FREE, reference_angle_rad and target_angle_rad follow actual_angle_rad. */
typedef struct
{
    LegMotion_Command command;
    LegMotion_Result result;
    uint32_t start_tick;
    float last_angle_rad;
    float actual_angle_rad;
    float start_angle_rad;
    float target_angle_rad;
    float reference_angle_rad;
    float start_length_m;
    float reference_length_m;
} LegMotion_Context;

/* Call once per control cycle, with adjacent actual turns strictly below pi.
 * start != 0 restarts from actual feedback and latches command. Otherwise the
 * command argument is ignored and may be NULL. The clock is in milliseconds.
 * Each duration == 0 sets its reference immediately; timeout_ms must be positive.
 * POSITION completes after both ramp times and both feedback tolerances.
 * POSITION targets are phi0 poses modulo 2*pi in the selected direction.
 * An identical pose commands no extra turn, including targets shifted by 2*pi.
 * FREE outputs Tp=0 and completes from length duration and tolerance only;
 * angle feedback is still validated and unwrapped. Set every command field.
 * DONE holds length plus angle in POSITION, or length only in FREE.
 * TIMEOUT/INVALID are
 * latched with zero output until restart. No HAL, CAN, delays or allocation. */
LegMotion_Result LegMotion_Run(LegMotion_Context *context,
                               const LegMotion_Command *command,
                               const LegMotion_Feedback *feedback,
                               uint32_t now_ms,
                               uint8_t start,
                               LegMotion_Output *output);

#endif
