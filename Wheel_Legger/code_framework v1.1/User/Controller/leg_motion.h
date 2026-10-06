#ifndef LEG_MOTION_H
#define LEG_MOTION_H

#include <stdint.h>

#define LEG_MOTION_PI 3.14159265358979323846f
#define LEG_MOTION_DEG_TO_RAD (LEG_MOTION_PI / 180.0f)
#define LEG_MOTION_RAD_TO_DEG (180.0f / LEG_MOTION_PI)

/* Rate-form PD gains; F0 is a force in N, Tp is a torque in N*m. */
#define LEG_MOTION_ANGLE_KP 15.0f
#define LEG_MOTION_ANGLE_KD 2.0f
#define LEG_MOTION_LENGTH_KP 300.0f
#define LEG_MOTION_LENGTH_KD 9.0f
#define LEG_MOTION_TORQUE_MAX 12.0f
#define LEG_MOTION_FORCE_MAX 20.0f
#define LEG_MOTION_LEG_WEIGHT_N 14.33f
#define LEG_MOTION_LOWER_LEG_WEIGHT_N 10.71f
#define LEG_MOTION_ANGLE_TOLERANCE_RAD 0.25f
#define LEG_MOTION_LENGTH_TOLERANCE_M 0.02f

typedef enum
{
    LEG_MOTION_NEGATIVE = -1, /* Decreasing phi0 on either mirrored leg. */
    LEG_MOTION_POSITIVE = 1
} LegMotion_Direction;

typedef enum
{
    LEG_MOTION_ABSOLUTE = 0, /* Body-relative pose, vertical leg = 0 deg. */
    LEG_MOTION_RELATIVE     /* Nonnegative travel; 360 means a full turn. */
} LegMotion_AngleMode;

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
    LegMotion_AngleMode angle_mode;
    float angle_deg;
    uint32_t duration_ms;
    float length_m;
    uint32_t timeout_ms; /* Total deadline, measured from this command's start. */
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
 * context. All angle fields below use vertical = 0, in continuous radians. */
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
 * duration_ms == 0 sets the reference immediately; timeout_ms must be positive.
 * Absolute targets follow direction; an identical pose commands no extra turn.
 * DONE is latched and continues holding the fixed target. TIMEOUT/INVALID are
 * latched with zero output until restart. No HAL, CAN, delays or allocation. */
LegMotion_Result LegMotion_Run(LegMotion_Context *context,
                               const LegMotion_Command *command,
                               const LegMotion_Feedback *feedback,
                               uint32_t now_ms,
                               uint8_t start,
                               LegMotion_Output *output);

#endif
