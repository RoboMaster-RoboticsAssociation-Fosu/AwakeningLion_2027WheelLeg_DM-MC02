#ifndef CHASSIS_RECOVERY_H
#define CHASSIS_RECOVERY_H /* 头文件保护，防止自起模块接口被重复包含 */

#include "leg_motion.h"

/* 自起参数集中在这里；phi0 与 VMC 反馈使用同一坐标。 */
#define SPIN_SWEEP_DIR LEG_MOTION_NEGATIVE /* 扫腿方向；负方向表示 phi0 减小，正方向表示 phi0 增大 */
#define SPIN_TARGET_PHI0_RAD 2.8f /* 扫腿目标 phi0（rad）；腿相对机身竖直向下时为 pi/2 */
#define SPIN_RAMP_TIME_MS 4000U /* 扫腿角度与伸腿长度参考的共同斜坡时间（ms） */
#define SPIN_RETRACT_TIME_MS 1000U /* 固定腿角收腿时，腿长参考的斜坡时间（ms） */
#define SPIN_EARLY_ANGLE 0.9f /* 提前收腿角窗宽度（rad）；负向窗口为 phi0-pi/2 位于 [-此值, 0] */
#define SPIN_ANGLE_WINDOW 0.05f /* 扫腿终点、第二段转动及交接的角度误差上限（rad），严格小于才算到位 */
#define SPIN_ALIGN_TARGET_PHI0_RAD 1.7f /* 两腿收完后共同转向的目标 phi0（rad） */
#define SPIN_ALIGN_TIME_MS 1000U /* 两腿同拍启动第二段转动的斜坡时间（ms） */
#define SPIN_SCAN_LENGTH 0.39f /* 扫腿阶段的目标腿长（m） */
#define SPIN_RETRACT_LENGTH 0.139f /* 收腿及保持阶段的目标腿长（m） */
#define SPIN_LENGTH_TOLERANCE LEG_MOTION_LENGTH_TOLERANCE_M /* 自起交接时允许的腿长误差（m），沿用单腿到位容差 */
#define SPIN_SCAN_TIMEOUT_MS 600000U /* 扫腿、收腿、等待及第二段转动的共享总超时（ms） */
#define SPIN_HANDOFF_TIMEOUT_MS 500U /* 两腿完成后，等待满足正常平衡交接条件的超时（ms） */

#define CHASSIS_RECOVERY_LEFT 0U /* 自起模块左腿的数组下标 */
#define CHASSIS_RECOVERY_RIGHT 1U /* 自起模块右腿的数组下标 */

typedef enum
{
    CHASSIS_RECOVERY_IDLE = 0,
    CHASSIS_RECOVERY_RUNNING,
    CHASSIS_RECOVERY_HANDOFF,
    CHASSIS_RECOVERY_DONE,
    CHASSIS_RECOVERY_FAULT
} ChassisRecovery_Result;

typedef enum
{
    CHASSIS_RECOVERY_SWING = 0,
    CHASSIS_RECOVERY_RETRACT,
    CHASSIS_RECOVERY_WAIT_ALIGN,
    CHASSIS_RECOVERY_ALIGN,
    CHASSIS_RECOVERY_HOLD
} ChassisRecovery_Phase;

typedef struct
{
    uint8_t enabled;
    float pitch_rad;
    LegMotion_Feedback leg[2];
} ChassisRecovery_Input;

typedef struct
{
    LegMotion_Output leg[2];
} ChassisRecovery_Output;

/* 初次使用清零；运行期间仅供观察，由模块维护。 */
typedef struct
{
    LegMotion_Context motion[2];
    ChassisRecovery_Phase phase[2];
    ChassisRecovery_Result result;
    uint32_t entry_tick;
    uint32_t handoff_tick;
} ChassisRecovery_Context;

/* 复位阶段和计时，保留单腿动作记录供故障后观察。 */
void ChassisRecovery_Reset(ChassisRecovery_Context *context);

/* 每周期调用一次；时钟单位 ms，模块不访问 HAL、CAN 或全局底盘。
 * 禁用返回 IDLE 并清零输出；FAULT 锁存，复位后才能重新启动。
 * 两腿收完后下一拍同启第二段转动；方向各自按最短路径锁存。
 * 双腿第二段完成当拍返回 HANDOFF，下一次调用才检查交接条件。
 * DONE 仍保持双腿输出，由调用者交回正常控制。 */
ChassisRecovery_Result ChassisRecovery_Run(ChassisRecovery_Context *context,
                                          const ChassisRecovery_Input *input,
                                          uint32_t now_ms,
                                          ChassisRecovery_Output *output);

#endif
