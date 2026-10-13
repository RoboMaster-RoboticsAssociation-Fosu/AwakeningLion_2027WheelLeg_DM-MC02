#ifndef GROUND_CONTACT_H
#define GROUND_CONTACT_H

#include <stdint.h>

/* 同车腿/轮质量；气弹簧已拆除。机体静载继续使用 chassis_task.h 的 body_mg。 */
#define GROUND_LEG_MASS_KG       1.65f
#define GROUND_WHEEL_MASS_KG     0.537f
#define GROUND_GRAVITY_M_S2      9.81f
#define GROUND_FILTER_TIME_S    0.02f
#define GROUND_OFF_RATIO        0.20f
#define GROUND_LAND_RATIO       0.35f
#define GROUND_OFF_CONFIRM_S    0.03f
#define GROUND_LAND_CONFIRM_S   0.05f
#define GROUND_DT_MAX_S         0.05f

/* 下标为腿：0 左、1 右；轮数组的左右下标不同，不能混用。 */
typedef struct
{
    float F0_feedback[2];    /* 反馈逆解的轴向力，N；不是 vmc.F0 控制指令。 */
    float Tp_feedback[2];    /* 反馈逆解的摆矩，N*m；不是 vmc.Tp 控制指令。 */
    float Fn_raw[2];         /* 瞬时地面支撑力估计，N。 */
    float Fn[2];             /* 低通后的支撑力估计，N。 */
    float Fn_ratio[2];       /* 支撑力/静载：约 1 为标称静载；<=0.20 是离地候选，>=0.35 是落地候选。 */
    float Fn_static;         /* 单腿标称静载，N。 */
    float dd_L0[2];          /* 腿长二阶导低通值，m/s^2。 */
    float dd_theta[2];       /* 腿角二阶导低通值，rad/s^2。 */
    float off_time_s[2];     /* 离地条件连续满足时间，s。 */
    float land_time_s[2];    /* 落地条件连续满足时间，s。 */
    uint8_t valid[2];        /* 1=本拍估算有效；0=无效，此时 off_flag 保留上次确认值，不能当作本拍结论。 */
    uint8_t initialized[2];  /* 已由首个有效样本建立滤波初值。 */
    uint8_t off_flag[2];     /* 0左/1右；值1=连续30ms确认离地，值0=未置离地或已连续50ms确认落地。 */
    uint8_t all_off_flag;    /* 整车离地标志：1=两腿 off_flag 均为1；配合 valid[0/1] 看，单腿离地不置位。 */
    uint8_t control_enabled; /* 实际采纳的离地控制开关，OFFLINE刷新、ONLINE锁存；1不代表电机正在输出。 */
} GroundContact_t;

/* Keil Watch 调试入口：直接修改下列请求变量，观察 Chassis.ground 中的结果。
   抱车观测：遥控保持 OFFLINE，ground_control_enable=0，ground_offline_debug_enable=1。
   程序必须保持运行；暂停内核时没有新采样，确认计时和 Watch 结果也不会更新。
   先检查 valid[0/1]，再看 Fn/Fn_ratio 和 off_flag/all_off_flag。 */

/** 离地控制请求，默认0。
 *  0：检测只观测，不根据检测标志切换 LQR、腿长或 LESO 控制。
 *  1：允许 ONLINE/NORMAL 时使用离地控制；先在 OFFLINE 设置，再切 ONLINE。
 *  OFFLINE 周期将请求采纳到 Chassis.ground.control_enabled，在线修改请求不立即生效。
 *  此变量不替代遥控使能；OFFLINE 的最终电机命令仍为零。
 */
extern volatile uint8_t ground_control_enable;

/** OFFLINE 观测开关，默认0，修改后在下一个 OFFLINE 控制周期生效。
 *  0：OFFLINE 每拍复位检测；1：OFFLINE 继续估算和判定，忽略 NORMAL 模式限制。
 *  不启用离地控制、不改变零输出，也不绕过反馈在线、关节使能与几何有效性检查。
 *  调试结束置0；ONLINE 时此开关不起作用，自起/零力状态仍复位检测。
 *  抱车不保证 all_off_flag=1：零力矩且静止时，自重项约21.45N，Fn_ratio约0.215，
 *  略高于0.20离地门限；腿完全伸直也可能估算无效，因此要结合原始观测判断。
 */
extern volatile uint8_t ground_offline_debug_enable;

/* 清空检测历史和状态，保留本轮锁存的控制开关。 */
void GroundContact_Reset(void);
/* dt 为 DWT 实测秒数；accel_z 为大地系去重力加速度，m/s^2。
   torque_fb[腿][后/前] 是同拍原始电机反馈力矩，N*m；右腿镜像在模块内归一化。
   未使能关节的输入用 NAN 表示无效，不能把零力矩误当成离地证据。 */
void GroundContact_Update(float dt, float accel_z, const float torque_fb[2][2]);

#endif
