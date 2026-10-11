#ifndef __USER_SYS_CONFIG_H
#define __USER_SYS_CONFIG_H

#include "cmsis_os.h"
#include "user_sys_config.h"
#include <stm32h7xx.h>

/* 控制来源二选一：修改 CHASSIS_CTRL_SOURCE 后重新编译、烧录，也可通过编译器宏覆盖。 */
/* 本地遥控：遥控接收机直接连接底盘 UART5，底盘独立运行或单独调试时使用。 */
#define CHASSIS_CTRL_SOURCE_LOCAL  0
/* 云台通信：遥控接收机连接云台板，由云台通过 CAN 向底盘 FDCAN3 转发遥控值时使用。 */
#define CHASSIS_CTRL_SOURCE_GIMBAL 1
#ifndef CHASSIS_CTRL_SOURCE
/* 默认使用本地遥控；需要云台控制时，将下一行的 LOCAL 配置改为 CHASSIS_CTRL_SOURCE_GIMBAL。 */
#define CHASSIS_CTRL_SOURCE CHASSIS_CTRL_SOURCE_LOCAL
#endif
#if CHASSIS_CTRL_SOURCE != CHASSIS_CTRL_SOURCE_LOCAL && \
    CHASSIS_CTRL_SOURCE != CHASSIS_CTRL_SOURCE_GIMBAL
#error "Invalid CHASSIS_CTRL_SOURCE"
#endif
/* Reject undefined selector symbols instead of treating them as zero. */
enum { CHASSIS_CTRL_SOURCE_SELECTED = CHASSIS_CTRL_SOURCE };

/* ϵͳDEBUG��־λ */
#define DEBUG						1

/* ϵͳ��ʱ����ʱ��(ms) */
#define SYS_DELAY_START_TIME		1000

/* Rad ת Ang*/
#define RAD_TO_ANG					(180.f / PI)
/* Ang ת Rad */
#define ANG_TO_RAD					(PI / 180.f)
/* �����ٶ�����(m/s) */


/* ���ҹؽڵ���޽� */
#define Chassis_Leg_Angle_min		(-0.7)
#define Chassis_Leg_Angle_max		(0.7)

/* ��������㱣�� */
#define Save_Motor_Zero_flag		0

#endif
