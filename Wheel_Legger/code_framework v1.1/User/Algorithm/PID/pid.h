/**
 ******************************************************************************
 * @file    pid.h
 * @version V1.0.0
 * @date    2026.03.05
 * @brief   PID控制器功能声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef PID_H
#define PID_H

/* Includes ----------------------------------------------------------------- */
#include "main.h"
#include "stdbool.h"

/* Defines ------------------------------------------------------------------ */

/* Enums -------------------------------------------------------------------- */
/**
  * @brief PID模式枚举
  */
typedef enum
{
    PID_POSITION = 0,		// 位置式PID
    PID_DELTA				    // 增量式PID
} PID_mode_e;

/* Structs ------------------------------------------------------------------ */
/**
  * @brief PID结构体
  */
typedef struct
{
    PID_mode_e pid_mode;		// PID模式
	bool	Initlized;			    // 初始化标志
    fp32 Kp;				      	// 比例系数
    fp32 Ki;				        // 积分系数
    fp32 Kd;				        // 微分系数
    fp32 max_out;		       	// 最大输出
    fp32 max_iout;	      	// 积分输出最大值
    fp32 set;				        // 目标值
    fp32 fdb;				        // 反馈值
    fp32 out;				        // 总输出
    fp32 Pout;			        // P项输出
    fp32 Iout;			        // I项输出
    fp32 Dout;				      // D项输出
    fp32 Dbuf[3];			    	// 微分缓冲 0:当前 1:上次 2:上上次
    fp32 error[3];		    	// 误差 0:当前 1:上次 2:上上次
} PidTypedef;

/**
  * @brief PID设置结构体
  */
typedef struct
{
	float ref;					  // 目标值
	float fdb;					  // 反馈值
	float PID_Init[3];		// PID初始化参数 {Kp, Ki, Kd}
} Pid_Set_Typedef;

/* Externs ------------------------------------------------------------------ */

/* Functions ---------------------------------------------------------------- */
void PID_init(PidTypedef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
fp32 PID_Calc(PidTypedef *pid, fp32 fdb, fp32 ref);
void PID_clear(PidTypedef *pid);

/* -------------------------------------------------------------------------- */
#endif /* PID_H */
