#ifndef __LESO_H
#define __LESO_H

#include "main.h"
#include "INS_task.h"

  /* 维数与 ABK_LQR.py 一致: 10状态 / 4输入 / 4扩张扰动 */
  #define LESO_N_STATE 10
  #define LESO_N_INPUT 4
  #define LESO_N_TOTAL (LESO_N_STATE + LESO_N_INPUT)   /* 14 */
  
/* ====== 参数区 ====== */

typedef enum
{
    LESO_WHEEL_OFF = 0,
    LESO_WHEEL_COMMON = 1,
    LESO_WHEEL_DIFF = 2,
    LESO_WHEEL_FULL = 3
} LESO_WheelMode_e;

extern volatile uint8_t leso_wheel_mode; /* OFF / COMMON / DIFF / FULL. */
extern volatile float leso_wheel_scale; /* Wheel injection target, 0..1. */
extern volatile float leso_leg_scale;   /* Hip injection target, 0..1. */
extern volatile float leso_dbg_comp_wheel; /* Maximum actual component scale. */
extern volatile float leso_dbg_comp_wheel_common;
extern volatile float leso_dbg_comp_wheel_diff;
extern volatile float leso_dbg_dh_wheel_common; /* Raw common estimate, N*m. */
extern volatile float leso_dbg_dh_wheel_diff;   /* Raw differential estimate, N*m. */
extern volatile float leso_dbg_comp_leg;   /* Ramped actual hip scale. */
extern volatile unsigned char leso_active; /* Any valid injection active. */
extern const float leso_dlim[4];     /* 扰动限幅 N·m [左轮,右轮,左Tp,右Tp] */

/* 站稳门限（进入注入的条件） */
#define LESO_GATE_DX      0.1f       /* 机体速度 m/s */
#define LESO_GATE_DTHETA  0.5f       /* 腿摆角速度 rad/s */
#define LESO_COMP_RATE    0.003f     /* comp 涓�闃舵枩鍧＄郴鏁帮紝1kHz 涓嬬害 0.33s */
#define LESO_GATE_TIME    0.5f       /* 持续时间 s */



typedef struct
{
  float xh[LESO_N_STATE];   /* 状态估计 x?[10] */
  float dh[LESO_N_INPUT];   /* 扰动估计 d?[4]: [左轮, 右轮, 左Tp, 右Tp], 单位 N·m */
  float e[LESO_N_STATE];    /* 残差 y ? x? */
} LESO_t;
 

void LESO_Init(LESO_t *o);
void LESO_Seed(LESO_t *o, const float *y);
void LESO_Update(LESO_t *o, const float *Ad, const float *Bd, const float *L,
			   const float *y, const float *u, const float *dlim);

void LESO_Service(void);   /* 主循环: LQR() 之后调用 */
void LESO_Feedback(float tw_l, float tw_r, float tpl, float tpr);
						 /* Chassis_CanTransimit 轮力矩限幅之后调用 */

#endif
