/**
 ******************************************************************************
 * @file    DM_Motor.h
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   DM系列电机驱动函数声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H

/* Includes ----------------------------------------------------------------- */
#include "main.h"
#include "bsp_can.h"
#include "pid.h"

/* Defines ------------------------------------------------------------------ */
#define Chassis_Wheel_J4310_Motor_LF_TxID		0x06		/* 关节4310左上电机接收ID */
#define Chassis_Wheel_J4310_Motor_LF_RxID		0x3		/* 关节4310左上电机发送ID */
														   
#define Chassis_Wheel_J4310_Motor_RF_TxID		0x08		/* 关节4310右上电机接收ID */
#define Chassis_Wheel_J4310_Motor_RF_RxID		0x4		/* 关节4310右上电机发送ID */
														   
#define Chassis_Wheel_J4310_Motor_RB_TxID		0x03		/* 关节4310右下电机接收ID */
#define Chassis_Wheel_J4310_Motor_RB_RxID		0x33		/* 关节4310右下电机发送ID */
														   
#define Chassis_Wheel_J4310_Motor_LB_TxID		0x04		/* 关节4310左下电机接收ID */
#define Chassis_Wheel_J4310_Motor_LB_RxID		0x44		/* 关节4310左下电机发送ID */

#define MIT_MODE 					0x000		/* MIT模式 */
#define POS_MODE					0x100		/* 位置模式 */
#define SPEED_MODE					0x200		/* 速度模式 */

#define KP_MIN    0.0f
#define KP_MAX    500.0f
#define KD_MIN    0.0f
#define KD_MAX    5.0f

#define J4310_P_MIN     -12.5f
#define J4310_P_MAX     12.5f
#define J4310_V_MIN     -30.0f
#define J4310_V_MAX     30.0f
#define J4310_T_MIN     -10.0f
#define J4310_T_MAX     10.0f

#define J4340_P_MIN     -12.5f
#define J4340_P_MAX     12.5f
#define J4340_V_MIN     -10.0f
#define J4340_V_MAX     10.0f
#define J4340_T_MIN 	-28.0f
#define J4340_T_MAX 	28.0f

#define J8009_P_MIN     -12.5f
#define J8009_P_MAX     12.5f
#define J8009_V_MIN     -45.0f
#define J8009_V_MAX     45.0f
#define J8009_T_MIN 	-40.0f
#define J8009_T_MAX 	40.0f


#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f


/* Enums -------------------------------------------------------------------- */
/**
 * @brief 电机初始化状态枚举
 */
typedef enum{
	DM_Init = 1,
	DM_DisInit = 0,
}DM_Motor_Init_e;

/**
 * @brief DM电机类型枚举
 */
typedef enum{
    DM_J4310 = 0,
    DM_J4340,
    DM_J8009,
    DM_MOTOR_TYPE_NUM,
}DM_Motor_Type_e;

/**
 * @brief DM电机模式枚举
 */
typedef enum{
    Mit_mode = 0x000,
    Pos_mode = 0x100,
    Spd_mode = 0x200,
}DM_Motor_mode_e;

/* Structs ------------------------------------------------------------------ */
/**
 * @brief DM电机ID结构体
 */
typedef struct
{
  uint32_t TxIdentifier;	/* FDCAN发送标识符 */
  uint32_t RxIdentifier;	/* FDCAN接收标识符 */
}DM_Motor_ID_Typedef;

/**
 * @brief DM电机数据结构体
 */
typedef struct 
{
	uint16_t id;			/* 电机ID */
	uint16_t state;			/* 电机状态 */
	int p_int;			    /* 位置整数 */
	int v_int;			    /* 速度整数 */
	int t_int;			    /* 扭矩整数 */
	int kp_int;			    /* KP整数 */
	int kd_int;			    /* KD整数 */
	float pos;			    /* 位置 */
	float vel;			    /* 速度 */
	float tor;			    /* 扭矩 */
	float Kp;			    /* KP系数 */
	float Kd;			    /* KD系数 */
	float Tmos;			    /* MOS管温度 */
	float Tcoil;			/* 线圈温度 */
}DM_Motor_Data_Typedef;

/**
 * @brief DM电机信息结构体
 */
typedef struct
{
    DM_Motor_mode_e Mode;		    /* 电机模式 */
    DM_Motor_Init_e Motor_state;	/* 电机状态 */
    DM_Motor_Type_e Motor_Type;	    /* 电机类型 */
    DM_Motor_ID_Typedef ID_Set;	    /* ID设置 */
    DM_Motor_Data_Typedef Data;	    /* 数据 */
}DM_Motor_Info_Typedef;

/**
 * @brief DM电机控制结构体
 */
typedef struct{
    Pid_Set_Typedef Angle_set;
    Pid_Set_Typedef Speed_set;
    PidTypedef Angle_pid;
    PidTypedef Speed_pid;
} DM_Motor_Ctrl_Typedef;

/* Externs ------------------------------------------------------------------ */

/* Functions ---------------------------------------------------------------- */
uint16_t DM_Enable_Motor(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, uint8_t delay_time);
uint16_t DM_Disable_Motor(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, uint8_t delay_time);
void DM_Save_Motor_Zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, uint8_t delay_time);
void DM_Motor_Info_Update(DM_Motor_Info_Typedef *motor, uint8_t *rx_data,uint32_t data_len);
void DM_Motor_Ctrl(hcan_t *hcan, volatile DM_Motor_Info_Typedef *motor, float pos, float vel, float kp, float kd, float torq, uint8_t delay_time);
float Hex_To_Float(uint32_t *Byte,int num);
uint32_t FloatTohex(float HEX);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);

/* -------------------------------------------------------------------------- */
#endif /* __DM_MOTOR_H */
