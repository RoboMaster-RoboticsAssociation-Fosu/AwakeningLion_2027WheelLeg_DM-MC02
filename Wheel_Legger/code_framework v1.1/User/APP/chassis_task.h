#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "main.h"
#include "DM_Motor.h"
#include "DJI_Motor.h"
#include "VMC_calc.h"
/*********************************/
#define LEG_LENGTH_RAMP_RATE_M_S 0.15f /* Balance length reference speed (m/s). */

#define LEG_PID_KP  2000.0f
#define LEG_PID_KI  0.0f
//#define LEG_PID_KD  800.0f
/* 腿长环速率增益，直接作用在 vmc.d_L0 上，单位 N*s/m。
   原 LEG_PID_KD 走 PID_Calc 的 Kd*(error[0]-error[1])，差分没除 dt，
   等效速率增益 = LEG_PID_KD * 循环周期T。T=4ms 时为 16，T=6ms 时为 24。
   先读实测 fb_dt，按 LEG_PID_KD * fb_dt 取初值再调。
   改成这个形式后腿长环微分增益不再随循环周期变化。 */
#define LEG_PID_KD_RATE  200.0f
#define LEG_PID_MAX_OUT  90.0f //90牛
#define LEG_PID_MAX_IOUT 0.0f

#define ROLL_PID_KP 340.0f
#define ROLL_PID_KD 80.0f
//#define ROLL_PID_KP 0.0f
//#define ROLL_PID_KD 0.0f
#define ROLL_PID_MAX_OUT  60.0f
#define ROLL_PID_MAX_IOUT 0.0f

#define body_mg (16.0f*9.8f)

#define LeftWheelT_TO_Current -3330.0f //左轮子力矩转电流
#define RightWheelT_TO_Current 3330.0f //右轮子力矩转电流
/*Torque = I / 16384 × 20 × 0.3 / (3591/187) × (268/17)
       = I × (20/16384) × 0.3 × (187/3591) × (268/17)
       = I × 0.001221 × 0.3 × 0.05207 × 15.765
       = I × 0.0003007        [N·m per CAN单位]
发送端（取倒数）：


I = T × 16384/20 / 0.3 × (3591/187) / (268/17)
  = T × 1 / 0.0003007
  = T × 3326   ≈ 3330*/

#define LEFT_Joint_Motor_CAN_hfdcan  hfdcan2
#define RIGHT_Joint_Motor_CAN_hfdcan  hfdcan1

#define LEFT_Wheel_CAN_hfdcan  hfdcan1
#define RIGHT_Wheel_CAN_hfdcan  hfdcan1



#define JOINT_DM_LEFT_FRONT_TxID_Set  0x04
#define JOINT_DM_RIGHT_FRONT_TxID_Set 0x01
#define JOINT_DM_RIGHT_BACK_TxID_Set  0x02
#define JOINT_DM_LEFT_BACK_TxID_Set   0x03


#define JOINT_DM_LEFT_FRONT_RxID_Set  0x14
#define JOINT_DM_RIGHT_FRONT_RxID_Set  0x11
#define JOINT_DM_RIGHT_BACK_RxID_Set  0x12
#define JOINT_DM_LEFT_BACK_RxID_Set   0x13

typedef struct
{	
	vmc_leg_t vmc;

	float wheel_s;          //轮子转速
	float last_wheel_s;
	float swing_s;          //摆杆的速度
	float stator_s;					//定子的速度
	
}Leg_Situation_t;

typedef struct
{
   	float roll;          // 横滚角
	float d_roll;         // 横滚角速度
   	float yaw;           // 偏航角度 
	float d_yaw;         // 偏航角度速度

	float theta;         // 俯仰角度
	float d_theta;       // 俯仰角度速度
	
	float x;			//车体位置，单位是m
	float Estimate_dx; //估计的车体相对于地面速度，单位是m/s
	float Estimate_h; //估计的车体高度，单位是m
	float Estimate_dyaw; //估计的车体偏航角度速度，单位是弧度/s

} BodyState_t;

typedef struct
{
	float v_set;//期望速度，单位是m/s
	float x_set;//期望位置，单位是m

	float yaw_set;//期望偏航角度，单位是弧度
	float yaw_set_v;//期望偏航角度速度，单位是弧度/s

	float roll_set;//期望roll角，单位是弧度
	float roll_set_v;//期望roll角速度，单位是弧度/s

	float set_L0_Left;//期望腿长，单位是m
	float set_L0_Right;//期望腿长，单位是m
	
	float set_phi0_Left;//期望摆角，单位是弧度
	float set_phi0_Right;//期望摆角，单位是弧度


} SetGoal_t;

typedef struct
{
	uint8_t start_situate_flag;//是否开始/继续积分
}Chassis_flag_t;

typedef enum{
	OFFLINE,
	ONLINE,
}Chassis_Enable_e;
typedef enum{
	FALLING_DOWN,
	FALLING_TO_NORMAL,
	NORMAL,
	ZERO_FORCE,
}Chassis_Mode_e;



typedef struct
{
    DM_Motor_Info_Typedef Joint_Motor[4];
	DJI_Motor_Info_Typedef Wheel_Motor[4];
 	
    SetGoal_t set_goal;

	Leg_Situation_t leg_situation[2];
	
	BodyState_t body_state;
	Chassis_flag_t chassis_flag;
	
	Chassis_Mode_e chassis_mode;
	Chassis_Enable_e chassis_enable;

} Chassis_Info_Typedef;

typedef enum {
	LEFT_FRONT_id = JOINT_DM_LEFT_FRONT_TxID_Set - 1,
	RIGHT_FRONT_id = JOINT_DM_RIGHT_FRONT_TxID_Set - 1,
	RIGHT_BACK_id = JOINT_DM_RIGHT_BACK_TxID_Set - 1,
	LEFT_BACK_id= JOINT_DM_LEFT_BACK_TxID_Set - 1
} Chassis_Joint_ID_e;

typedef enum {
	LEFT_Leg = 0,
	RIGHT_Leg = 1,
} Chassis_Joint_VMC_e;

typedef enum {
	LEFT_Wheel = 1,
	RIGHT_Wheel = 0
} Chassis_Wheel_e;


/*******************************************************************************************************
寻找最小弧度
********************************************************************************************************/


extern Chassis_Info_Typedef Chassis;



void chassis_task(void);

#endif
