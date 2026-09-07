#include "chassis_task.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "usart.h"
#include "DM_Motor.h"
#include "fdcan.h"
#include "pid.h"
#include "VMC_calc.h"
#include "INS_task.h"
#include "LQR.h"
#include "some_para.h"
#include "bsp_dwt.h"  
#include "LESO.h"

/* 任务运行周期 */
#define Chassis_Time						1		

#define mirror -1.0f

void chassis_feedback_update(void);
void LQR(void);
void VMC_translate(void);
void Chassis_CanTransimit(void);
void LEG_Lenth_Control(void);
void YAW_Parameter_Processing(void);


PidTypedef LegLenth_Left_Pid;
PidTypedef LegLenth_Right_Pid;

float fb_dt;

extern INS_t INS;
Chassis_Info_Typedef Chassis = {
	.Joint_Motor[LEFT_FRONT_id] = 
	{
		.Motor_Type = DM_J8009,
		.Mode = Mit_mode, 
		.ID_Set = {
			.TxIdentifier = JOINT_DM_LEFT_FRONT_TxID_Set,
			.RxIdentifier = JOINT_DM_LEFT_FRONT_RxID_Set,
		},
	},
	.Joint_Motor[RIGHT_FRONT_id] = 
	{
		.Motor_Type = DM_J8009,
		.Mode = Mit_mode,
		.ID_Set = {
			.TxIdentifier = JOINT_DM_RIGHT_FRONT_TxID_Set,
			.RxIdentifier = JOINT_DM_RIGHT_FRONT_RxID_Set,
		},
	},
	.Joint_Motor[RIGHT_BACK_id] = 
	{
		.Motor_Type = DM_J8009,
		.Mode = Mit_mode,
		.ID_Set = {
			.TxIdentifier = JOINT_DM_RIGHT_BACK_TxID_Set,
			.RxIdentifier = JOINT_DM_RIGHT_BACK_RxID_Set,
		},
	},
	.Joint_Motor[LEFT_BACK_id] = 
	{
		.Motor_Type = DM_J8009,
		.Mode = Mit_mode,
		.ID_Set = {
			.TxIdentifier = JOINT_DM_LEFT_BACK_TxID_Set,
			.RxIdentifier = JOINT_DM_LEFT_BACK_RxID_Set,
		},
	},
	.Wheel_Motor[LEFT_Wheel] = 
	{
		.Motor_Type = DJI_M3508,
        .ID_Set = {
            .TxIdentifier = Chassis_3508_MotorA_TxID,
            .RxIdentifier = Chassis_3508_Motor2_RxID,
        }
	},
	.Wheel_Motor[RIGHT_Wheel] = {
        .Motor_Type = DJI_M3508,
        .ID_Set = {
            .TxIdentifier = Chassis_3508_MotorA_TxID,
            .RxIdentifier = Chassis_3508_Motor1_RxID,
        }
    },

};



static void Chassis_init(void);

void chassis_task(void)
{
	while(INS.ins_flag==0)
	{
	  osDelay(1);	
	}
	Chassis_init();
    while (1)
    {

		chassis_feedback_update();
		
		YAW_Parameter_Processing();
		LQR();
		LESO_Service();
		LEG_Lenth_Control();
		VMC_translate();
		Chassis_CanTransimit();

		//osDelay(Chassis_Time);
		
    }
}

static void Compensate_PID_Init(void)
{
	const static float leglenth_right_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
	const static float leglenth_left_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
	
 
	PID_init(&LegLenth_Left_Pid, PID_POSITION,leglenth_left_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid
	PID_init(&LegLenth_Right_Pid, PID_POSITION,leglenth_right_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid
	
}



static void DM_Enable(void)
{
	uint8_t Enable_DelayTime = 1;
	for (uint8_t i = 0; i < 10; i++)
    {
		
		
		DM_Enable_Motor(&LEFT_Joint_Motor_CAN_hfdcan,Chassis.Joint_Motor[LEFT_FRONT_id].ID_Set.TxIdentifier,Chassis.Joint_Motor[LEFT_FRONT_id].Mode,Enable_DelayTime);
		DM_Enable_Motor(&LEFT_Joint_Motor_CAN_hfdcan,Chassis.Joint_Motor[LEFT_BACK_id].ID_Set.TxIdentifier,Chassis.Joint_Motor[LEFT_BACK_id].Mode,Enable_DelayTime);
		DM_Enable_Motor(&RIGHT_Joint_Motor_CAN_hfdcan,Chassis.Joint_Motor[RIGHT_FRONT_id].ID_Set.TxIdentifier,Chassis.Joint_Motor[RIGHT_FRONT_id].Mode,Enable_DelayTime);
		DM_Enable_Motor(&RIGHT_Joint_Motor_CAN_hfdcan,Chassis.Joint_Motor[RIGHT_BACK_id].ID_Set.TxIdentifier,Chassis.Joint_Motor[RIGHT_BACK_id].Mode,Enable_DelayTime);
		
	}
	
}

static void Chassis_init(void)
{
	DM_Enable();
	VMC_init(&Chassis.leg_situation[LEFT_Leg].vmc);
	VMC_init(&Chassis.leg_situation[RIGHT_Leg].vmc);
	Compensate_PID_Init();
	Chassis.chassis_mode = offline;
	Chassis.set_goal.roll_set = 0.0f;
}

void chassis_feedback_update(void)
{
	static uint32_t fb_dwt_cnt = 0;
	fb_dt = DWT_GetDeltaT(&fb_dwt_cnt);   /* 本周期实际耗时(秒)，DWT实测 */
	if (fb_dt > 0.05f || fb_dt <= 0.0f)
		fb_dt = Chassis_Time * 0.001f;          /* 首拍/异常时回退到名义周期 */
	Chassis.leg_situation[LEFT_Leg].last_wheel_s =   Chassis.leg_situation[LEFT_Leg].wheel_s;
	Chassis.leg_situation[LEFT_Leg].wheel_s      = -(Chassis.Wheel_Motor[LEFT_Wheel].Data.Velocity/(Gear_Ratio*60.0f))*2.0f*PI*wheel_R;
	Chassis.leg_situation[RIGHT_Leg].last_wheel_s =   Chassis.leg_situation[RIGHT_Leg].wheel_s;
	Chassis.leg_situation[RIGHT_Leg].wheel_s      = ( Chassis.Wheel_Motor[RIGHT_Wheel].Data.Velocity/(Gear_Ratio*60.0f))*2.0f*PI*wheel_R;

	Chassis.body_state.yaw     =  INS.Yaw;
	Chassis.body_state.d_yaw   =  INS.Gyro[2];
	Chassis.body_state.theta   =  INS.Pitch;
	Chassis.body_state.d_theta =  -INS.Gyro[1];
	Chassis.body_state.roll = INS.Roll;
	Chassis.body_state.d_roll = -INS.Gyro[0];

	Chassis.leg_situation[LEFT_Leg].vmc.phi4 =pi/2 +  Chassis.Joint_Motor[LEFT_FRONT_id].Data.pos;
	Chassis.leg_situation[LEFT_Leg].vmc.phi1 = pi/2 + Chassis.Joint_Motor[LEFT_BACK_id].Data.pos;
	Chassis.leg_situation[RIGHT_Leg].vmc.phi4 =pi/2 + mirror*Chassis.Joint_Motor[RIGHT_FRONT_id].Data.pos;
	Chassis.leg_situation[RIGHT_Leg].vmc.phi1 = pi/2 + mirror*Chassis.Joint_Motor[RIGHT_BACK_id].Data.pos;
	Chassis.leg_situation[LEFT_Leg].vmc.d_phi1 = Chassis.Joint_Motor[LEFT_BACK_id].Data.vel;
	Chassis.leg_situation[LEFT_Leg].vmc.d_phi4 = Chassis.Joint_Motor[LEFT_FRONT_id].Data.vel;
	Chassis.leg_situation[RIGHT_Leg].vmc.d_phi1 = mirror*Chassis.Joint_Motor[RIGHT_BACK_id].Data.vel;
	Chassis.leg_situation[RIGHT_Leg].vmc.d_phi4 = mirror*Chassis.Joint_Motor[RIGHT_FRONT_id].Data.vel;

	VMC_calc_1(&Chassis.leg_situation[LEFT_Leg].vmc,&INS,fb_dt);
	VMC_calc_1(&Chassis.leg_situation[RIGHT_Leg].vmc,&INS,fb_dt);
	
	Chassis.leg_situation[LEFT_Leg].stator_s = Chassis.leg_situation[LEFT_Leg].wheel_s + Chassis.leg_situation[LEFT_Leg].vmc.d_theta*wheel_R; ///未打滑的理想情况下,定子相对于地面的速度
	Chassis.leg_situation[RIGHT_Leg].stator_s = Chassis.leg_situation[RIGHT_Leg].wheel_s + Chassis.leg_situation[RIGHT_Leg].vmc.d_theta*wheel_R; ///未打滑的理想情况下,定子相对于地面的速度
	Chassis.leg_situation[LEFT_Leg].swing_s  = Chassis.leg_situation[LEFT_Leg].vmc.d_L0*arm_sin_f32(Chassis.leg_situation[LEFT_Leg].vmc.theta) + Chassis.leg_situation[LEFT_Leg].vmc.L0*(Chassis.leg_situation[LEFT_Leg].vmc.d_theta)*arm_cos_f32(Chassis.leg_situation[LEFT_Leg].vmc.theta); ////摆杆相对于机体的速度
	Chassis.leg_situation[RIGHT_Leg].swing_s  = Chassis.leg_situation[RIGHT_Leg].vmc.d_L0*arm_sin_f32(Chassis.leg_situation[RIGHT_Leg].vmc.theta) + Chassis.leg_situation[RIGHT_Leg].vmc.L0*(Chassis.leg_situation[RIGHT_Leg].vmc.d_theta)*arm_cos_f32(Chassis.leg_situation[RIGHT_Leg].vmc.theta);

	//车体估计值获取
	Chassis.body_state.Estimate_dx   = ( Chassis.leg_situation[LEFT_Leg].stator_s + Chassis.leg_situation[LEFT_Leg].swing_s
											+ Chassis.leg_situation[RIGHT_Leg].stator_s + Chassis.leg_situation[RIGHT_Leg].swing_s )/2.0f; //估计的车体相对于地面速度
											
	Chassis.body_state.Estimate_dyaw   = (Chassis.leg_situation[RIGHT_Leg].wheel_s - Chassis.leg_situation[LEFT_Leg].wheel_s)/L_wheel; //估计的车体YAW速度
	
	Chassis.body_state.Estimate_h    = 0.5f*(Chassis.leg_situation[LEFT_Leg].vmc.L0 + Chassis.leg_situation[RIGHT_Leg].vmc.L0);  //估计的车体高度

	if(fabs(Chassis.set_goal.v_set) >= 0.05f|| Chassis.chassis_mode == offline)        // 遥控有速度指令（在动）
	{
   	 	Chassis.chassis_flag.start_situate_flag = 0;
    	Chassis.body_state.x = 0;                      // 运动中：位置清零，不积分
	//	Chassis.body_state.x += Chassis.body_state.Estimate_dx*fb_dt;       // 位置 += 速度*周期
	}
	else if(((fabs(Chassis.set_goal.v_set)<=0.05f && fabs(Chassis.body_state.Estimate_dx)<=0.2f)) || Chassis.chassis_flag.start_situate_flag)
	{
    	Chassis.chassis_flag.start_situate_flag = 1;           // 静止：开始/继续积分
  		Chassis.body_state.x += Chassis.body_state.Estimate_dx*fb_dt;       // 位置 += 速度*周期
	}
	else                                  // 其他异常状态
	{
     	Chassis.chassis_flag.start_situate_flag = 0;
    	Chassis.body_state.x = 0;
	}
}

void YAW_Parameter_Processing(void)
{
	if((fabsf(Chassis.set_goal.yaw_set_v) > 0.1f) || Chassis.chassis_mode == offline)
    	{
			Chassis.set_goal.yaw_set = Chassis.body_state.yaw;
		}
}

void LQR(void)
{
	LQR_Calc(Chassis.leg_situation[LEFT_Leg].vmc.L0,Chassis.leg_situation[RIGHT_Leg].vmc.L0);
}

void VMC_translate(void)
{
	VMC_calc_2(&Chassis.leg_situation[LEFT_Leg].vmc);
	VMC_calc_2(&Chassis.leg_situation[RIGHT_Leg].vmc);
}
void Chassis_CanTransimit(void)
{
	if(Chassis.chassis_mode == online)
	{
		mySaturate(&Chassis.leg_situation[LEFT_Leg].vmc.torque_set[0], J8009_T_MIN, J8009_T_MAX);
		mySaturate(&Chassis.leg_situation[LEFT_Leg].vmc.torque_set[1], J8009_T_MIN, J8009_T_MAX);
		mySaturate(&Chassis.leg_situation[RIGHT_Leg].vmc.torque_set[0], J8009_T_MIN, J8009_T_MAX);
		mySaturate(&Chassis.leg_situation[RIGHT_Leg].vmc.torque_set[1], J8009_T_MIN, J8009_T_MAX);
		DM_Motor_Ctrl(&LEFT_Joint_Motor_CAN_hfdcan,&Chassis.Joint_Motor[LEFT_FRONT_id],0,0,0,0,Chassis.leg_situation[LEFT_Leg].vmc.torque_set[1],0);
		DM_Motor_Ctrl(&LEFT_Joint_Motor_CAN_hfdcan,&Chassis.Joint_Motor[LEFT_BACK_id],0,0,0,0,Chassis.leg_situation[LEFT_Leg].vmc.torque_set[0],0);
		osDelay(Chassis_Time);
		DM_Motor_Ctrl(&RIGHT_Joint_Motor_CAN_hfdcan,&Chassis.Joint_Motor[RIGHT_FRONT_id],0,0,0,0,-Chassis.leg_situation[RIGHT_Leg].vmc.torque_set[1],0);
		DM_Motor_Ctrl(&RIGHT_Joint_Motor_CAN_hfdcan,&Chassis.Joint_Motor[RIGHT_BACK_id],0,0,0,0,-Chassis.leg_situation[RIGHT_Leg].vmc.torque_set[0],0);

		mySaturate(&Chassis.Wheel_Motor[LEFT_Wheel].wheel_T,-4.8f,4.8f);
		mySaturate(&Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T,-4.8f,4.8f);
//		mySaturate(&Chassis.Wheel_Motor[LEFT_Wheel].wheel_T,-0.1f,0.1f);
//		mySaturate(&Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T,-0.1f,0.1f);
		 LESO_Feedback(Chassis.Wheel_Motor[LEFT_Wheel].wheel_T, 
                        Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T,
                        Chassis.leg_situation[LEFT_Leg].vmc.Tp,
                        Chassis.leg_situation[RIGHT_Leg].vmc.Tp);
		Chassis.Wheel_Motor[LEFT_Wheel].Data.SET_Current = Chassis.Wheel_Motor[LEFT_Wheel].wheel_T*LeftWheelT_TO_Current;
		Chassis.Wheel_Motor[RIGHT_Wheel].Data.SET_Current = Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T*RightWheelT_TO_Current;
		VAL_LIMIT(Chassis.Wheel_Motor[LEFT_Wheel].Data.SET_Current,-16384,16384);
		VAL_LIMIT(Chassis.Wheel_Motor[RIGHT_Wheel].Data.SET_Current,-16384,16384);

		DJI_Motor_ctrl(Chassis.Wheel_Motor,&LEFT_Wheel_CAN_hfdcan,0);
	}else if(Chassis.chassis_mode == offline)
	{
		DM_Motor_Ctrl(&LEFT_Joint_Motor_CAN_hfdcan,&Chassis.Joint_Motor[LEFT_FRONT_id],0,0,0,0,0,Chassis_Time);
		DM_Motor_Ctrl(&LEFT_Joint_Motor_CAN_hfdcan,&Chassis.Joint_Motor[LEFT_BACK_id],0,0,0,0,0,Chassis_Time);
		DM_Motor_Ctrl(&RIGHT_Joint_Motor_CAN_hfdcan,&Chassis.Joint_Motor[RIGHT_FRONT_id],0,0,0,0,0,Chassis_Time);
		DM_Motor_Ctrl(&RIGHT_Joint_Motor_CAN_hfdcan,&Chassis.Joint_Motor[RIGHT_BACK_id],0,0,0,0,0,Chassis_Time);
		Chassis.Wheel_Motor[LEFT_Wheel].Data.SET_Current = 0;
		Chassis.Wheel_Motor[RIGHT_Wheel].Data.SET_Current = 0;
		DJI_Motor_ctrl(Chassis.Wheel_Motor,&LEFT_Wheel_CAN_hfdcan,Chassis_Time);
	}
}

void LEG_Lenth_Control(void)
{
	float roll_err  = Chassis.set_goal.roll_set - Chassis.body_state.roll;           /* rad，EKF 横滚角 */
    float roll_rate = Chassis.set_goal.roll_set_v - Chassis.body_state.d_roll;        /* rad/s，横滚角速度轴号/符号需实测确认 */
    float F_roll    = ROLL_PID_KP * roll_err + ROLL_PID_KD * roll_rate;
	mySaturate(&F_roll,-ROLL_PID_MAX_OUT,ROLL_PID_MAX_OUT);
	
	Chassis.leg_situation[LEFT_Leg].vmc.F0 = (body_mg/2.0f)*arm_cos_f32(Chassis.leg_situation[LEFT_Leg].vmc.theta)+PID_Calc(&LegLenth_Left_Pid,Chassis.leg_situation[LEFT_Leg].vmc.L0,Chassis.set_goal.set_L0_Left) + F_roll;
	Chassis.leg_situation[RIGHT_Leg].vmc.F0 = (body_mg/2.0f)*arm_cos_f32(Chassis.leg_situation[RIGHT_Leg].vmc.theta)+PID_Calc(&LegLenth_Right_Pid,Chassis.leg_situation[RIGHT_Leg].vmc.L0,Chassis.set_goal.set_L0_Right) - F_roll;
//	Chassis.leg_situation[LEFT_Leg].vmc.F0 = PID_Calc(&LegLenth_Left_Pid,Chassis.leg_situation[LEFT_Leg].vmc.L0,0.15f);
//	Chassis.leg_situation[RIGHT_Leg].vmc.F0 = PID_Calc(&LegLenth_Right_Pid,Chassis.leg_situation[RIGHT_Leg].vmc.L0,0.15f);
}


