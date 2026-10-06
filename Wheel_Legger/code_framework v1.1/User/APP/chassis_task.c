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
#include "leg_motion.h"
#include <math.h>
#include <stddef.h>


#define Chassis_Time						1		

#define mirror -1.0f

/* 方向按 phi0 增减定义，左右腿使用同一套物理坐标。 */
#define SPIN_SWEEP_DIR LEG_MOTION_NEGATIVE
/* 腿垂直向下为 0 度，目标对应原来的 phi0 约 2.254 rad。 */
#define SPIN_TARGET_ANGLE_DEG 39.143f
#define SPIN_RAMP_TIME_MS 4000U
#define SPIN_RETRACT_TIME_MS 1000U
#define SPIN_EARLY_ANGLE 0.9f
#define SPIN_ANGLE_WINDOW LEG_MOTION_ANGLE_TOLERANCE_RAD
#define SPIN_SCAN_LENGTH 0.30f
#define SPIN_RETRACT_LENGTH 0.13f
#define SPIN_LENGTH_TOLERANCE LEG_MOTION_LENGTH_TOLERANCE_M
#define SPIN_SCAN_TIMEOUT_MS 600000U
#define SPIN_HANDOFF_TIMEOUT_MS 1500U

void chassis_feedback_update(void);
void LQR(void);
void VMC_translate(void);
void Chassis_CanTransimit(void);
void LEG_Lenth_Control(void);
void YAW_Parameter_Processing(void);
void normal_mode(void);
void falling_to_down(void);
void falling_down(void);
void falling_down_detect(void);
void zero_force(void);
static void chassis_zero_outputs(void);
static void chassis_recovery_reset(void);
static void chassis_recovery_abort(void);


PidTypedef LegLenth_Left_Pid;
PidTypedef LegLenth_Right_Pid;



float F_roll;

float fb_dt;

/* ---- 周期分段计时（临时插桩，用于定位 fb_dt = 6 ms 的去向）----
   t_sum 是任务自身的执行时间。若 t_sum 远小于 fb_dt，说明是被高优先级
   任务抢占，而不是自己慢。定位完可整段删除。 */
float t_fb, t_mode, t_leso, t_can, t_sum;

typedef enum
{
    RECOVERY_SWING = 0,
    RECOVERY_RETRACT,
    RECOVERY_HOLD
} Recovery_Phase;

/* 左右腿分别保存动作状态，可在调试器中观察。 */
LegMotion_Context chassis_leg_motion[2];
static Recovery_Phase recovery_phase[2];
static uint8_t recovery_started;
static uint8_t recovery_output_pending;
static uint32_t recovery_entry_tick;
static uint32_t recovery_handoff_tick;

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
	/* 绝对定时：osDelay 是相对延时，周期会变成 1ms + 本拍执行时间并随之抖动；
	   osDelayUntil 把唤醒点钉在固定节拍上。CMSIS-RTOS v1 的原型是
	   osDelayUntil(uint32_t *PreviousWakeTime, uint32_t millisec)，两个参数，
	   PreviousWakeTime 由函数内部自动累加，循环里不要再赋值。
	   注意：依赖 FreeRTOSConfig.h 的 INCLUDE_vTaskDelayUntil = 1，为 0 时这个
	   函数直接返回 osErrorResource，什么也不做。 */
	uint32_t PreviousWakeTime;

	while(INS.ins_flag==0)
	{
	  osDelay(1);	
	}
	Chassis_init();

	PreviousWakeTime = osKernelSysTick();	/* 进循环前取一次基准 */

    while (1)
    {
		{ static uint32_t prof_cnt = 0; DWT_GetDeltaT(&prof_cnt);

		chassis_feedback_update();//数据更新
		t_fb = DWT_GetDeltaT(&prof_cnt);
		
		YAW_Parameter_Processing();//yaw的目标角设置
		falling_down_detect();//倒地检测
		if(Chassis.chassis_enable == OFFLINE)
		{
			/* enable off: zero output (cleared in Chassis_CanTransimit) */
		}
		

		
		switch(Chassis.chassis_mode)//不同状态下的切换
		{
			case ZERO_FORCE:
			{
				zero_force();
			}break;                 /* 零力状态到此结束，不继续执行自起。 */
			case FALLING_DOWN:
			{
				falling_down();
			}break;
			case FALLING_TO_NORMAL:
			{
				falling_to_down();
			}break;
			case NORMAL:
			{
				normal_mode();
			}break;
			
			default:
			{
				
			}break;
		}
		
		t_mode = DWT_GetDeltaT(&prof_cnt);

		LESO_Service(); /* every loop: early-returns when offline/falling, so the NORMAL entry edge re-seeds */
		t_leso = DWT_GetDeltaT(&prof_cnt);

		VMC_translate();//VMC逆解
		Chassis_CanTransimit();//电机命令控制
		t_can = DWT_GetDeltaT(&prof_cnt);

		t_sum = t_fb + t_mode + t_leso + t_can;
		}

		/* 补拍守卫：卸力分支里 Chassis_CanTransimit 五个调用都传 Chassis_Time，
		   一拍要 5 ms，而 PreviousWakeTime 每拍只加 1 tick，于是越落越多。
		   切回 STANDING 时 vTaskDelayUntil 发现唤醒点早已过去就不再阻塞，
		   环会以 286 us 空转补课，CAN 发送频率飙到 1 kHz 的 3.5 倍把总线打爆。
		   落后超过一拍就直接对齐到当前时刻，不补课。 */
		if ((int32_t)(osKernelSysTick() - PreviousWakeTime) > (int32_t)Chassis_Time)
		{
			PreviousWakeTime = osKernelSysTick();
		}
		osDelayUntil(&PreviousWakeTime, Chassis_Time);
    }
}

//static void Compensate_PID_Init(void)
//{
//	const static float leglenth_right_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
//	const static float leglenth_left_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
//	const static float falling_leg_left_pid[3] = {FALLING_LEG_PID_KP, FALLING_LEG_PID_KI,FALLING_LEG_PID_MAX_OUT};
//	const static float falling_leg_right_pid[3] = {FALLING_LEG_PID_KP, FALLING_LEG_PID_KI,FALLING_LEG_PID_MAX_OUT};
// 	const static float phi0_left_pid[3] = {PHI0_PID_KP, PHI0_PID_KI,PHI0_PID_MAX_OUT};
// 	const static float phi0_right_pid[3] = {PHI0_PID_KP, PHI0_PID_KI,PHI0_PID_MAX_OUT};
// 
/* 任务运行周期 */



//	PID_init(&Phi0_Left_Pid, PID_POSITION,phi0_left_pid, PHI0_PID_MAX_OUT, PHI0_PID_MAX_IOUT);//phi0 pid
//	PID_init(&Phi0_Right_Pid, PID_POSITION,phi0_right_pid, PHI0_PID_MAX_OUT, PHI0_PID_MAX_IOUT);//phi0 pid
//}



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
//	Compensate_PID_Init();
	Chassis.chassis_enable = OFFLINE;
	Chassis.chassis_mode = NORMAL;
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


	Chassis.body_state.Estimate_dx   = ( Chassis.leg_situation[LEFT_Leg].stator_s + Chassis.leg_situation[LEFT_Leg].swing_s
											+ Chassis.leg_situation[RIGHT_Leg].stator_s + Chassis.leg_situation[RIGHT_Leg].swing_s )/2.0f; //估计的车体相对于地面速度
											
	Chassis.body_state.Estimate_dyaw   = (Chassis.leg_situation[RIGHT_Leg].wheel_s - Chassis.leg_situation[LEFT_Leg].wheel_s)/L_wheel; //估计的车体YAW速度
	
	Chassis.body_state.Estimate_h    = 0.5f*(Chassis.leg_situation[LEFT_Leg].vmc.L0 + Chassis.leg_situation[RIGHT_Leg].vmc.L0);  //估计的车体高度

	if(fabs(Chassis.set_goal.v_set) >= 0.05f|| Chassis.chassis_enable == OFFLINE)        // 遥控有速度指令（在动）
	{
   	 	Chassis.chassis_flag.start_situate_flag = 0;
    	Chassis.body_state.x = 0;                      // 运动中：位置清零，不积分

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
	if((fabsf(Chassis.set_goal.yaw_set_v) > 0.1f) || Chassis.chassis_enable == OFFLINE)
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
    /* 标记本周期输出来自自起；读取后清除，避免带入下一周期。 */
    uint8_t recovery_output = recovery_output_pending;
    recovery_output_pending = 0U;

    /* 直接清零最终输出，避免无效雅可比与零相乘得到 NaN。 */
    if (Chassis.chassis_enable == OFFLINE || Chassis.chassis_mode == ZERO_FORCE)
    {
        chassis_zero_outputs();
        return;
    }
    VMC_calc_2(&Chassis.leg_situation[LEFT_Leg].vmc);
    VMC_calc_2(&Chassis.leg_situation[RIGHT_Leg].vmc);

    /* 刚交回 NORMAL 时，本周期仍是自起输出，也要检查映射结果。 */
    if (recovery_output || Chassis.chassis_mode == FALLING_DOWN ||
        Chassis.chassis_mode == FALLING_TO_NORMAL)
    {
        uint8_t leg;
        for (leg = 0U; leg < 2U; ++leg)
        {
            vmc_leg_t *vmc = &Chassis.leg_situation[leg].vmc;
            if (!isfinite(vmc->torque_set[0]) || !isfinite(vmc->torque_set[1]))
            {
                chassis_recovery_abort();
                return;
            }
        }
    }
}

void Chassis_CanTransimit(void)
{
	if(Chassis.chassis_enable == ONLINE)
	{
		
		Chassis.Wheel_Motor[LEFT_Wheel].Data.SET_Current = Chassis.Wheel_Motor[LEFT_Wheel].wheel_T*LeftWheelT_TO_Current;
		Chassis.Wheel_Motor[RIGHT_Wheel].Data.SET_Current = Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T*RightWheelT_TO_Current;
		VAL_LIMIT(Chassis.Wheel_Motor[LEFT_Wheel].Data.SET_Current,-16384,16384);
		VAL_LIMIT(Chassis.Wheel_Motor[RIGHT_Wheel].Data.SET_Current,-16384,16384);

		DJI_Motor_ctrl(Chassis.Wheel_Motor,&LEFT_Wheel_CAN_hfdcan,0);
		
		mySaturate(&Chassis.leg_situation[LEFT_Leg].vmc.torque_set[0], J8009_T_MIN, J8009_T_MAX);
		mySaturate(&Chassis.leg_situation[LEFT_Leg].vmc.torque_set[1], J8009_T_MIN, J8009_T_MAX);
		mySaturate(&Chassis.leg_situation[RIGHT_Leg].vmc.torque_set[0], J8009_T_MIN, J8009_T_MAX);
		mySaturate(&Chassis.leg_situation[RIGHT_Leg].vmc.torque_set[1], J8009_T_MIN, J8009_T_MAX);
		DM_Motor_Ctrl(&LEFT_Joint_Motor_CAN_hfdcan,&Chassis.Joint_Motor[LEFT_FRONT_id],0,0,0,0,Chassis.leg_situation[LEFT_Leg].vmc.torque_set[1],0);
		DM_Motor_Ctrl(&LEFT_Joint_Motor_CAN_hfdcan,&Chassis.Joint_Motor[LEFT_BACK_id],0,0,0,0,Chassis.leg_situation[LEFT_Leg].vmc.torque_set[0],0);
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

	}else
	{
		Chassis.Wheel_Motor[LEFT_Wheel].Data.SET_Current = 0;
		Chassis.Wheel_Motor[RIGHT_Wheel].Data.SET_Current = 0;
		DJI_Motor_ctrl(Chassis.Wheel_Motor,&LEFT_Wheel_CAN_hfdcan,0);
		DM_Motor_Ctrl(&LEFT_Joint_Motor_CAN_hfdcan,&Chassis.Joint_Motor[LEFT_FRONT_id],0,0,0,0,0,0);
		DM_Motor_Ctrl(&LEFT_Joint_Motor_CAN_hfdcan,&Chassis.Joint_Motor[LEFT_BACK_id],0,0,0,0,0,0);
		DM_Motor_Ctrl(&RIGHT_Joint_Motor_CAN_hfdcan,&Chassis.Joint_Motor[RIGHT_FRONT_id],0,0,0,0,0,0);
		DM_Motor_Ctrl(&RIGHT_Joint_Motor_CAN_hfdcan,&Chassis.Joint_Motor[RIGHT_BACK_id],0,0,0,0,0,0);
	}
}

void LEG_Lenth_Control(void)
{
	/* 腿长环改为对 (L0, d_L0) 的显式 PD，形式与下面的 roll 环一致。
	   d_L0 由 VMC_calc_1 用雅可比和关节角速度解析算出，不是对 L0 做差分。
	   set_L0 是档位常数，所以目标速率为 0。
	   注意 LEG_PID_KD_RATE 与原 LEG_PID_KD 量纲不同，换算见 chassis_task.h。 */
	float left_lenth_err   = Chassis.set_goal.set_L0_Left  - Chassis.leg_situation[LEFT_Leg].vmc.L0;
	float right_lenth_err  = Chassis.set_goal.set_L0_Right - Chassis.leg_situation[RIGHT_Leg].vmc.L0;
	float left_lenth_rate  = 0.0f - Chassis.leg_situation[LEFT_Leg].vmc.d_L0;
	float right_lenth_rate = 0.0f - Chassis.leg_situation[RIGHT_Leg].vmc.d_L0;
	float F_leg_L = LEG_PID_KP * left_lenth_err  + LEG_PID_KD_RATE * left_lenth_rate;
	float F_leg_R = LEG_PID_KP * right_lenth_err + LEG_PID_KD_RATE * right_lenth_rate;
	
	float roll_err  = Chassis.set_goal.roll_set - Chassis.body_state.roll;           /* rad，EKF 横滚角 */
	float roll_rate = Chassis.set_goal.roll_set_v - Chassis.body_state.d_roll;        /* rad/s，横滚角速度轴号/符号需实测确认 */


	mySaturate(&F_leg_L,-LEG_PID_MAX_OUT,LEG_PID_MAX_OUT);
	mySaturate(&F_leg_R,-LEG_PID_MAX_OUT,LEG_PID_MAX_OUT);

	F_roll    = ROLL_PID_KP * roll_err + ROLL_PID_KD * roll_rate;
	mySaturate(&F_roll,-ROLL_PID_MAX_OUT,ROLL_PID_MAX_OUT);

	Chassis.leg_situation[LEFT_Leg].vmc.F0  = (body_mg/2.0f)*arm_cos_f32(Chassis.leg_situation[LEFT_Leg].vmc.theta)  + F_leg_L + F_roll;
	Chassis.leg_situation[RIGHT_Leg].vmc.F0 = (body_mg/2.0f)*arm_cos_f32(Chassis.leg_situation[RIGHT_Leg].vmc.theta) + F_leg_R - F_roll;
}

void falling_down_detect(void)
{
    static Chassis_Enable_e last_enable = OFFLINE;
    float phi0_L = Chassis.leg_situation[LEFT_Leg].vmc.phi0;
    float phi0_R = Chassis.leg_situation[RIGHT_Leg].vmc.phi0;

    /* OFF 到 ON 时复位自起阶段，再根据当前姿态选择自起或平衡。 */
    if (Chassis.chassis_enable == ONLINE && last_enable == OFFLINE)
    {
        last_enable = ONLINE;
        chassis_recovery_reset();
        if (phi0_L < 0.4f || phi0_L > 2.5f ||
            phi0_R < 0.4f || phi0_R > 2.5f ||
            fabs(Chassis.body_state.theta) > 0.3f)
        {
            Chassis.chassis_mode = FALLING_DOWN;
        }
        else
        {
            Chassis.chassis_mode = NORMAL;
        }
        return;
    }
    last_enable = Chassis.chassis_enable;

    /* 正常运行时持续检测；倒地后卸力，等待再次 OFF 到 ON。 */
    if (Chassis.chassis_enable == OFFLINE || Chassis.chassis_mode != NORMAL)
    {
        return;
    }
    if (phi0_L < 0.4f || phi0_L > 2.5f ||
        phi0_R < 0.4f || phi0_R > 2.5f ||
        fabs(Chassis.body_state.theta) > 0.3f)
    {
        Chassis.chassis_mode = ZERO_FORCE;
    }
}

/* 腿部虚拟力、最终关节力矩和轮输出一起清零，不再经过 VMC 映射。 */
static void chassis_zero_outputs(void)
{
    uint8_t leg;

    for (leg = 0U; leg < 2U; ++leg)
    {
        vmc_leg_t *vmc = &Chassis.leg_situation[leg].vmc;
        vmc->F0 = 0.0f;
        vmc->Tp = 0.0f;
        vmc->torque_set[0] = 0.0f;
        vmc->torque_set[1] = 0.0f;
    }
    Chassis.Wheel_Motor[LEFT_Wheel].wheel_T = 0.0f;
    Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T = 0.0f;
    Chassis.Wheel_Motor[LEFT_Wheel].Data.SET_Current = 0;
    Chassis.Wheel_Motor[RIGHT_Wheel].Data.SET_Current = 0;
}

static void chassis_recovery_reset(void)
{
    recovery_started = 0U;
    recovery_output_pending = 0U;
    recovery_phase[LEFT_Leg] = RECOVERY_SWING;
    recovery_phase[RIGHT_Leg] = RECOVERY_SWING;
    /* 保留单腿上下文供故障后观察，下次启动时重新初始化。 */
}

static void chassis_recovery_abort(void)
{
    Chassis.chassis_mode = ZERO_FORCE;
    chassis_zero_outputs();
    chassis_recovery_reset();
}

static uint8_t chassis_recovery_at_stance(float phi0)
{
    float spin;
    float error;
    uint8_t early;

    if (!isfinite(phi0))
    {
        return 0U;
    }
    spin = remainderf(phi0 - LEG_MOTION_PI * 0.5f,
                       2.0f * LEG_MOTION_PI);
    /* 零角度和整圈对应相同姿态。 */
    if (fabsf(spin) < 0.000001f)
    {
        spin = 0.0f;
    }
    if (SPIN_SWEEP_DIR == LEG_MOTION_POSITIVE)
    {
        early = (spin >= 0.0f && spin <= SPIN_EARLY_ANGLE);
    }
    else
    {
        early = (spin <= 0.0f && spin >= -SPIN_EARLY_ANGLE);
    }
    error = remainderf(SPIN_TARGET_ANGLE_DEG * LEG_MOTION_DEG_TO_RAD - spin,
                        2.0f * LEG_MOTION_PI);
    return early || fabsf(error) < SPIN_ANGLE_WINDOW;
}

/* 单腿按扫腿、收腿、保持推进；反馈和输出在这里直接接入动作模块。 */
static LegMotion_Result falling_leg_control(uint8_t leg, uint8_t start, uint32_t now)
{
    vmc_leg_t *vmc = &Chassis.leg_situation[leg].vmc;
    LegMotion_Command command;
    const LegMotion_Command *active_command = &command;
    LegMotion_Feedback feedback;
    LegMotion_Output output;
    LegMotion_Result result;

    /* 实际到位才首次进入收腿，并捕获这一刻的角度和腿长。 */
    if (recovery_phase[leg] == RECOVERY_SWING &&
        chassis_recovery_at_stance(vmc->phi0))
    {
        recovery_phase[leg] = RECOVERY_RETRACT;
        start = 1U;
    }

    command.direction = SPIN_SWEEP_DIR;
    /* 收腿只使用剩余时间，不延长从自起入口开始的总截止时间。 */
    command.timeout_ms = SPIN_SCAN_TIMEOUT_MS - (uint32_t)(now - recovery_entry_tick);
    switch (recovery_phase[leg])
    {
        case RECOVERY_SWING:
        {
            command.angle_mode = LEG_MOTION_ABSOLUTE;
            command.angle_deg = SPIN_TARGET_ANGLE_DEG;
            command.duration_ms = SPIN_RAMP_TIME_MS;
            command.length_m = SPIN_SCAN_LENGTH;
        }break;
        case RECOVERY_RETRACT:
        {
            /* 相对转角为零：固定进入收腿时的角度，只改变腿长。 */
            command.angle_mode = LEG_MOTION_RELATIVE;
            command.angle_deg = 0.0f;
            command.duration_ms = SPIN_RETRACT_TIME_MS;
            command.length_m = SPIN_RETRACT_LENGTH;
        }break;
        case RECOVERY_HOLD:
        {
            /* 已完成的动作继续保持，不重新启动，也不覆盖锁存目标。 */
            active_command = NULL;
            start = 0U;
        }break;
    }

    feedback.phi0_rad = vmc->phi0;
    feedback.angular_velocity_rad_s = vmc->d_phi0;
    feedback.length_m = vmc->L0;
    feedback.length_velocity_m_s = vmc->d_L0;
    result = LegMotion_Run(&chassis_leg_motion[leg], active_command,
                           &feedback, now, start, &output);

    recovery_output_pending = 1U;
    vmc->F0 = output.F0;
    vmc->Tp = output.Tp;
    if (recovery_phase[leg] == RECOVERY_RETRACT && result == LEG_MOTION_DONE)
    {
        recovery_phase[leg] = RECOVERY_HOLD;
    }
    return result;
}

static uint8_t chassis_recovery_handoff_ready(void)
{
    uint8_t leg;

    if (!isfinite(Chassis.body_state.theta) ||
        fabsf(Chassis.body_state.theta) >= 0.2f)
    {
        return 0U;
    }
    for (leg = 0U; leg < 2U; ++leg)
    {
        vmc_leg_t *vmc = &Chassis.leg_situation[leg].vmc;

        if (!chassis_recovery_at_stance(vmc->phi0) ||
            vmc->phi0 < 0.4f || vmc->phi0 > 2.5f ||
            fabsf(SPIN_RETRACT_LENGTH - vmc->L0) >= SPIN_LENGTH_TOLERANCE ||
            fabsf(vmc->d_L0) >= 0.05f)
        {
            return 0U;
        }
    }
    return 1U;
}

void zero_force(void)
{
    chassis_zero_outputs();
    if (Chassis.Joint_Motor[LEFT_FRONT_id].Data.state != 1 ||
        Chassis.Joint_Motor[RIGHT_FRONT_id].Data.state != 1 ||
        Chassis.Joint_Motor[RIGHT_BACK_id].Data.state != 1 ||
        Chassis.Joint_Motor[LEFT_BACK_id].Data.state != 1)
    {
        DM_Enable();
        osDelay(1);
    }
    chassis_recovery_reset();
}

void normal_mode(void)
{
    LQR();
    LEG_Lenth_Control();
}

void falling_down(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed;
    uint8_t first_cycle = 0U;
    LegMotion_Result result;

    if (Chassis.chassis_enable == OFFLINE)
    {
        chassis_zero_outputs();
        chassis_recovery_reset();
        return;
    }
    Chassis.Wheel_Motor[LEFT_Wheel].wheel_T = 0.0f;
    Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T = 0.0f;

    /* 首次进入自起，复位两腿阶段并开始总计时。 */
    if (recovery_started == 0U)
    {
        chassis_recovery_reset();
        recovery_started = 1U;
        recovery_entry_tick = now;
        first_cycle = 1U;
    }
    elapsed = (uint32_t)(now - recovery_entry_tick);
    if (elapsed >= SPIN_SCAN_TIMEOUT_MS)
    {
        chassis_recovery_abort();
        return;
    }

    /* 左腿自起；失败时立即清零，不继续执行右腿。 */
    result = falling_leg_control(LEFT_Leg, first_cycle, now);
    if (result == LEG_MOTION_TIMEOUT || result == LEG_MOTION_INVALID)
    {
        chassis_recovery_abort();
        return;
    }

    /* 右腿自起；失败时同时清除本周期左腿已经算出的输出。 */
    result = falling_leg_control(RIGHT_Leg, first_cycle, now);
    if (result == LEG_MOTION_TIMEOUT || result == LEG_MOTION_INVALID)
    {
        chassis_recovery_abort();
        return;
    }

    /* 两腿都收腿完成后，进入独立计时的平衡交接阶段。 */
    if (recovery_phase[LEFT_Leg] == RECOVERY_HOLD &&
        recovery_phase[RIGHT_Leg] == RECOVERY_HOLD)
    {
        recovery_handoff_tick = now;
        Chassis.chassis_mode = FALLING_TO_NORMAL;
    }
}

void falling_to_down(void)
{
    uint32_t now = HAL_GetTick();
    LegMotion_Result result;

    if (Chassis.chassis_enable == OFFLINE)
    {
        chassis_zero_outputs();
        chassis_recovery_reset();
        return;
    }
    Chassis.Wheel_Motor[LEFT_Wheel].wheel_T = 0.0f;
    Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T = 0.0f;

    /* 左腿保持已经完成的目标。 */
    result = falling_leg_control(LEFT_Leg, 0U, now);
    if (result != LEG_MOTION_DONE)
    {
        chassis_recovery_abort();
        return;
    }

    /* 右腿保持已经完成的目标。 */
    result = falling_leg_control(RIGHT_Leg, 0U, now);
    if (result != LEG_MOTION_DONE)
    {
        chassis_recovery_abort();
        return;
    }

    /* 姿态和腿长满足交接条件才恢复平衡，否则等待到交接超时。 */
    if (chassis_recovery_handoff_ready())
    {
        Chassis.chassis_mode = NORMAL;
    }
    else if ((uint32_t)(now - recovery_handoff_tick) >= SPIN_HANDOFF_TIMEOUT_MS)
    {
        chassis_recovery_abort();
    }
}
