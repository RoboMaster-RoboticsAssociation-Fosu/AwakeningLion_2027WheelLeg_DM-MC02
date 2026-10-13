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
#include "chassis_recovery.h"
#include <math.h>
#include <stddef.h>


#define Chassis_Time						1		

#define mirror -1.0f

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
static void chassis_recovery_control(void);
static void chassis_leg_length_reference_update(void);
static void chassis_motor_online_update(void);


PidTypedef LegLenth_Left_Pid;
PidTypedef LegLenth_Right_Pid;



float F_roll;

float fb_dt;

/* Independent balance references; inspect .out in the debugger. */
static ramp_function_source_t leg_length_ramp[2];
static uint8_t leg_length_ramp_initialized;

/* Written only by the CAN RX callbacks; no feedback means offline. */
static volatile uint32_t motor_feedback_tick[CHASSIS_MOTOR_COUNT];
static volatile uint8_t motor_feedback_seen[CHASSIS_MOTOR_COUNT];

/* ---- 周期分段计时（临时插桩，用于定位 fb_dt = 6 ms 的去向）----
   t_sum 是任务自身的执行时间。若 t_sum 远小于 fb_dt，说明是被高优先级
   任务抢占，而不是自己慢。定位完可整段删除。 */
float t_fb, t_mode, t_leso, t_can, t_sum;

/* 自起阶段和两腿动作统一保存，可在调试器中观察。 */
ChassisRecovery_Context chassis_recovery;

/* Startup event trace; values stay latched until the next received OFF-to-ON. */
volatile uint32_t recovery_dbg_rx_on_tick;
volatile uint32_t recovery_dbg_rx_on_seq;
volatile uint32_t recovery_dbg_task_on_tick;
volatile uint32_t recovery_dbg_task_on_seq;
volatile uint32_t recovery_dbg_enable_tick;
volatile uint32_t recovery_dbg_first_call_tick;
volatile uint32_t recovery_dbg_first_call_delay_ms = UINT32_MAX;
volatile uint32_t recovery_dbg_reference_delay_ms = UINT32_MAX;
volatile uint32_t recovery_dbg_first_call_seq;
volatile uint32_t recovery_dbg_first_result = UINT32_MAX;
volatile uint32_t recovery_dbg_first_phase[2] = {UINT32_MAX, UINT32_MAX};
volatile float recovery_dbg_first_phi0[2];
volatile uint32_t recovery_dbg_abort_tick = UINT32_MAX;
volatile uint32_t recovery_dbg_abort_count;

static uint8_t recovery_output_pending;

extern INS_t INS;
Chassis_Info_Typedef Chassis = {
    .set_goal = {
        .set_L0_Left = 0.15f,
        .set_L0_Right = 0.15f,
    },
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
        /* OFFLINE 调试观测允许保留历史；在线自起/零力仍复位检测。 */
        if (Chassis.chassis_mode != NORMAL &&
            !(Chassis.chassis_enable == OFFLINE && ground_offline_debug_enable))
        {
            GroundContact_Reset();
        }
        chassis_leg_length_reference_update();
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
	Chassis.chassis_mode = ZERO_FORCE;
	Chassis.set_goal.roll_set = 0.0f;
}

void chassis_feedback_update(void)
{
    DM_Motor_Data_Typedef joint_fb[4];
    INS_t ins_fb;
    float torque_fb[2][2];
    float wheel_velocity[2];
    float detect_dt;
    uint8_t joint;
    uint8_t was_airborne = Chassis.ground.control_enabled && Chassis.ground.all_off_flag;
    uint32_t primask;
	static uint32_t fb_dwt_cnt = 0;
    fb_dt = DWT_GetDeltaT(&fb_dwt_cnt);
    detect_dt = fb_dt; /* 检测保留原始周期，异常时不借用控制回退值累计确认。 */
    /* 在线检查在检测前刷新；发送端仍独立检查一次。 */
    chassis_motor_online_update();
    primask = __get_PRIMASK();
    __disable_irq();
    ins_fb = INS;
    for (joint = 0U; joint < 4U; ++joint) joint_fb[joint] = Chassis.Joint_Motor[joint].Data;
    wheel_velocity[LEFT_Wheel] = Chassis.Wheel_Motor[LEFT_Wheel].Data.Velocity;
    wheel_velocity[RIGHT_Wheel] = Chassis.Wheel_Motor[RIGHT_Wheel].Data.Velocity;
    __set_PRIMASK(primask);
    /* 快照以外的几何计算不关中断，避免延长 CAN/IMU 的响应时间。 */
	if (!isfinite(fb_dt) || fb_dt > 0.05f || fb_dt <= 0.0f)
		fb_dt = Chassis_Time * 0.001f;          /* 首拍/异常时回退到名义周期 */
	Chassis.leg_situation[LEFT_Leg].last_wheel_s =   Chassis.leg_situation[LEFT_Leg].wheel_s;
	Chassis.leg_situation[LEFT_Leg].wheel_s      = -(wheel_velocity[LEFT_Wheel]/(Gear_Ratio*60.0f))*2.0f*PI*wheel_R;
	Chassis.leg_situation[RIGHT_Leg].last_wheel_s =   Chassis.leg_situation[RIGHT_Leg].wheel_s;
	Chassis.leg_situation[RIGHT_Leg].wheel_s      = ( wheel_velocity[RIGHT_Wheel]/(Gear_Ratio*60.0f))*2.0f*PI*wheel_R;

	Chassis.body_state.yaw     =  ins_fb.Yaw;
	Chassis.body_state.d_yaw   =  ins_fb.Gyro[2];
	Chassis.body_state.theta   =  ins_fb.Pitch;
	Chassis.body_state.d_theta =  -ins_fb.Gyro[1];
	Chassis.body_state.roll = ins_fb.Roll;
	Chassis.body_state.d_roll = -ins_fb.Gyro[0];

	Chassis.leg_situation[LEFT_Leg].vmc.phi4 =pi/2 +  joint_fb[LEFT_FRONT_id].pos;
	Chassis.leg_situation[LEFT_Leg].vmc.phi1 = pi/2 + joint_fb[LEFT_BACK_id].pos;
	Chassis.leg_situation[RIGHT_Leg].vmc.phi4 =pi/2 + mirror*joint_fb[RIGHT_FRONT_id].pos;
	Chassis.leg_situation[RIGHT_Leg].vmc.phi1 = pi/2 + mirror*joint_fb[RIGHT_BACK_id].pos;
	Chassis.leg_situation[LEFT_Leg].vmc.d_phi1 = joint_fb[LEFT_BACK_id].vel;
	Chassis.leg_situation[LEFT_Leg].vmc.d_phi4 = joint_fb[LEFT_FRONT_id].vel;
	Chassis.leg_situation[RIGHT_Leg].vmc.d_phi1 = mirror*joint_fb[RIGHT_BACK_id].vel;
	Chassis.leg_situation[RIGHT_Leg].vmc.d_phi4 = mirror*joint_fb[RIGHT_FRONT_id].vel;

	VMC_calc_1(&Chassis.leg_situation[LEFT_Leg].vmc,&ins_fb,fb_dt);
	VMC_calc_1(&Chassis.leg_situation[RIGHT_Leg].vmc,&ins_fb,fb_dt);
	
	Chassis.leg_situation[LEFT_Leg].stator_s = Chassis.leg_situation[LEFT_Leg].wheel_s + Chassis.leg_situation[LEFT_Leg].vmc.d_theta*wheel_R; ///未打滑的理想情况下,定子相对于地面的速度
	Chassis.leg_situation[RIGHT_Leg].stator_s = Chassis.leg_situation[RIGHT_Leg].wheel_s + Chassis.leg_situation[RIGHT_Leg].vmc.d_theta*wheel_R; ///未打滑的理想情况下,定子相对于地面的速度
	Chassis.leg_situation[LEFT_Leg].swing_s  = Chassis.leg_situation[LEFT_Leg].vmc.d_L0*arm_sin_f32(Chassis.leg_situation[LEFT_Leg].vmc.theta) + Chassis.leg_situation[LEFT_Leg].vmc.L0*(Chassis.leg_situation[LEFT_Leg].vmc.d_theta)*arm_cos_f32(Chassis.leg_situation[LEFT_Leg].vmc.theta); ////摆杆相对于机体的速度
	Chassis.leg_situation[RIGHT_Leg].swing_s  = Chassis.leg_situation[RIGHT_Leg].vmc.d_L0*arm_sin_f32(Chassis.leg_situation[RIGHT_Leg].vmc.theta) + Chassis.leg_situation[RIGHT_Leg].vmc.L0*(Chassis.leg_situation[RIGHT_Leg].vmc.d_theta)*arm_cos_f32(Chassis.leg_situation[RIGHT_Leg].vmc.theta);


	Chassis.body_state.Estimate_dx   = ( Chassis.leg_situation[LEFT_Leg].stator_s + Chassis.leg_situation[LEFT_Leg].swing_s
											+ Chassis.leg_situation[RIGHT_Leg].stator_s + Chassis.leg_situation[RIGHT_Leg].swing_s )/2.0f; //估计的车体相对于地面速度
											
	Chassis.body_state.Estimate_dyaw   = (Chassis.leg_situation[RIGHT_Leg].wheel_s - Chassis.leg_situation[LEFT_Leg].wheel_s)/L_wheel; //估计的车体YAW速度
	
	Chassis.body_state.Estimate_h    = 0.5f*(Chassis.leg_situation[LEFT_Leg].vmc.L0 + Chassis.leg_situation[RIGHT_Leg].vmc.L0);  //估计的车体高度

    /* 数组第二维是后/前关节；原始右腿力矩交给检测模块统一镜像。 */
    torque_fb[LEFT_Leg][0] = joint_fb[LEFT_BACK_id].state == 1U ? joint_fb[LEFT_BACK_id].tor : NAN;
    torque_fb[LEFT_Leg][1] = joint_fb[LEFT_FRONT_id].state == 1U ? joint_fb[LEFT_FRONT_id].tor : NAN;
    torque_fb[RIGHT_Leg][0] = joint_fb[RIGHT_BACK_id].state == 1U ? joint_fb[RIGHT_BACK_id].tor : NAN;
    torque_fb[RIGHT_Leg][1] = joint_fb[RIGHT_FRONT_id].state == 1U ? joint_fb[RIGHT_FRONT_id].tor : NAN;
    GroundContact_Update(detect_dt, ins_fb.MotionAccel_n[2], (const float (*)[2])torque_fb);
    if (was_airborne && !Chassis.ground.all_off_flag)
    {
        /* 位置/航向反馈恢复前对齐保持目标，避免带着空中积累的误差落地。 */
        Chassis.set_goal.yaw_set = ins_fb.Yaw;
    }
    if ((Chassis.ground.control_enabled && Chassis.ground.all_off_flag) || was_airborne)
    {
        Chassis.chassis_flag.start_situate_flag = 0U;
        Chassis.body_state.x = 0.0f;
    }
    else if(fabs(Chassis.set_goal.v_set) >= 0.05f|| Chassis.chassis_enable == OFFLINE)        // 遥控有速度指令（在动）
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

void Chassis_MotorFeedbackReceived(Chassis_MotorFeedback_e motor)
{
    if ((unsigned int)motor >= CHASSIS_MOTOR_COUNT)
    {
        return;
    }
    motor_feedback_tick[motor] = HAL_GetTick();
    motor_feedback_seen[motor] = 1U;
}

static void chassis_joint_enable_check(void)
{
    static uint8_t enable_requested;
    /* IDs follow the first four motor feedback slots. */
    const uint8_t joint_id[4] = {LEFT_FRONT_id, LEFT_BACK_id, RIGHT_FRONT_id, RIGHT_BACK_id};
    uint32_t primask = __get_PRIMASK();
    uint32_t now;
    uint8_t joint;
    uint8_t any_disabled = 0U;
    uint8_t all_enabled = 1U;

    __disable_irq();
    now = HAL_GetTick();
    for (joint = 0U; joint < 4U; ++joint)
    {
        if (motor_feedback_seen[joint] == 0U ||
            (uint32_t)(now - motor_feedback_tick[joint]) >= CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS)
        {
            all_enabled = 0U;
        }
        else if (Chassis.Joint_Motor[joint_id[joint]].Data.state != 1U)
        {
            any_disabled = 1U;
            all_enabled = 0U;
        }
    }
    __set_PRIMASK(primask);

    if (all_enabled != 0U)
    {
        enable_requested = 0U;
    }
    if (any_disabled != 0U && enable_requested == 0U)
    {
        enable_requested = 1U;
        DM_Enable();
    }
}

static void chassis_motor_online_update(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t now;
    uint8_t motor;
    uint8_t all_online = 1U;

    /* Sample time and feedback together so RX cannot race the timeout check. */
    __disable_irq();
    now = HAL_GetTick();
    for (motor = 0U; motor < CHASSIS_MOTOR_COUNT; ++motor)
    {
        if (motor_feedback_seen[motor] == 0U ||
            (uint32_t)(now - motor_feedback_tick[motor]) >= CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS)
        {
            all_online = 0U;
            break;
        }
    }
    __set_PRIMASK(primask);
    Chassis.all_motors_online = all_online;
}

void Chassis_CanTransimit(void)
{
    chassis_joint_enable_check();
    chassis_motor_online_update();
	if(Chassis.chassis_enable == ONLINE && Chassis.all_motors_online)
	{
        /* 发送前最终检查；轮下标 LEFT=1、RIGHT=0，与腿下标相反。 */
        if (Chassis.chassis_mode == NORMAL && Chassis.ground.control_enabled && Chassis.ground.all_off_flag)
        {
            Chassis.Wheel_Motor[LEFT_Wheel].wheel_T = 0.0f;
            Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T = 0.0f;
        }
        /* 先限幅轮力矩，再转 int16_t 电流，避免越界后反向。 */
		mySaturate(&Chassis.Wheel_Motor[LEFT_Wheel].wheel_T,-4.8f,4.8f);
		mySaturate(&Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T,-4.8f,4.8f);
//		mySaturate(&Chassis.Wheel_Motor[LEFT_Wheel].wheel_T,-0.1f,0.1f);
//		mySaturate(&Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T,-0.1f,0.1f);

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

static void chassis_leg_length_reference_update(void)
{
    float dt = fb_dt;
    uint8_t leg;

    if (Chassis.chassis_enable != ONLINE || Chassis.chassis_mode != NORMAL)
    {
        leg_length_ramp_initialized = 0U;
        return;
    }

    if (!isfinite(dt) || dt <= 0.0f || dt > 0.05f)
    {
        dt = Chassis_Time * 0.001f;
    }

    if (leg_length_ramp_initialized == 0U)
    {
        for (leg = 0U; leg < 2U; ++leg)
        {
            float length = Chassis.leg_situation[leg].vmc.L0;
            /* Include entry feedback, especially recovery's 0.139 m stance. */
            ramp_init(&leg_length_ramp[leg], dt,
                      fmaxf(0.25f, length), fminf(0.15f, length));
            leg_length_ramp[leg].input = LEG_LENGTH_RAMP_RATE_M_S;
            leg_length_ramp[leg].out = length;
        }
        leg_length_ramp_initialized = 1U;
        return;
    }

    leg_length_ramp[LEFT_Leg].frame_period = dt;
    leg_length_ramp[RIGHT_Leg].frame_period = dt;
    ramp_calc(&leg_length_ramp[LEFT_Leg], Chassis.set_goal.set_L0_Left);
    ramp_calc(&leg_length_ramp[RIGHT_Leg], Chassis.set_goal.set_L0_Right);
}

void LEG_Lenth_Control(void)
{
    /* Track the ramped position; keep measured-velocity damping. */
	float left_lenth_err   = leg_length_ramp[LEFT_Leg].out  - Chassis.leg_situation[LEFT_Leg].vmc.L0;
	float right_lenth_err  = leg_length_ramp[RIGHT_Leg].out - Chassis.leg_situation[RIGHT_Leg].vmc.L0;
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

    if (Chassis.ground.control_enabled && Chassis.ground.all_off_flag)
    {
        /* 无地面反力时只保持腿长 PD；车体重力前馈和 Roll 不能在空中纠正姿态。 */
        F_roll = 0.0f;
        Chassis.leg_situation[LEFT_Leg].vmc.F0 = F_leg_L;
        Chassis.leg_situation[RIGHT_Leg].vmc.F0 = F_leg_R;
    }
    else
    {
        Chassis.leg_situation[LEFT_Leg].vmc.F0 = (body_mg/2.0f)*arm_cos_f32(Chassis.leg_situation[LEFT_Leg].vmc.theta) + F_leg_L + F_roll;
        Chassis.leg_situation[RIGHT_Leg].vmc.F0 = (body_mg/2.0f)*arm_cos_f32(Chassis.leg_situation[RIGHT_Leg].vmc.theta) + F_leg_R - F_roll;
    }
}

void falling_down_detect(void)
{
    static Chassis_Enable_e last_enable = OFFLINE;
    float phi0_L = Chassis.leg_situation[LEFT_Leg].vmc.phi0;
    float phi0_R = Chassis.leg_situation[RIGHT_Leg].vmc.phi0;

    /* OFF 到 ON 时统一复位并进入自起，由模块选择收腿或扫腿。 */
    if (Chassis.chassis_enable == ONLINE && last_enable == OFFLINE)
    {
        last_enable = ONLINE;
        recovery_dbg_enable_tick = HAL_GetTick();
        chassis_recovery_reset();
        /* Every enable edge completes recovery before NORMAL takes over. */
        Chassis.chassis_mode = FALLING_DOWN;
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
    recovery_output_pending = 0U;
    ChassisRecovery_Reset(&chassis_recovery);
}

static void chassis_recovery_abort(void)
{
    recovery_dbg_abort_tick = HAL_GetTick();
    ++recovery_dbg_abort_count;
    Chassis.chassis_mode = ZERO_FORCE;
    chassis_zero_outputs();
    chassis_recovery_reset();
}

void zero_force(void)
{
    chassis_zero_outputs();
    chassis_recovery_reset();
}

void normal_mode(void)
{
    LQR();
    LEG_Lenth_Control();
}

/* 底盘只负责传入反馈、回写输出和切换模式。 */
static void chassis_recovery_control(void)
{
    ChassisRecovery_Input input;
    ChassisRecovery_Output output;
    ChassisRecovery_Result result;
    uint32_t recovery_dbg_now;
    uint32_t recovery_dbg_seq;
    uint8_t recovery_dbg_capture;

    input.enabled = (Chassis.chassis_enable == ONLINE);
    input.pitch_rad = Chassis.body_state.theta;
    input.leg[CHASSIS_RECOVERY_LEFT].phi0_rad = Chassis.leg_situation[LEFT_Leg].vmc.phi0;
    input.leg[CHASSIS_RECOVERY_LEFT].angular_velocity_rad_s = Chassis.leg_situation[LEFT_Leg].vmc.d_phi0;
    input.leg[CHASSIS_RECOVERY_LEFT].length_m = Chassis.leg_situation[LEFT_Leg].vmc.L0;
    input.leg[CHASSIS_RECOVERY_LEFT].length_velocity_m_s = Chassis.leg_situation[LEFT_Leg].vmc.d_L0;
    input.leg[CHASSIS_RECOVERY_RIGHT].phi0_rad = Chassis.leg_situation[RIGHT_Leg].vmc.phi0;
    input.leg[CHASSIS_RECOVERY_RIGHT].angular_velocity_rad_s = Chassis.leg_situation[RIGHT_Leg].vmc.d_phi0;
    input.leg[CHASSIS_RECOVERY_RIGHT].length_m = Chassis.leg_situation[RIGHT_Leg].vmc.L0;
    input.leg[CHASSIS_RECOVERY_RIGHT].length_velocity_m_s = Chassis.leg_situation[RIGHT_Leg].vmc.d_L0;

    recovery_dbg_now = HAL_GetTick();
    recovery_dbg_seq = recovery_dbg_rx_on_seq;
    recovery_dbg_capture = input.enabled != 0U && recovery_dbg_first_call_seq != recovery_dbg_seq;
    if (recovery_dbg_capture != 0U)
    {
        recovery_dbg_first_call_tick = recovery_dbg_now;
        recovery_dbg_first_phi0[0] = input.leg[0].phi0_rad;
        recovery_dbg_first_phi0[1] = input.leg[1].phi0_rad;
    }
    result = ChassisRecovery_Run(&chassis_recovery, &input, recovery_dbg_now, &output);
    if (recovery_dbg_capture != 0U)
    {
        recovery_dbg_first_call_delay_ms = recovery_dbg_now - recovery_dbg_rx_on_tick;
        recovery_dbg_first_result = (uint32_t)result;
        recovery_dbg_first_phase[0] = (uint32_t)chassis_recovery.phase[0];
        recovery_dbg_first_phase[1] = (uint32_t)chassis_recovery.phase[1];
        recovery_dbg_first_call_seq = recovery_dbg_seq;
    }
    if (input.enabled != 0U && result == CHASSIS_RECOVERY_RUNNING &&
        recovery_dbg_reference_delay_ms == UINT32_MAX &&
        recovery_dbg_rx_on_seq != 0U &&
        recovery_dbg_first_call_seq == recovery_dbg_rx_on_seq &&
        chassis_recovery.phase[0] == CHASSIS_RECOVERY_RETRACT &&
        chassis_recovery.phase[1] == CHASSIS_RECOVERY_RETRACT &&
        chassis_recovery.motion[0].reference_length_m < chassis_recovery.motion[0].start_length_m - 0.002f &&
        chassis_recovery.motion[1].reference_length_m < chassis_recovery.motion[1].start_length_m - 0.002f)
    {
        recovery_dbg_reference_delay_ms = recovery_dbg_now - recovery_dbg_rx_on_tick;
    }
    switch (result)
    {
        case CHASSIS_RECOVERY_IDLE:
        {
            chassis_zero_outputs();
            recovery_output_pending = 0U;
            return;
        }
        case CHASSIS_RECOVERY_RUNNING:
        {
            Chassis.chassis_mode = FALLING_DOWN;
        }break;
        case CHASSIS_RECOVERY_HANDOFF:
        {
            Chassis.chassis_mode = FALLING_TO_NORMAL;
        }break;
        case CHASSIS_RECOVERY_DONE:
        {
            Chassis.chassis_mode = NORMAL;
        }break;
        case CHASSIS_RECOVERY_FAULT:
        default:
        {
            chassis_recovery_abort();
            return;
        }
    }

    Chassis.leg_situation[LEFT_Leg].vmc.F0 = output.leg[CHASSIS_RECOVERY_LEFT].F0;
    Chassis.leg_situation[LEFT_Leg].vmc.Tp = output.leg[CHASSIS_RECOVERY_LEFT].Tp;
    Chassis.leg_situation[RIGHT_Leg].vmc.F0 = output.leg[CHASSIS_RECOVERY_RIGHT].F0;
    Chassis.leg_situation[RIGHT_Leg].vmc.Tp = output.leg[CHASSIS_RECOVERY_RIGHT].Tp;
    Chassis.Wheel_Motor[LEFT_Wheel].wheel_T = 0.0f;
    Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T = 0.0f;
    /* 即使本拍已经切到 NORMAL，VMC 仍需检查这次自起输出。 */
    recovery_output_pending = 1U;
}

void falling_down(void)
{
    chassis_recovery_control();
}

void falling_to_down(void)
{
    chassis_recovery_control();
}
