/**
 ******************************************************************************
 * @file    pid.c
 * @version V1.0.0
 * @date    2026.03.05
 * @brief   PID控制器算法实现
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "pid.h"

/* Defines ----------------------------------------------------------------- */
#define LimitMax(input, max)   	 \
    {                          	 \
        if (input > max)       	 \
        {                      	 \
            input = max;       	 \
        }                      	 \
        else if (input < -max) 	 \
        {                      	 \
            input = -max;      	 \
        }                      	 \
    }

/* Global variable --------------------------------------------------------- */

/* Static Fun -------------------------------------------------------------- */

/* Functions --------------------------------------------------------------- */
/**
  * @brief  PID初始化
  * @param  pid: 指向PID结构体的指针
  * @param  mode: PID模式 (位置式/增量式)
  * @param  PID: 包含Kp, Ki, Kd的数组
  * @param  max_out: 最大输出限幅
  * @param  max_iout: 积分输出最大值
  * @return 无
  * @note   无
  */
void PID_init(PidTypedef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
	if (pid == NULL || PID == NULL)
	{
		return;
	}
	#if DEBUG
		pid->Kp = PID[0];
		pid->Ki = PID[1];
		pid->Kd = PID[2];
	#endif
	if(pid->Initlized != true)
	{
		pid->pid_mode = mode;
		pid->Kp = PID[0];
		pid->Ki = PID[1];
		pid->Kd = PID[2];
		pid->max_out = max_out;
		pid->max_iout = max_iout;
		pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
		pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
		pid->Initlized = true;
	}
}

/**
  * @brief  PID计算
  * @param  pid: 指向PID结构体的指针
  * @param  fdb: 反馈值
  * @param  ref: 目标值
  * @return fp32: PID计算输出
  * @note   无
  */
fp32 PID_Calc(PidTypedef *pid, fp32 fdb, fp32 ref)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = ref;
    pid->fdb = fdb;
    pid->error[0] = ref - fdb;
	if(pid->Initlized == true)
	{
		switch(pid->pid_mode)
		{
			case PID_POSITION:
			{
				pid->Pout = pid->Kp * pid->error[0];
				pid->Iout += pid->Ki * pid->error[0];
				pid->Dbuf[2] = pid->Dbuf[1];
				pid->Dbuf[1] = pid->Dbuf[0];
				pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
				pid->Dout = pid->Kd * pid->Dbuf[0];
				LimitMax(pid->Iout, pid->max_iout);
				pid->out = pid->Pout + pid->Iout + pid->Dout;
				LimitMax(pid->out, pid->max_out);
			}break;
			
			case PID_DELTA:
			{
				pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
				pid->Iout = pid->Ki * pid->error[0];
				pid->Dbuf[2] = pid->Dbuf[1];
				pid->Dbuf[1] = pid->Dbuf[0];
				pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
				pid->Dout = pid->Kd * pid->Dbuf[0];
				pid->out += pid->Pout + pid->Iout + pid->Dout;
				LimitMax(pid->out, pid->max_out);
			}break;
		}
	}
	else
	{
	    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
		pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
		pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
		pid->fdb = pid->set = 0.0f;	
	}
    return pid->out;
}

/**
  * @brief  清除PID内部数据
  * @param  pid: 指向PID结构体的指针
  * @return 无
  * @note   无
  */
void PID_clear(PidTypedef *pid)
{
    if (pid == NULL)
    {
        return;
    }
	pid->Initlized = false;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

/* Private functions ------------------------------------------------------- */

/* Interrupt functions ----------------------------------------------------- */

/* ------------------------------------------------------------------------- */
