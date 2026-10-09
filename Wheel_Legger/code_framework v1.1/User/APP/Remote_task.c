#include "Remote_task.h"
#include "cmsis_os.h"
#include "Remote_Control.h"
#include "chassis_task.h"
//S1   S2
//	1
//	3
//	2
/*

			|									|
    ????????|????????ch2+			     ????????|????????ch0+
			|									|
			|									|
			|									|	
			ch3+								ch1+

*/
#define DT7_TASK_PERIOD_MS  30

void remote_task(void)
{
    while (1)
    {
//		if(remote_ctrl.rc.s1 != remote_ctrl.rc.last_s1)
//		{
//			remote_ctrl.rc.last_s1 = remote_ctrl.rc.s1;
//		}
//		if(remote_ctrl.rc.s2 != remote_ctrl.rc.last_s2)
//		{
//			remote_ctrl.rc.last_s2 = remote_ctrl.rc.s2;
//		}
		
		if(remote_ctrl.rc.s2 == 2 || remote_ctrl.rc.s2 == 0)
		{
			Chassis.chassis_enable = OFFLINE;

			Chassis.set_goal.v_set = 0;
			Chassis.set_goal.yaw_set_v = 0;
		}else if(remote_ctrl.rc.s2 == 1 || remote_ctrl.rc.s2 == 3)
		{
			Chassis.chassis_enable = ONLINE;

			Chassis.set_goal.v_set = (float)remote_ctrl.rc.ch3/RC_RESOLUTION*2.0f;
			Chassis.set_goal.yaw_set_v = -(float)remote_ctrl.rc.ch0/RC_RESOLUTION*3.0f;
		}
		if(Chassis.chassis_mode  == NORMAL)
			{
				if(remote_ctrl.rc.s1 == 2 || remote_ctrl.rc.s1 == 0)
				{
					Chassis.set_goal.set_L0_Left = Chassis.set_goal.set_L0_Right = 0.159f;
				}else if(remote_ctrl.rc.s1 == 1)
				{
					Chassis.set_goal.set_L0_Left = Chassis.set_goal.set_L0_Right = 0.25f;
				}else if(remote_ctrl.rc.s1 == 3)
				{
//				Chassis.set_goal.set_L0_Left = Chassis.set_goal.set_L0_Right = -(float)remote_ctrl.rc.ch1/RC_RESOLUTION*2.0f + 0.15f;
				Chassis.set_goal.set_L0_Left = Chassis.set_goal.set_L0_Right = 0.20f;
				}
			}
		osDelay(DT7_TASK_PERIOD_MS);
    }
}
