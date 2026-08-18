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
        ！！！！|！！！！ch2+						！！！！|！！！！ch0+
			|									|
			|									|
			|									|	
			ch3+								ch1+

*/


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
		
		if(remote_ctrl.rc.s2 == 2)
		{
			Chassis.chassis_mode = offline;

			Chassis.set_goal.v_set = 0;
			Chassis.set_goal.yaw_set_v = 0;
		}else
		{
			Chassis.chassis_mode = online;

			Chassis.set_goal.v_set = -remote_ctrl.rc.ch3/RC_RESOLUTION*4.0f;
			Chassis.set_goal.yaw_set_v = remote_ctrl.rc.ch0/RC_RESOLUTION*0.1f;
		}
    }
}
