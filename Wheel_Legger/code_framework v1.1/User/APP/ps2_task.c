#include "ps2_task.h"
#include "chassis_task.h"

#define PS2_TASK_PERIOD_MS  30

//static void PS2_Ctrl_Chassis(PS2_Info_Typedef *PS2, Chassis_Info_Typedef *Chassis);

void PS2_task(void)
{
    while (1)
    {
//        PS2_ScanKey(&PS2_Info);
//        PS2_Ctrl_Chassis(&PS2_Info, &Chassis);
//        osDelay(PS2_TASK_PERIOD_MS);
    }
}

//static void PS2_Ctrl_Chassis(PS2_Info_Typedef *PS2, Chassis_Info_Typedef *Chassis)
//{
//    if (PS2->PS2_buff[2] != 0x5A)
//        return;

//    /* 摇杆→速度 */
//    int16_t jx = PS2_LRC_UD(PS2);
//    int16_t jy = PS2_LRC_LR(PS2);
//    int16_t jz = PS2_RRC_LR(PS2);

//    if (PS2->Data.mode == PS2_MODE_RED)
//    {
//        float vx = (float)jx / PS2_RC_MAX * CHASSIS_MAX_V;
//        float vy = (float)jy / PS2_RC_MAX * CHASSIS_MAX_V;
//        float wz = (float)jz / PS2_RC_MAX * CHASSIS_MAX_W;
//        Chassis_Set_Velocity(Chassis, vx, vy, wz);
//    }
//    else
//    {
//        float vx = 0, vy = 0, wz = 0;
//        if (PS2->Data.PS2_UP)    vx =  CHASSIS_MAX_V;
//        if (PS2->Data.PS2_DOWN)  vx = -CHASSIS_MAX_V;
//        if (PS2->Data.PS2_LEFT)  vy =  CHASSIS_MAX_V;
//        if (PS2->Data.PS2_RIGHT) vy = -CHASSIS_MAX_V;
//        if (PS2->Data.PS2_L1)    wz = -0.5f;
//        if (PS2->Data.PS2_R1)    wz =  0.5f;
//        Chassis_Set_Velocity(Chassis, vx, vy, wz);
//    }
//}
