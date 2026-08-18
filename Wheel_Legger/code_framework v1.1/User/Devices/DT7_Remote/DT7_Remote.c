#include "DT7_Remote.h"
#include "bsp_dma.h"
#include "usart.h"
#include "main.h"
#include "usart.h"
#include "CHASSIS_Task.h"

extern UART_HandleTypeDef huart5;

uint8_t SBUS_MultiRx_Buf[2][RC_FRAME_LENGTH];
RC_Ctl_t RC_CtrlData;

RM_MOVE_Typedef rm;

static void chassis_operation_func(int16_t forward_back, int16_t left_right, int16_t rotate);

void DBUS_Init(void)
{
	USART_DMAEx_MultiBuffer_Init(&huart5, (uint32_t *)SBUS_MultiRx_Buf[0], (uint32_t *)SBUS_MultiRx_Buf[1],36);
}

void RemoteDataProcess(uint8_t *pData,RC_Ctl_t *RC_CtrlData)
{
    if(pData == NULL) return;
    
    // 解析4个摇杆通道，每个通道11bit
    RC_CtrlData->rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
	RC_CtrlData->rc.ch0 -= 1024;
    RC_CtrlData->rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	RC_CtrlData->rc.ch1 -= 1024;
    RC_CtrlData->rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
	RC_CtrlData->rc.ch2 -= 1024;
    RC_CtrlData->rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF;
	RC_CtrlData->rc.ch3 -= 1024;
    
    // 解析2个开关状态，每个开关2bit
    RC_CtrlData->rc.s1 = ((pData[5] >> 4) & 0x0C) >> 2;
    RC_CtrlData->rc.s2 = ((pData[5] >> 4) & 0x03);
    
    // 解析鼠标数据
    RC_CtrlData->mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    RC_CtrlData->mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    RC_CtrlData->mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
    RC_CtrlData->mouse.press_l = pData[12];
    RC_CtrlData->mouse.press_r = pData[13];
    
    // 解析键盘数据
    RC_CtrlData->key.v = pData[14];
	
	chassis_operation_func(RC_CtrlData->rc.ch3,RC_CtrlData->rc.ch2,RC_CtrlData->rc.ch0);
}

static void chassis_operation_func(int16_t forward_back, int16_t left_right, int16_t rotate)
{
	rm.vx = (float)forward_back / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X;
	rm.vy = (float)left_right / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y;
	rm.vw = (float)rotate / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_R;
}
