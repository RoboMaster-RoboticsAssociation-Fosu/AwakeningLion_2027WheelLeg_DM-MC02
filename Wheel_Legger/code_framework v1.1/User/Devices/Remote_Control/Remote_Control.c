/**
 ******************************************************************************
 * @file    Remote_Control.c
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   遥控器数据处理函数
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "Remote_Control.h"
#include "stm32h7xx.h"
#include "string.h"

/* Defines ----------------------------------------------------------------- */

/* Global variable --------------------------------------------------------- */
RC_Ctl_t remote_ctrl;

uint32_t remote_last_rx_tick = 0;

__attribute__((section (".AXI_SRAM"))) uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_LEN];

/* Static Fun -------------------------------------------------------------- */


/* Functions --------------------------------------------------------------- */
/**
 * @brief  SBUS数据解析至遥控器
 * @param  sbus_buf: SBUS数据缓冲区指针
 * @param  remote_ctrl: 遥控器信息结构体指针
 * @return 无
 * @note   无
 */
void SBUS_TO_RC(uint8_t *pData,RC_Ctl_t *RC_CtrlData)
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
	
	RC_CtrlData->rc.iw = ((int16_t)pData[16] | ((int16_t)pData[17] << 8)) & 0x07FF;
	RC_CtrlData->rc.iw -= 1024;

    remote_last_rx_tick = HAL_GetTick();
}

/**
 * @brief  遥控器离线检测
 * @param  无
 * @return true: 离线, false: 在线
 * @note   50ms 内未收到新帧即判定离线
 */
bool Remote_Is_Offline(void)
{
    return (HAL_GetTick() - remote_last_rx_tick) >= REMOTE_TIMEOUT_MS;
}

/* Private functions ------------------------------------------------------- */

/* Interrupt functions ----------------------------------------------------- */

/* ------------------------------------------------------------------------- */