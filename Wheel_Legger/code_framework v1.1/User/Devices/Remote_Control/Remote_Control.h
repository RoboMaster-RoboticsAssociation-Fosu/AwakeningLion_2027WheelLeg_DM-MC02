/**
 ******************************************************************************
 * @file    Remote_Control.h
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   遥控器数据处理函数声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 待测试
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

/* Includes ----------------------------------------------------------------- */
#include "cmsis_os.h"
#include "stdbool.h"
#include "stdlib.h"
#include "main.h"

/* Defines ------------------------------------------------------------------ */
#define SBUS_RX_BUF_LEN		18u			/* SBUS接收数据长度 */

#define REMOTE_TIMEOUT_MS  50u  /* 遥控器超时时间，单位：毫秒 */

#define RC_RESOLUTION 660


// 遥控器通道值的范围定义
#define RC_CH_VALUE_MIN     ((uint16_t)364)
#define RC_CH_VALUE_OFFSET  ((uint16_t)1024)
#define RC_CH_VALUE_MAX     ((uint16_t)1684)

// 遥控器开关状态定义
#define RC_SW_UP            ((uint8_t)1)
#define RC_SW_MID           ((uint8_t)3)
#define RC_SW_DOWN          ((uint8_t)2)

// 键盘按键位定义
#define KEY_PRESSED_OFFSET_W     ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S     ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A     ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D     ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q     ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E     ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL  ((uint16_t)0x01<<7)

/* Enums -------------------------------------------------------------------- */
/**
 * @brief 键盘状态枚举
 */

/* Structs ------------------------------------------------------------------ */
/**
 * @brief 键盘信息结构体
 */


/**
 * @brief 遥控器按键结构体
 */


/**
 * @brief 遥控器信息结构体
 */
typedef __packed struct
{
    // 遥控器通道数据
    __packed struct
    {
        int16_t ch0; // 通道0：右摇杆X轴
        int16_t ch1; // 通道1：右摇杆Y轴
        int16_t ch2; // 通道2：左摇杆X轴
        int16_t ch3; // 通道3：左摇杆Y轴
        uint8_t  s1;  // S1开关状态
        uint8_t  s2;  // S2开关状态
		int16_t iw;
		
		uint8_t  last_s1;  // S1开关状态
        uint8_t  last_s2;  // S2开关状态
    }rc;
    
    // 鼠标数据
    __packed struct
    {
        int16_t x;       // 鼠标X轴移动速度
        int16_t y;       // 鼠标Y轴移动速度
        int16_t z;       // 鼠标Z轴（滚轮）移动速度
        uint8_t press_l; // 鼠标左键是否按下
        uint8_t press_r; // 鼠标右键是否按下
    }mouse;
    
    // 键盘数据
    __packed struct
    {
        uint16_t v; // 按键状态
    }key;
}RC_Ctl_t;

/* Externs ------------------------------------------------------------------ */
extern RC_Ctl_t remote_ctrl;
extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_LEN];
extern uint32_t remote_last_rx_tick;

/* Functions ---------------------------------------------------------------- */
void SBUS_TO_RC(uint8_t *pData,RC_Ctl_t *RC_CtrlData);
bool Remote_Is_Offline(void);


/* -------------------------------------------------------------------------- */
#endif /* REMOTE_CONTROL_H */
