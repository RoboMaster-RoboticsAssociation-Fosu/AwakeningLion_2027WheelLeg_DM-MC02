#ifndef _DT7_REMOTE_H
#define _DT7_REMOTE_H

#include "main.h"

#define RC_FRAME_LENGTH 18
#define SBUS_RX_BUF_NUM 36

extern uint8_t SBUS_MultiRx_Buf[2][RC_FRAME_LENGTH];

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

// 遥控器控制数据结构体
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

typedef struct
{
	int32_t vx;
	int32_t vy;
	int32_t vw;
}RM_MOVE_Typedef;

// 全局的遥控器控制数据，外部文件可以直接读取这个变量获取控制信息
extern RC_Ctl_t RC_CtrlData;
extern RM_MOVE_Typedef rm;

extern void DBUS_Init(void);
extern void RemoteDataProcess(uint8_t *pData,RC_Ctl_t *RC_CtrlData);

#endif
