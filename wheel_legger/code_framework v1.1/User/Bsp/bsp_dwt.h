/**
 ******************************************************************************
 * @file    bsp_dwt.h
 * @version V1.1.0
 * @date    2026.03.04
 * @brief   DWT内核精确延时函数声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef _BSP_DWT_H
#define _BSP_DWT_H

/* Includes ----------------------------------------------------------------- */
#include "main.h"
#include "stdint.h"

/* Defines ------------------------------------------------------------------ */
// 在 bsp_dwt.h 里加
#define DWT_Delay_us(us)    DWT_Delay((float)(us) * 0.000001f)
/* Enums -------------------------------------------------------------------- */

/* Structs ------------------------------------------------------------------ */
/**
 * @brief DWT时间结构体
 */
typedef struct
{
    uint32_t s;		/*!< 秒 */
    uint16_t ms;	/*!< 毫秒 */
    uint16_t us;	/*!< 微秒 */
} DWT_Time_t;

/* Externs ------------------------------------------------------------------ */
extern DWT_Time_t SysTime;

/* Functions ---------------------------------------------------------------- */
void DWT_Init(uint32_t CPU_Freq_mHz);
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(float Delay);
void DWT_SysTimeUpdate(void);

/* -------------------------------------------------------------------------- */
#endif /* _BSP_DWT_H */
