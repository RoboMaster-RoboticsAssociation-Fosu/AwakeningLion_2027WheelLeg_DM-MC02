#ifndef REMOTE_SOURCE_PLATFORM_H
#define REMOTE_SOURCE_PLATFORM_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
/* Wire parsing uses explicit fields; host tests do not depend on struct packing. */
#define __packed
#define FDCAN_STANDARD_ID 0U
#define FDCAN_EXTENDED_ID 1U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_REMOTE_FRAME 1U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_FD_CAN 1U
#define FDCAN_DLC_BYTES_8 (8U << 16)
#define FDCAN_RX_FIFO0 0U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
#define RESET 0U
#define HAL_OK 0
#define HAL_ERROR 1
#define FDCAN1 ((void *)(uintptr_t)1U)
#define FDCAN3 ((void *)(uintptr_t)3U)
typedef struct { void *Instance; } FDCAN_HandleTypeDef;
typedef struct {
    uint32_t Identifier, IdType, RxFrameType, DataLength, FDFormat;
} FDCAN_RxHeaderTypeDef;
enum { OFFLINE, ONLINE };
enum { ZERO_FORCE, FALLING_DOWN, FALLING_TO_NORMAL, NORMAL };
enum { RIGHT_Wheel, LEFT_Wheel };
enum { RIGHT_FRONT_id, RIGHT_BACK_id, LEFT_BACK_id, LEFT_FRONT_id };
#define Chassis_3508_Motor1_RxID 0x201U
#define Chassis_3508_Motor2_RxID 0x202U
#define JOINT_DM_RIGHT_FRONT_RxID_Set 0x11U
#define JOINT_DM_RIGHT_BACK_RxID_Set 0x12U
typedef struct {
    int chassis_enable, chassis_mode;
    struct { float v_set, yaw_set_v, set_L0_Left, set_L0_Right; } set_goal;
    int Wheel_Motor[2], Joint_Motor[4];
} Chassis_Info_Typedef;
extern Chassis_Info_Typedef Chassis;
extern FDCAN_RxHeaderTypeDef RxHeader1, RxHeader3;
extern uint8_t g_Can1RxData[64], g_Can3RxData[64];
uint32_t HAL_GetTick(void);
uint32_t osKernelSysTick(void);
int osDelayUntil(uint32_t *tick, uint32_t period);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *handle, uint32_t fifo);
int HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *handle, uint32_t fifo, FDCAN_RxHeaderTypeDef *header, uint8_t *data);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *handle, uint32_t flags);
void DJI_Motor_Info_Update(int *motor, uint8_t *data, uint32_t size);
void DM_Motor_Info_Update(int *motor, uint8_t *data, uint32_t size);
#endif
