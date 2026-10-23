#ifndef __BOARD_COMM_TASK_H
#define __BOARD_COMM_TASK_H

#include "Remote_Control.h"

#define GIM_TO_CHA_TXID1 0x51U
#define GIM_TO_CHA_TXID2 0x61U
#define GIM_TO_CHA_TXID3 0x71U

/* CAN3 中断直接解析选中来源；本模块不再创建任务或软件队列。 */
void BoardComm_OnCanRx(const FDCAN_RxHeaderTypeDef *header, const uint8_t *data);
/* 仅供 Remote_GetSnapshot 的短临界区及 CAN3 接收中断调用。 */
bool BoardComm_IsOnline(uint32_t now);

#endif
