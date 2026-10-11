#ifndef __BOARD_COMM_TASK_H
#define __BOARD_COMM_TASK_H

#include "Remote_Control.h"

#define GIM_TO_CHA_TXID1 0x51U
#define GIM_TO_CHA_TXID2 0x61U
#define GIM_TO_CHA_TXID3 0x71U
#define BOARD_COMM_TASK_PERIOD_MS 1U

typedef struct
{
    RC_Ctl_t remote;
    uint32_t rx_tick[2];
    uint32_t loss_seq;
    uint8_t received_mask;
    uint8_t rc_lost;
    uint8_t online_cnt;
} BoardComm_Remote_t;

/* CAN and UART never share their receive storage. */
extern volatile BoardComm_Remote_t board_remote_ctrl;
extern volatile uint32_t board_comm_rx_overruns;

/* Interrupt entry: validate and cache only, without writing chassis commands. */
void BoardComm_OnCanRx(const FDCAN_RxHeaderTypeDef *header, const uint8_t *data);
/* One task iteration, also used by the host verification harness. */
void BoardComm_Update(void);
bool BoardComm_GetRemoteSnapshot(BoardComm_Remote_t *snapshot);
void board_comm_task(void);

#endif
