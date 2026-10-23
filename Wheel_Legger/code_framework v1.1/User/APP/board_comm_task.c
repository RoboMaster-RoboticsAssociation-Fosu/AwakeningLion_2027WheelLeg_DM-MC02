#include "board_comm_task.h"
#include "user_sys_config.h"

#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
#define BOARD_COMM_REQUIRED_MASK 0x03U

/* 仅保存在线判定信息；遥控数据直接写入当前选中的 remote_ctrl。 */
static uint32_t board_rx_tick[2];
static uint8_t board_received_mask;
static uint8_t board_rc_lost = 1U;
static uint8_t board_was_online;
volatile uint8_t board_remote_online_cnt;  /* 只供 Watch 观察，不是帧序号。 */

static int16_t BoardComm_ReadInt16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}
#endif

/* 调用方在快照临界区或 CAN3 中断内读取，必需帧分别检查 50 ms。 */
bool BoardComm_IsOnline(uint32_t now)
{
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    return board_received_mask == BOARD_COMM_REQUIRED_MASK && board_rc_lost == 0U &&
           (uint32_t)(now - board_rx_tick[0]) < REMOTE_TIMEOUT_MS &&
           (uint32_t)(now - board_rx_tick[1]) < REMOTE_TIMEOUT_MS;
#else
    (void)now;
    return false;
#endif
}

void BoardComm_OnCanRx(const FDCAN_RxHeaderTypeDef *header, const uint8_t *data)
{
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    uint32_t now;
    uint8_t online;

    if (header == NULL || data == NULL ||
        header->IdType != FDCAN_STANDARD_ID ||
        header->RxFrameType != FDCAN_DATA_FRAME ||
        header->FDFormat != FDCAN_CLASSIC_CAN ||
        header->DataLength != FDCAN_DLC_BYTES_8 ||
        (header->Identifier != GIM_TO_CHA_TXID1 &&
         header->Identifier != GIM_TO_CHA_TXID2 &&
         header->Identifier != GIM_TO_CHA_TXID3))
    {
        return;
    }
    now = HAL_GetTick();
    /* 先检查旧帧年龄，防止刚恢复的帧掩盖两次遥控任务之间的超时。 */
    if (board_was_online != 0U && !BoardComm_IsOnline(now))
    {
        ++remote_loss_seq;
        board_was_online = 0U;
    }

    switch (header->Identifier)
    {
        case GIM_TO_CHA_TXID1:
            /* 云台已减去 1024；按大端有符号值直接接收。 */
            remote_ctrl.rc.ch0 = BoardComm_ReadInt16(&data[0]);
            remote_ctrl.rc.ch1 = BoardComm_ReadInt16(&data[2]);
            remote_ctrl.rc.ch2 = BoardComm_ReadInt16(&data[4]);
            remote_ctrl.rc.ch3 = BoardComm_ReadInt16(&data[6]);
            board_rx_tick[0] = now;
            board_received_mask |= 0x01U;
            break;
        case GIM_TO_CHA_TXID2:
            remote_ctrl.rc.iw = BoardComm_ReadInt16(&data[0]);
            remote_ctrl.rc.last_s1 = remote_ctrl.rc.s1;
            remote_ctrl.rc.last_s2 = remote_ctrl.rc.s2;
            remote_ctrl.rc.s1 = data[2];
            remote_ctrl.rc.s2 = data[3];
            remote_ctrl.key.v = (uint16_t)(((uint16_t)data[4] << 8) | data[5]);
            board_rc_lost = data[6];
            board_remote_online_cnt = data[7];
            board_rx_tick[1] = now;
            remote_last_rx_tick = now;  /* 使能操作随 0x61 到达，用于启动诊断。 */
            board_received_mask |= 0x02U;
            break;
        case GIM_TO_CHA_TXID3:
            remote_ctrl.mouse.x = BoardComm_ReadInt16(&data[0]);
            remote_ctrl.mouse.y = BoardComm_ReadInt16(&data[2]);
            remote_ctrl.mouse.z = BoardComm_ReadInt16(&data[4]);
            remote_ctrl.mouse.press_l = data[6];
            remote_ctrl.mouse.press_r = data[7];
            break;
    }

    online = (uint8_t)BoardComm_IsOnline(now);
    /* 短暂 rc_lost=1 后立即恢复也保留事件，不能由后一帧清除。 */
    if (board_was_online != 0U && online == 0U)
    {
        ++remote_loss_seq;
    }
    board_was_online = online;
#else
    /* LOCAL 时 CAN3 仍可接收，但不改遥控值、时间和失联序号。 */
    (void)header;
    (void)data;
#endif
}
