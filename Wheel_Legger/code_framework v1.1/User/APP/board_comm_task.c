#include "board_comm_task.h"

#define BOARD_COMM_REQUIRED_MASK 0x03U
#define BOARD_COMM_RX_QUEUE_SIZE 8U

typedef struct
{
    uint8_t data[8];
    uint32_t rx_tick;
    uint8_t slot;
} BoardComm_Frame_t;

/* One ISR producer and one task consumer; queue usable capacity is seven. */
static volatile BoardComm_Frame_t board_rx_frames[BOARD_COMM_RX_QUEUE_SIZE];
static volatile uint8_t board_rx_head;
static volatile uint8_t board_rx_tail;
static uint8_t board_link_was_valid;
static uint32_t board_rx_overruns_seen;
volatile uint32_t board_comm_rx_overruns;
volatile BoardComm_Remote_t board_remote_ctrl = { .rc_lost = 1U };

static int16_t BoardComm_ReadInt16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

static bool BoardComm_IsValid(const BoardComm_Remote_t *snapshot, uint32_t now)
{
    return snapshot->received_mask == BOARD_COMM_REQUIRED_MASK &&
           snapshot->rc_lost == 0U &&
           (uint32_t)(now - snapshot->rx_tick[0]) < REMOTE_TIMEOUT_MS &&
           (uint32_t)(now - snapshot->rx_tick[1]) < REMOTE_TIMEOUT_MS;
}

void BoardComm_OnCanRx(const FDCAN_RxHeaderTypeDef *header, const uint8_t *data)
{
    uint8_t slot;
    uint8_t head;
    uint8_t next_head;
    uint32_t i;

    if (header == NULL || data == NULL ||
        header->IdType != FDCAN_STANDARD_ID ||
        header->RxFrameType != FDCAN_DATA_FRAME ||
        header->FDFormat != FDCAN_CLASSIC_CAN ||
        header->DataLength != FDCAN_DLC_BYTES_8)
    {
        return;
    }
    switch (header->Identifier)
    {
        case GIM_TO_CHA_TXID1: slot = 0U; break;
        case GIM_TO_CHA_TXID2: slot = 1U; break;
        case GIM_TO_CHA_TXID3: slot = 2U; break;
        default: return;
    }
    head = board_rx_head;
    next_head = (uint8_t)((head + 1U) % BOARD_COMM_RX_QUEUE_SIZE);
    if (next_head == board_rx_tail)
    {
        ++board_comm_rx_overruns;
        return;
    }
    for (i = 0U; i < 8U; ++i)
    {
        board_rx_frames[head].data[i] = data[i];
    }
    board_rx_frames[head].rx_tick = HAL_GetTick();
    board_rx_frames[head].slot = slot;
    board_rx_head = next_head;
}

static void BoardComm_ApplyFrame(BoardComm_Remote_t *next, const BoardComm_Frame_t *frame)
{
    if (frame->slot == 0U)
    {
        next->remote.rc.ch0 = BoardComm_ReadInt16(&frame->data[0]);
        next->remote.rc.ch1 = BoardComm_ReadInt16(&frame->data[2]);
        next->remote.rc.ch2 = BoardComm_ReadInt16(&frame->data[4]);
        next->remote.rc.ch3 = BoardComm_ReadInt16(&frame->data[6]);
        next->rx_tick[0] = frame->rx_tick;
        next->received_mask |= 0x01U;
    }
    else if (frame->slot == 1U)
    {
        next->remote.rc.iw = BoardComm_ReadInt16(&frame->data[0]);
        next->remote.rc.last_s1 = next->remote.rc.s1;
        next->remote.rc.last_s2 = next->remote.rc.s2;
        next->remote.rc.s1 = frame->data[2];
        next->remote.rc.s2 = frame->data[3];
        next->remote.key.v = (uint16_t)(((uint16_t)frame->data[4] << 8) |
                                      (uint16_t)frame->data[5]);
        next->rc_lost = frame->data[6];
        next->online_cnt = frame->data[7];
        next->rx_tick[1] = frame->rx_tick;
        next->received_mask |= 0x02U;
    }
    else
    {
        next->remote.mouse.x = BoardComm_ReadInt16(&frame->data[0]);
        next->remote.mouse.y = BoardComm_ReadInt16(&frame->data[2]);
        next->remote.mouse.z = BoardComm_ReadInt16(&frame->data[4]);
        next->remote.mouse.press_l = frame->data[6];
        next->remote.mouse.press_r = frame->data[7];
    }
}

void BoardComm_Update(void)
{
    BoardComm_Frame_t frames[BOARD_COMM_RX_QUEUE_SIZE];
    BoardComm_Remote_t next;
    uint32_t interrupt_mask;
    uint32_t now;
    uint32_t overruns;
    uint32_t count = 0U;
    uint32_t i;
    uint8_t valid;

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    now = HAL_GetTick();
    overruns = board_comm_rx_overruns;
    while (board_rx_tail != board_rx_head)
    {
        frames[count++] = board_rx_frames[board_rx_tail];
        board_rx_tail = (uint8_t)((board_rx_tail + 1U) % BOARD_COMM_RX_QUEUE_SIZE);
    }
    next = board_remote_ctrl;
    __set_PRIMASK(interrupt_mask);

    /* Retain a timeout even if fresh frames are already waiting in the queue. */
    if (board_link_was_valid != 0U && !BoardComm_IsValid(&next, now))
    {
        ++next.loss_seq;
        board_link_was_valid = 0U;
    }
    if (overruns != board_rx_overruns_seen)
    {
        /* Dropped frames may contain a loss report; require a fresh full pair. */
        board_rx_overruns_seen = overruns;
        ++next.loss_seq;
        next.received_mask = 0U;
        next.rc_lost = 1U;
        board_link_was_valid = 0U;
    }
    else
    {
        for (i = 0U; i < count; ++i)
        {
            BoardComm_ApplyFrame(&next, &frames[i]);
            valid = (uint8_t)BoardComm_IsValid(&next, now);
            if (board_link_was_valid != 0U && valid == 0U)
            {
                ++next.loss_seq;
            }
            board_link_was_valid = valid;
        }
    }

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    board_remote_ctrl = next;
    __set_PRIMASK(interrupt_mask);
}

bool BoardComm_GetRemoteSnapshot(BoardComm_Remote_t *snapshot)
{
    uint32_t interrupt_mask;
    uint32_t now;
    if (snapshot == NULL)
    {
        return false;
    }
    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    now = HAL_GetTick();
    *snapshot = board_remote_ctrl;
    __set_PRIMASK(interrupt_mask);
    return BoardComm_IsValid(snapshot, now);
}

void board_comm_task(void)
{
    uint32_t previous_wake_time = osKernelSysTick();
    for (;;)
    {
        BoardComm_Update();
        osDelayUntil(&previous_wake_time, BOARD_COMM_TASK_PERIOD_MS);
    }
}
