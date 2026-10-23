#include "main.h"
#include "Remote_Control.h"
#include "Remote_task.h"
#include "chassis_task.h"
#include "board_comm_task.h"
#include "user_sys_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

Chassis_Info_Typedef Chassis;
FDCAN_RxHeaderTypeDef RxHeader1, RxHeader3;
uint8_t g_Can1RxData[64], g_Can3RxData[64];
volatile uint32_t recovery_dbg_rx_on_tick, recovery_dbg_rx_on_seq;
volatile uint32_t recovery_dbg_task_on_tick, recovery_dbg_task_on_seq;
volatile uint32_t recovery_dbg_first_call_delay_ms, recovery_dbg_reference_delay_ms;
volatile uint32_t recovery_dbg_first_result, recovery_dbg_first_phase[2];
volatile float recovery_dbg_first_phi0[2];
volatile uint32_t recovery_dbg_abort_tick, recovery_dbg_abort_count;
static uint32_t now, interrupt_mask;
static unsigned int checks, failures, motor_updates;
typedef struct { FDCAN_RxHeaderTypeDef header; uint8_t data[8]; } Frame;
static Frame fifo_frames[8];
static unsigned int fifo_count, fifo_pos, fifo_reads;
static int fifo_fail;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; fprintf(stderr, "REMOTE FAIL line %d: %s\n", __LINE__, #c); } } while (0)
#define NEAR(a,b) CHECK(fabsf((a) - (b)) < 0.00001f)
uint32_t HAL_GetTick(void) { return now; }
uint32_t osKernelSysTick(void) { return now; }
int osDelayUntil(uint32_t *tick, uint32_t period) { *tick += period; return 0; }
uint32_t __get_PRIMASK(void) { return interrupt_mask; }
void __disable_irq(void) { interrupt_mask = 1U; }
void __set_PRIMASK(uint32_t mask) { interrupt_mask = mask; }
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *handle, uint32_t fifo)
{ (void)handle; (void)fifo; return fifo_count - fifo_pos; }
int HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *handle, uint32_t fifo, FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)handle; (void)fifo; ++fifo_reads;
    if (fifo_fail) { fifo_fail = 0; return HAL_ERROR; }
    if (fifo_pos == fifo_count) return HAL_ERROR;
    *header = fifo_frames[fifo_pos].header;
    memcpy(data, fifo_frames[fifo_pos++].data, 8);
    return HAL_OK;
}
void DJI_Motor_Info_Update(int *motor, uint8_t *data, uint32_t size)
{ (void)motor; (void)data; (void)size; ++motor_updates; }
void DM_Motor_Info_Update(int *motor, uint8_t *data, uint32_t size)
{ (void)motor; (void)data; (void)size; ++motor_updates; }
static FDCAN_RxHeaderTypeDef Header(uint32_t id)
{
    FDCAN_RxHeaderTypeDef h = { id, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8, FDCAN_CLASSIC_CAN };
    return h;
}
static void Put16(uint8_t *data, int16_t value)
{ uint16_t raw = (uint16_t)value; data[0] = (uint8_t)(raw >> 8); data[1] = (uint8_t)raw; }
static void Sticks(int16_t ch0, int16_t ch3)
{
    uint8_t data[8]; FDCAN_RxHeaderTypeDef h = Header(GIM_TO_CHA_TXID1);
    Put16(data, ch0); Put16(data + 2, -660); Put16(data + 4, 660); Put16(data + 6, ch3);
    BoardComm_OnCanRx(&h, data);
}
static void StatusIw(uint8_t s1, uint8_t s2, uint8_t lost, int16_t iw)
{
    uint8_t data[8] = { 0, 0, s1, s2, 0xA5, 0x5A, lost, 250 };
    FDCAN_RxHeaderTypeDef h = Header(GIM_TO_CHA_TXID2);
    Put16(data, iw); BoardComm_OnCanRx(&h, data);
}
static void Status(uint8_t s1, uint8_t s2, uint8_t lost)
{ StatusIw(s1, s2, lost, 0); }
static void Pair(int16_t ch0, int16_t ch3, uint8_t s1, uint8_t s2, uint8_t lost)
{ Sticks(ch0, ch3); Status(s1, s2, lost); }
static void Mouse(void)
{
    uint8_t data[8] = { 0x80, 0, 0x7F, 0xFF, 0xFF, 0xFF, 1, 0 };
    FDCAN_RxHeaderTypeDef h = Header(GIM_TO_CHA_TXID3);
    BoardComm_OnCanRx(&h, data);
}
static void LocalIw(int16_t ch0, int16_t ch3, uint8_t s1, uint8_t s2, int16_t iw)
{
    uint8_t data[18] = {0}; unsigned int channel, bit;
    int16_t channels[4] = { ch0, -660, 660, ch3 };
    for (channel = 0; channel < 4; ++channel) {
        uint16_t value = (uint16_t)(channels[channel] + 1024);
        for (bit = 0; bit < 11; ++bit) {
            unsigned int target = channel * 11 + bit;
            if ((value >> bit) & 1U) data[target / 8] |= (uint8_t)(1U << (target % 8));
        }
    }
    data[5] |= (uint8_t)((s1 << 6) | (s2 << 4));
    data[14] = 0x5A;
    uint16_t dial = (uint16_t)(iw + 1024);
    data[16] = (uint8_t)dial; data[17] = (uint8_t)(dial >> 8);
    SBUS_TO_RC(data, &remote_ctrl);
}
static void Local(int16_t ch0, int16_t ch3, uint8_t s1, uint8_t s2)
{ LocalIw(ch0, ch3, s1, s2, 0); }
static void BothIw(int16_t ch0, int16_t ch3, uint8_t s1, uint8_t s2, int16_t iw)
{ LocalIw(ch0, ch3, s1, s2, iw); Sticks(ch0, ch3); StatusIw(s1, s2, 0, iw); }
static void Both(int16_t ch0, int16_t ch3, uint8_t s1, uint8_t s2)
{ Local(ch0, ch3, s1, s2); Pair(ch0, ch3, s1, s2, 0); }
/* Exercise the public interface and ensure mode ownership stays in chassis. */
static void Step(void)
{
    int mode = Chassis.chassis_mode;
    int state = Chassis.normal_state;
    Remote_ControlStep();
    CHECK(Chassis.chassis_mode == mode);
    CHECK(Chassis.normal_state == state);
}
static void Online(void) { CHECK(Chassis.chassis_enable == ONLINE); }
static void Offline(void)
{ CHECK(Chassis.chassis_enable == OFFLINE); NEAR(Chassis.set_goal.v_set, 0); NEAR(Chassis.set_goal.yaw_set_v, 0); CHECK(Chassis.normal_state_request == NORMAL_MOVE); }
static void PrepareDown(void)
{
    Both(0, 0, RC_SW_UP, RC_SW_DOWN); Step(); Offline();
    CHECK(recovery_dbg_rx_on_seq == 0U);
}
static void Rearm(void)
{
    Step(); Offline(); CHECK(recovery_dbg_rx_on_seq == 0U);
    for (now = 0U; now < 3U; ++now) {
        Both(0, 0, RC_SW_MID, now == 2U ? RC_SW_UP : RC_SW_MID); Step(); Offline();
        CHECK(recovery_dbg_rx_on_seq == 0U);
    }
    Both(0, 0, RC_SW_MID, RC_SW_DOWN); Step(); Offline();
    CHECK(recovery_dbg_rx_on_seq == 0U);
    ++now; Both(0, 0, RC_SW_MID, RC_SW_MID); Step(); Online();
    CHECK(recovery_dbg_rx_on_seq == 1U && recovery_dbg_rx_on_tick == now);
    for (unsigned i = 0U; i < 10U; ++i) {
        ++now; Both(0, 0, RC_SW_MID, RC_SW_MID); Step(); Online();
        CHECK(recovery_dbg_rx_on_seq == 1U);
    }
    /* An invalid S2 must latch OFFLINE until another valid DOWN. */
    ++now; Both(0, 0, RC_SW_MID, 0U); Step(); Offline();
    ++now; Both(0, 0, RC_SW_MID, RC_SW_MID); Step(); Offline();
    CHECK(recovery_dbg_rx_on_seq == 1U);
    ++now; Both(0, 0, RC_SW_MID, RC_SW_DOWN); Step(); Offline();
    ++now; Both(0, 0, RC_SW_MID, RC_SW_UP); Step(); Online();
    CHECK(recovery_dbg_rx_on_seq == 2U);
    /* A DOWN from before the timeout is not a fresh acknowledgement. */
    ++now; Both(0, 0, RC_SW_MID, RC_SW_DOWN); Step(); Offline();
    now += REMOTE_TIMEOUT_MS; Step(); Offline();
    CHECK(recovery_dbg_rx_on_seq == 2U);
    for (unsigned i = 0U; i < 3U; ++i) {
        ++now; Both(0, 0, RC_SW_MID, RC_SW_MID); Step(); Offline();
        CHECK(recovery_dbg_rx_on_seq == 2U);
        ++now; Both(0, 0, RC_SW_MID, 0U); Step(); Offline();
        now += REMOTE_TIMEOUT_MS; Step(); Offline();
    }
    ++now; Both(0, 0, RC_SW_MID, RC_SW_DOWN); Step(); Offline();
    CHECK(recovery_dbg_rx_on_seq == 2U);
    ++now; Both(0, 0, RC_SW_MID, RC_SW_MID); Step(); Online();
    CHECK(recovery_dbg_rx_on_seq == 3U && recovery_dbg_rx_on_tick == now);
    for (unsigned i = 0U; i < 10U; ++i) {
        ++now; Both(0, 0, RC_SW_MID, RC_SW_MID); Step(); Online();
        CHECK(recovery_dbg_rx_on_seq == 3U);
    }
}
static void Protocol(void)
{
    RC_Ctl_t input;
    uint32_t stamp = 99, seq = 99;
    CHECK(!Remote_GetSnapshot(&input, &stamp, &seq)); CHECK(Remote_Is_Offline());
    CHECK(stamp == 0 && seq == 0);
    CHECK(!Remote_GetSnapshot(NULL, NULL, NULL));
    now = 12; Sticks(-330, 660);
    CHECK(!Remote_GetSnapshot(&input, &stamp, &seq));
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    CHECK(remote_ctrl.rc.ch0 == -330 && remote_ctrl.rc.ch3 == 660);
#else
    CHECK(remote_ctrl.rc.ch0 == 0 && remote_ctrl.rc.ch3 == 0);
#endif
    now = 13; StatusIw(RC_SW_UP, RC_SW_MID, 0, -321);
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    CHECK(Remote_GetSnapshot(&input, &stamp, &seq));
    CHECK(remote_ctrl.rc.iw == -321 && remote_ctrl.key.v == 0xA55A);
#else
    CHECK(!Remote_GetSnapshot(&input, &stamp, &seq));
    CHECK(remote_ctrl.rc.s2 == 0 && remote_last_rx_tick == 0 && remote_loss_seq == 0);
    LocalIw(-330, 660, RC_SW_UP, RC_SW_MID, -321);
    CHECK(Remote_GetSnapshot(&input, &stamp, &seq));
    CHECK(remote_ctrl.key.v == 0x5A);
#endif
    CHECK(input.rc.ch0 == -330 && input.rc.ch1 == -660);
    CHECK(input.rc.ch2 == 660 && input.rc.ch3 == 660);
    CHECK(input.rc.iw == -321 && input.rc.s1 == RC_SW_UP && input.rc.s2 == RC_SW_MID);
    CHECK(stamp == 13 && seq == 0 && recovery_dbg_rx_on_seq == 0);
    Mouse(); CHECK(Remote_GetSnapshot(&input, &stamp, &seq));
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    CHECK(input.mouse.x == INT16_MIN && input.mouse.y == INT16_MAX);
    CHECK(input.mouse.z == -1 && input.mouse.press_l == 1);
#else
    CHECK(input.mouse.x == 0 && input.mouse.y == 0 && input.mouse.press_l == 0);
#endif
    {
        RC_Ctl_t before = remote_ctrl;
        uint8_t data[8] = {0}; FDCAN_RxHeaderTypeDef h;
        now = 30;
        for (unsigned i = 0; i < 16; ++i) {
            if (i == 8) continue;
            h = Header(GIM_TO_CHA_TXID1); h.DataLength = i << 16; BoardComm_OnCanRx(&h, data);
            h.Identifier = GIM_TO_CHA_TXID2; BoardComm_OnCanRx(&h, data);
        }
        h = Header(GIM_TO_CHA_TXID1); h.IdType = FDCAN_EXTENDED_ID; BoardComm_OnCanRx(&h, data);
        h = Header(GIM_TO_CHA_TXID1); h.RxFrameType = FDCAN_REMOTE_FRAME; BoardComm_OnCanRx(&h, data);
        h = Header(GIM_TO_CHA_TXID1); h.FDFormat = FDCAN_FD_CAN; BoardComm_OnCanRx(&h, data);
        h = Header(0x777); BoardComm_OnCanRx(&h, data);
        BoardComm_OnCanRx(NULL, data); BoardComm_OnCanRx(&h, NULL);
        CHECK(Remote_GetSnapshot(&input, &stamp, &seq));
        CHECK(memcmp(&before, &remote_ctrl, sizeof(before)) == 0 && stamp == 13 && seq == 0);
    }
    /* The other source must not change data, freshness or loss history. */
    now = 63;
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_LOCAL
    Pair(660, -660, RC_SW_DOWN, RC_SW_DOWN, 1);
#else
    LocalIw(660, -660, RC_SW_DOWN, RC_SW_DOWN, 660);
#endif
    CHECK(!Remote_GetSnapshot(&input, &stamp, &seq));
    CHECK(stamp == 13 && seq == 0 && input.rc.ch0 == -330 && input.rc.s2 == RC_SW_MID);
}
static void Mapping(void)
{
    static const uint8_t switches[] = {RC_SW_DOWN, RC_SW_MID, RC_SW_UP, 0};
    static const float lengths[] = {0.159f, 0.20f, 0.25f, 0.159f};
    unsigned int i;
    for (i = 0; i < 4; ++i) {
        now += 2; Both(-330, 660, switches[i], RC_SW_UP); Step(); Online();
        NEAR(Chassis.set_goal.v_set, 2); NEAR(Chassis.set_goal.yaw_set_v, 3.0f);
        CHECK(Chassis.normal_state_request == NORMAL_SPIN);
        NEAR(Chassis.set_goal.set_L0_Left, lengths[i]); NEAR(Chassis.set_goal.set_L0_Right, lengths[i]);
    }
    CHECK(recovery_dbg_rx_on_seq == 1 && recovery_dbg_rx_on_tick == 2);
    CHECK(recovery_dbg_task_on_seq == 1 && recovery_dbg_first_result == UINT32_MAX);
    recovery_dbg_first_call_delay_ms = 123;
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_LOCAL
    Pair(660, -660, RC_SW_DOWN, RC_SW_DOWN, 1);
#else
    Local(660, -660, RC_SW_DOWN, RC_SW_DOWN);
#endif
    Step(); Online(); NEAR(Chassis.set_goal.v_set, 2);
    CHECK(recovery_dbg_rx_on_seq == 1 && recovery_dbg_first_call_delay_ms == 123);
    now += 1; Both(660, -660, RC_SW_MID, RC_SW_MID); Step(); Online();
    NEAR(Chassis.set_goal.v_set, -2); NEAR(Chassis.set_goal.yaw_set_v, -3);
    Chassis.chassis_mode = FALLING_DOWN; Chassis.set_goal.set_L0_Left = 0.139f;
    Chassis.set_goal.set_L0_Right = 0.139f;
    Both(0, 0, RC_SW_UP, RC_SW_MID); Step();
    NEAR(Chassis.set_goal.set_L0_Left, 0.139f); NEAR(Chassis.set_goal.set_L0_Right, 0.139f);
    now += 1; Both(0, 0, RC_SW_UP, RC_SW_DOWN); Step(); Offline();
    now += 1; Both(0, 0, RC_SW_UP, 0); Step(); Offline();
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    Pair(0, 0, RC_SW_UP, 7, 0); Step(); Offline();
#endif
}
static void Timeout(void)
{
    Both(330, -660, RC_SW_UP, RC_SW_UP); Step(); Online();
    now = 49; Mouse(); Step(); Online();
    now = 50; Step(); Offline(); CHECK(Remote_Is_Offline());
    now = 51; Both(330, -660, RC_SW_UP, RC_SW_UP); Step(); Offline();
    CHECK(recovery_dbg_rx_on_seq == 1U);
    now = 52; Both(0, 0, RC_SW_UP, 0); Step(); Offline();
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    CHECK(remote_rearm_required == 1);
    now = 53; Pair(0, 0, RC_SW_UP, RC_SW_DOWN, 1); Step(); Offline();
    CHECK(remote_rearm_required == 1);
#endif
    now = 54; Both(0, 0, RC_SW_UP, RC_SW_DOWN); Step(); Offline();
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    CHECK(remote_rearm_required == 0);
#endif
    now = 55; Both(330, -660, RC_SW_UP, RC_SW_UP); Step(); Online();
    CHECK(recovery_dbg_rx_on_tick == 55);
    CHECK(recovery_dbg_rx_on_seq == 2 && recovery_dbg_task_on_seq == 2);
}
static void Missing(int sticks_missing)
{
    RC_Ctl_t input;
    Both(0, 660, RC_SW_UP, RC_SW_UP); Step(); Online();
    now = 49;
    if (sticks_missing) Status(RC_SW_UP, RC_SW_UP, 0); else Sticks(0, 660);
    CHECK(Remote_GetSnapshot(&input, NULL, NULL));
    now = 50; CHECK(!Remote_GetSnapshot(&input, NULL, NULL)); Step(); Offline();
    now = 51; Both(0, 660, RC_SW_UP, RC_SW_UP); Step(); Offline();
    CHECK(recovery_dbg_rx_on_seq == 1U && remote_rearm_required == 1);
}
static void Lost(void)
{
    RC_Ctl_t input; uint32_t seq;
    Both(0, 660, RC_SW_UP, RC_SW_UP); Step(); Online();
    now = 10; Pair(0, 660, RC_SW_UP, RC_SW_UP, 1);
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    CHECK(!Remote_GetSnapshot(&input, NULL, &seq)); CHECK(seq == 1);
#else
    CHECK(Remote_GetSnapshot(&input, NULL, &seq)); CHECK(seq == 0);
#endif
    /* No control step between loss and recovery: the event must survive. */
    now = 11; Pair(0, 660, RC_SW_UP, RC_SW_UP, 0); Step();
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    Offline(); CHECK(remote_rearm_required == 1);
    now = 12; Pair(0, 0, RC_SW_DOWN, RC_SW_DOWN, 0); Step(); Offline();
    now = 13; Pair(0, 660, RC_SW_UP, RC_SW_UP, 0); Step(); Online();
    CHECK(recovery_dbg_rx_on_seq == 2);
#else
    Online(); CHECK(recovery_dbg_rx_on_seq == 1);
#endif
}
static void Burst(void)
{
    RC_Ctl_t input; uint32_t seq;
    Both(0, 660, RC_SW_UP, RC_SW_UP); Step(); Online();
    now = 1; Status(RC_SW_UP, RC_SW_UP, 1);
    now = 2; Status(RC_SW_UP, RC_SW_UP, 0);
    CHECK(Remote_GetSnapshot(&input, NULL, &seq));
    Step();
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    CHECK(seq == 1); Offline(); CHECK(remote_rearm_required == 1);
#else
    CHECK(seq == 0); Online();
#endif
}
static void Direct(void)
{
    RC_Ctl_t input; uint32_t stamp, seq;
    Both(0, 660, RC_SW_UP, RC_SW_UP); Step(); Online();
    now = 1;
    for (unsigned i = 0; i < 100; ++i) {
        Sticks((int16_t)i, (int16_t)-i);
        StatusIw(RC_SW_UP, RC_SW_UP, i == 20U || i == 70U, (int16_t)i);
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
        CHECK(remote_ctrl.rc.ch0 == (int16_t)i && remote_ctrl.rc.iw == (int16_t)i);
#endif
    }
    CHECK(Remote_GetSnapshot(&input, &stamp, &seq));
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    CHECK(stamp == 1 && seq == 2 && input.rc.ch3 == -99);
    Step(); Offline(); CHECK(remote_rearm_required == 1);
    /* Fresh traffic cannot erase either loss event. */
    now = 2; Pair(0, 660, RC_SW_UP, RC_SW_UP, 0); Step(); Offline();
    now = 3; Pair(0, 0, RC_SW_UP, RC_SW_DOWN, 0); Step(); Offline();
    now = 4; Pair(0, 660, RC_SW_UP, RC_SW_UP, 0); Step(); Online();
    CHECK(recovery_dbg_rx_on_seq == 2);
#else
    CHECK(stamp == 0 && seq == 0 && input.rc.ch3 == 660);
    Step(); Online(); CHECK(recovery_dbg_rx_on_seq == 1);
#endif
}
static void StartupStatus(void)
{
    RC_Ctl_t input;
    Status(RC_SW_UP, RC_SW_UP, 0);
    CHECK(!Remote_GetSnapshot(&input, NULL, NULL)); Step(); Offline();
    now = 50; Sticks(0, 660);
    CHECK(!Remote_GetSnapshot(&input, NULL, NULL)); Step(); Offline();
    now = 51; Status(RC_SW_UP, RC_SW_UP, 0);
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    CHECK(Remote_GetSnapshot(&input, NULL, NULL));
#else
    CHECK(!Remote_GetSnapshot(&input, NULL, NULL));
    Local(0, 660, RC_SW_UP, RC_SW_UP);
#endif
    Step(); Offline(); CHECK(recovery_dbg_rx_on_seq == 0U && remote_rearm_required == 1);
    now = 52; Both(0, 0, RC_SW_UP, RC_SW_DOWN); Step(); Offline();
    now = 53; Both(0, 660, RC_SW_UP, RC_SW_UP); Step(); Online();
    CHECK(recovery_dbg_rx_on_seq == 1U);
}
static void Wrap(void)
{
    now = UINT32_MAX - 20U; PrepareDown();
    Both(0, 660, RC_SW_UP, RC_SW_UP); Step(); Online();
    now = 28; Step(); Online(); CHECK(!Remote_Is_Offline());
    now = 29; Step(); Offline(); CHECK(Remote_Is_Offline());
    /* Recover across wrap without losing the expired interval. */
    Both(0, 660, RC_SW_UP, RC_SW_UP); Step(); Offline();
    CHECK(remote_loss_seq == 1 && remote_rearm_required == 1);
    ++now; Both(0, 0, RC_SW_UP, RC_SW_DOWN); Step(); Offline();
    ++now; Both(0, 660, RC_SW_UP, RC_SW_UP); Step(); Online();
    CHECK(recovery_dbg_rx_on_seq == 2);
}
/* Fresh recovery before the next task still retains expired mandatory-frame gaps. */
static void Gaps(void)
{
    RC_Ctl_t input; uint32_t seq;
    Both(0, 660, RC_SW_UP, RC_SW_UP); Step(); Online();
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    now = 49; Status(RC_SW_UP, RC_SW_UP, 0);
    now = 50; Sticks(123, 456);
#else
    now = 50; Local(123, 456, RC_SW_UP, RC_SW_UP);
#endif
    CHECK(Remote_GetSnapshot(&input, NULL, &seq));
    CHECK(seq == 1 && input.rc.ch0 == 123 && input.rc.ch3 == 456);
    Step(); Offline(); CHECK(remote_rearm_required == 1);
    ++now; Both(0, 0, RC_SW_UP, RC_SW_DOWN); Step(); Offline();
    ++now; Both(0, 660, RC_SW_UP, RC_SW_UP); Step(); Online();
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    now += 49; Sticks(0, 660);
    ++now; Status(RC_SW_UP, RC_SW_UP, 0);
#else
    now += 50; Local(0, 660, RC_SW_UP, RC_SW_UP);
#endif
    CHECK(Remote_GetSnapshot(&input, NULL, &seq)); CHECK(seq == 2);
    Step(); Offline(); CHECK(recovery_dbg_rx_on_seq == 2);
}
static void Fifo(void)
{
    FDCAN_HandleTypeDef can3 = {FDCAN3}, can1 = {FDCAN1};
    RC_Ctl_t input; uint32_t seq;
    fifo_frames[0].header = Header(GIM_TO_CHA_TXID1);
    fifo_frames[0].header.DataLength = 7U << 16;
    fifo_frames[1].header = Header(GIM_TO_CHA_TXID1); Put16(fifo_frames[1].data + 6, 660);
    fifo_frames[2].header = Header(GIM_TO_CHA_TXID2);
    fifo_frames[2].data[2] = RC_SW_UP; fifo_frames[2].data[3] = RC_SW_UP;
    fifo_count = 3; Chassis.set_goal.v_set = 7;
    HAL_FDCAN_RxFifo0Callback(&can3, 0); CHECK(fifo_reads == 0);
    fifo_fail = 1; HAL_FDCAN_RxFifo0Callback(&can3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    CHECK(fifo_reads == 1 && fifo_pos == 0);
    HAL_FDCAN_RxFifo0Callback(&can3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    CHECK(fifo_reads == 4 && fifo_pos == 3); NEAR(Chassis.set_goal.v_set, 7);
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    /* These values must be visible immediately on return from the actual callback. */
    CHECK(remote_ctrl.rc.ch3 == 660 && remote_ctrl.rc.s2 == RC_SW_UP);
    CHECK(Remote_GetSnapshot(&input, NULL, &seq));
#else
    CHECK(remote_ctrl.rc.ch3 == 0 && remote_ctrl.rc.s2 == 0);
    CHECK(!Remote_GetSnapshot(&input, NULL, &seq));
#endif
    Step(); Offline(); CHECK(recovery_dbg_rx_on_seq == 0U);
    Both(0, 660, RC_SW_UP, RC_SW_DOWN); Step(); Offline();
    Both(0, 660, RC_SW_UP, RC_SW_UP); Step(); Online();
    NEAR(Chassis.set_goal.v_set, 2);
    /* Two status frames in the same hardware FIFO retain the transient loss. */
    fifo_pos = 0; fifo_count = 2;
    for (unsigned i = 0; i < 2; ++i) {
        memset(&fifo_frames[i], 0, sizeof(fifo_frames[i]));
        fifo_frames[i].header = Header(GIM_TO_CHA_TXID2);
        fifo_frames[i].data[2] = RC_SW_UP; fifo_frames[i].data[3] = RC_SW_UP;
        fifo_frames[i].data[6] = i == 0;
    }
    HAL_FDCAN_RxFifo0Callback(&can3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    CHECK(fifo_pos == 2 && Remote_GetSnapshot(&input, NULL, &seq));
    Step();
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    CHECK(seq == 1 && remote_ctrl.rc.s2 == RC_SW_UP); Offline();
#else
    CHECK(seq == 0); Online();
#endif
    fifo_pos = 0; fifo_count = 1; fifo_frames[0].header = Header(0x201);
    HAL_FDCAN_RxFifo0Callback(&can1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    CHECK(motor_updates == 1);
}
static void Stream(void)
{
    for (now = 0; now < 500; ++now) {
        Both(-660, 660, RC_SW_MID, RC_SW_MID); Step(); Online();
        CHECK(remote_rearm_required == 0);
        NEAR(Chassis.set_goal.yaw_set_v, 3);
    }
    CHECK(recovery_dbg_rx_on_seq == 1);
    /* No task step during the 50 ms gap: the receive layer must retain it. */
    now = 549; Both(0, 660, RC_SW_UP, RC_SW_UP); Step();
    Offline(); CHECK(remote_rearm_required == 1 && remote_loss_seq == 1);
    CHECK(recovery_dbg_rx_on_seq == 1);
    ++now; Both(0, 0, RC_SW_UP, RC_SW_DOWN); Step(); Offline();
    ++now; Both(0, 660, RC_SW_UP, RC_SW_UP); Step(); Online();
    CHECK(recovery_dbg_rx_on_seq == 2);
}
static void Snapshot(void)
{
    RC_Ctl_t input; uint32_t stamp = 99, seq = 99;
    CHECK(!Remote_GetSnapshot(&input, &stamp, &seq)); CHECK(stamp == 0 && seq == 0);
    CHECK(!Remote_GetSnapshot(NULL, &stamp, &seq)); SBUS_TO_RC(NULL, &remote_ctrl);
    { uint8_t data[18] = {0}; SBUS_TO_RC(data, NULL); }
    CHECK(Remote_Is_Offline()); now = 4; Both(-123, 456, RC_SW_MID, RC_SW_UP);
    interrupt_mask = 1;
    CHECK(Remote_GetSnapshot(&input, &stamp, &seq)); CHECK(interrupt_mask == 1);
    CHECK(stamp == 4 && seq == 0 && input.rc.ch0 == -123 && input.rc.ch3 == 456);
    CHECK(Remote_GetSnapshot(&input, NULL, NULL)); CHECK(interrupt_mask == 1);
    interrupt_mask = 0;
    CHECK(Remote_GetSnapshot(&input, &stamp, &seq)); CHECK(interrupt_mask == 0);
    Step(); Offline(); CHECK(recovery_dbg_rx_on_seq == 0U);
    Both(-123, 456, RC_SW_MID, RC_SW_DOWN); Step(); Offline();
    Both(-123, 456, RC_SW_MID, RC_SW_UP); Step(); Online();
    CHECK(recovery_dbg_rx_on_tick == 4);
    now = 54; Both(-321, 654, RC_SW_MID, RC_SW_UP);
    CHECK(Remote_GetSnapshot(&input, &stamp, &seq));
    CHECK(stamp == 54 && seq == 1 && input.rc.ch0 == -321 && input.rc.ch3 == 654);
    Step(); Offline(); CHECK(remote_rearm_required == 1);
}
static void LegLength(void)
{
    static const int modes[] = {ZERO_FORCE, FALLING_DOWN, FALLING_TO_NORMAL, NORMAL};
    unsigned int i;
    for (i = 0; i < sizeof(modes) / sizeof(modes[0]); ++i) {
        ++now; Chassis.chassis_mode = modes[i];
        Chassis.set_goal.set_L0_Left = 0.18f; Chassis.set_goal.set_L0_Right = 0.22f;
        Both(0, 0, RC_SW_UP, RC_SW_UP); Step(); Online();
        if (modes[i] == NORMAL) {
            NEAR(Chassis.set_goal.set_L0_Left, LEG_LENGTH_LONG_M); NEAR(Chassis.set_goal.set_L0_Right, LEG_LENGTH_LONG_M);
        } else {
            NEAR(Chassis.set_goal.set_L0_Left, 0.18f); NEAR(Chassis.set_goal.set_L0_Right, 0.22f);
        }
    }
    ++now; Both(0, 0, 7, RC_SW_UP);
    /* DBUS switches are only two bits; inject a corrupt field defensively. */
    remote_ctrl.rc.s1 = 7;
    Chassis.set_goal.set_L0_Left = 0.18f; Chassis.set_goal.set_L0_Right = 0.22f;
    Step(); Online();
    NEAR(Chassis.set_goal.set_L0_Left, 0.18f); NEAR(Chassis.set_goal.set_L0_Right, 0.22f);
    ++now; Both(0, 0, RC_SW_DOWN, RC_SW_DOWN); Step(); Offline();
    /* A valid NORMAL sample still selects leg length while disabled. */
    NEAR(Chassis.set_goal.set_L0_Left, LEG_LENGTH_SHORT_M); NEAR(Chassis.set_goal.set_L0_Right, LEG_LENGTH_SHORT_M);
    ++now; Chassis.chassis_mode = FALLING_DOWN; Both(0, 0, RC_SW_UP, RC_SW_UP); Step();
    Chassis.set_goal.set_L0_Left = 0.18f; Chassis.set_goal.set_L0_Right = 0.22f;
    now += REMOTE_TIMEOUT_MS; Chassis.chassis_mode = NORMAL; Step(); Offline();
    NEAR(Chassis.set_goal.set_L0_Left, 0.18f); NEAR(Chassis.set_goal.set_L0_Right, 0.22f);
}
static void SpinMapping(void)
{
    /* Neither dial sign/amplitude nor steering may change the fixed SPIN target. */
    static const int16_t dial[] = {0, 1, -1, 5, -5, 6, -6, 20, -20, 21, -21,
                                  330, -330, 660, -660, 1000, -1000};
    static const int16_t steering[] = {-660, 0, 660};
    for (unsigned i = 0U; i < sizeof(dial) / sizeof(dial[0]); ++i)
    {
        ++now; BothIw(steering[i % 3U], 660, RC_SW_UP, RC_SW_UP, dial[i]); Step(); Online();
        CHECK(Chassis.normal_state_request == NORMAL_SPIN);
        NEAR(Chassis.set_goal.yaw_set_v, 3.0f);
        NEAR(Chassis.set_goal.v_set, 2.0f);
        NEAR(Chassis.set_goal.set_L0_Left, 0.25f);
        CHECK(recovery_dbg_rx_on_seq == 1U);
    }
    for (unsigned i = 0U; i < 3U; ++i)
    {
        ++now; BothIw(steering[i], -330, RC_SW_MID, RC_SW_MID, dial[13U + i]); Step(); Online();
        CHECK(Chassis.normal_state_request == NORMAL_MOVE);
        NEAR(Chassis.set_goal.yaw_set_v, i == 0U ? 3.0f : (i == 1U ? 0.0f : -3.0f));
        NEAR(Chassis.set_goal.v_set, -1.0f);
        NEAR(Chassis.set_goal.set_L0_Left, 0.20f);
        CHECK(recovery_dbg_rx_on_seq == 1U);
    }
    /* Select S2 from the configured source even when the other source disagrees. */
    ++now; LocalIw(-330, 330, RC_SW_UP, RC_SW_UP, 0);
    Sticks(-330, -660); StatusIw(RC_SW_MID, RC_SW_MID, 0, -660); Step(); Online();
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_LOCAL
    CHECK(Chassis.normal_state_request == NORMAL_SPIN);
    NEAR(Chassis.set_goal.yaw_set_v, 3.0f); NEAR(Chassis.set_goal.v_set, 1.0f);
#else
    CHECK(Chassis.normal_state_request == NORMAL_MOVE);
    NEAR(Chassis.set_goal.yaw_set_v, 1.5f); NEAR(Chassis.set_goal.v_set, -2.0f);
#endif
    ++now; LocalIw(330, 330, RC_SW_MID, RC_SW_MID, 660);
    Sticks(-660, -660); StatusIw(RC_SW_UP, RC_SW_UP, 0, 0); Step(); Online();
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_LOCAL
    CHECK(Chassis.normal_state_request == NORMAL_MOVE);
    NEAR(Chassis.set_goal.yaw_set_v, -1.5f); NEAR(Chassis.set_goal.v_set, 1.0f);
#else
    CHECK(Chassis.normal_state_request == NORMAL_SPIN);
    NEAR(Chassis.set_goal.yaw_set_v, 3.0f); NEAR(Chassis.set_goal.v_set, -2.0f);
    const int16_t extremes[] = {INT16_MIN, INT16_MAX};
    for (unsigned i = 0U; i < 2U; ++i)
    {
        ++now; Sticks(660, 0); StatusIw(RC_SW_UP, RC_SW_UP, 0, extremes[i]); Step(); Online();
        CHECK(Chassis.normal_state_request == NORMAL_SPIN);
        NEAR(Chassis.set_goal.yaw_set_v, 3.0f);
    }
#endif
    CHECK(recovery_dbg_rx_on_seq == 1U);
    ++now; BothIw(0, 0, RC_SW_DOWN, RC_SW_DOWN, 660); Step(); Offline();
    ++now; BothIw(0, 0, RC_SW_DOWN, 0, -660); Step(); Offline();
}
static void SpinStartup(void)
{
    Chassis.chassis_mode = FALLING_DOWN;
    Both(0, 0, RC_SW_UP, RC_SW_UP); Step(); Online();
    CHECK(Chassis.normal_state_request == NORMAL_MOVE);
    CHECK(recovery_dbg_rx_on_seq == 1U);
    ++now; Chassis.chassis_mode = FALLING_TO_NORMAL; Step(); Online();
    CHECK(Chassis.normal_state_request == NORMAL_MOVE);
    ++now; Chassis.chassis_mode = NORMAL; Step(); Online();
    CHECK(Chassis.normal_state_request == NORMAL_SPIN);
    NEAR(Chassis.set_goal.yaw_set_v, 3.0f);
    CHECK(recovery_dbg_rx_on_seq == 1U);
    /* S2 exits and re-enters SPIN without creating another enable edge. */
    ++now; BothIw(-330, 0, RC_SW_UP, RC_SW_MID, 660); Step(); Online();
    CHECK(Chassis.normal_state_request == NORMAL_MOVE);
    NEAR(Chassis.set_goal.yaw_set_v, 1.5f);
    ++now; Both(660, 0, RC_SW_UP, RC_SW_UP); Step(); Online();
    CHECK(Chassis.normal_state_request == NORMAL_SPIN);
    NEAR(Chassis.set_goal.yaw_set_v, 3.0f);
    CHECK(recovery_dbg_rx_on_seq == 1U);
}
static void SpinTimeout(void)
{
    Both(0, 0, RC_SW_UP, RC_SW_UP); Step(); Online();
    CHECK(Chassis.normal_state_request == NORMAL_SPIN);
    NEAR(Chassis.set_goal.yaw_set_v, 3.0f);
    now = REMOTE_TIMEOUT_MS; Step(); Offline();
    ++now; Both(0, 0, RC_SW_UP, RC_SW_UP); Step(); Offline();
    CHECK(remote_rearm_required == 1U);
    ++now; Both(0, 0, RC_SW_UP, RC_SW_MID); Step(); Offline();
    CHECK(remote_rearm_required == 1U);
    ++now; Both(0, 0, RC_SW_DOWN, RC_SW_DOWN); Step(); Offline();
    CHECK(remote_rearm_required == 0U);
    ++now; Both(0, 0, RC_SW_UP, RC_SW_UP); Step(); Online();
    CHECK(Chassis.normal_state_request == NORMAL_SPIN);
    NEAR(Chassis.set_goal.yaw_set_v, 3.0f);
    CHECK(recovery_dbg_rx_on_seq == 2U);
#if CHASSIS_CTRL_SOURCE == CHASSIS_CTRL_SOURCE_GIMBAL
    ++now; StatusIw(RC_SW_UP, RC_SW_UP, 1, 660); Step(); Offline();
    CHECK(remote_rearm_required == 1U);
#endif
}
int main(int argc, char **argv)
{
    if (argc != 2) return 2;
    Chassis.chassis_mode = NORMAL;
    if (strcmp(argv[1], "protocol") && strcmp(argv[1], "startup_status") &&
        strcmp(argv[1], "fifo") && strcmp(argv[1], "snapshot") &&
        strcmp(argv[1], "wrap") && strcmp(argv[1], "rearm")) PrepareDown();
    if (!strcmp(argv[1], "rearm")) Rearm();
    else if (!strcmp(argv[1], "protocol")) Protocol();
    else if (!strcmp(argv[1], "mapping")) Mapping();
    else if (!strcmp(argv[1], "timeout")) Timeout();
    else if (!strcmp(argv[1], "missing_sticks")) Missing(1);
    else if (!strcmp(argv[1], "missing_status")) Missing(0);
    else if (!strcmp(argv[1], "lost")) Lost();
    else if (!strcmp(argv[1], "burst")) Burst();
    else if (!strcmp(argv[1], "direct")) Direct();
    else if (!strcmp(argv[1], "startup_status")) StartupStatus();
    else if (!strcmp(argv[1], "wrap")) Wrap();
    else if (!strcmp(argv[1], "gaps")) Gaps();
    else if (!strcmp(argv[1], "fifo")) Fifo();
    else if (!strcmp(argv[1], "stream")) Stream();
    else if (!strcmp(argv[1], "snapshot")) Snapshot();
    else if (!strcmp(argv[1], "leg_length")) LegLength();
    else if (!strcmp(argv[1], "spin_mapping")) SpinMapping();
    else if (!strcmp(argv[1], "spin_startup")) SpinStartup();
    else if (!strcmp(argv[1], "spin_timeout")) SpinTimeout();
    else return 2;
    CHECK(interrupt_mask == 0);
    printf("remote source %d %-14s: %u checks, %u failures\n", CHASSIS_CTRL_SOURCE, argv[1], checks, failures);
    return failures ? 1 : 0;
}
