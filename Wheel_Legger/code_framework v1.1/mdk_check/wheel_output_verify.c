/* Real transmission and CAN packing; hardware calls are capture stubs. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "DJI_Motor.h"
#include "wheel_definitions.inc"
#define PI 3.14159265358979323846f
#define Chassis_Time 1
#define J8009_T_MIN (-40.0f)
#define J8009_T_MAX 40.0f
#define LEFT_Wheel_CAN_hfdcan wheel_can
#define LEFT_Joint_Motor_CAN_hfdcan joint_can_left
#define RIGHT_Joint_Motor_CAN_hfdcan joint_can_right

enum { OFFLINE, ONLINE };
enum { LEFT_Leg, RIGHT_Leg };
typedef struct { float Yaw; } INS_t;
typedef struct
{
    int chassis_enable;
    DJI_Motor_Info_Typedef Wheel_Motor[4];
    int Joint_Motor[4];
    struct { struct { float L0, theta, d_theta, Tp, torque_set[2]; } vmc; } leg_situation[2];
    struct { float x, Estimate_dx, d_yaw, theta, d_theta; } body_state;
    struct { float v_set, yaw_set, yaw_set_v; } set_goal;
} Chassis_Info_Typedef;
INS_t INS;
Chassis_Info_Typedef Chassis;
static hcan_t wheel_can, joint_can_left, joint_can_right;
static uint8_t frame[8];
static float feedback[4];
static unsigned int sends, checks, failures;
static uint32_t frame_id;
static void osDelay(uint16_t delay) { (void)delay; }
static void canx_send_data(hcan_t *can, uint32_t id, uint8_t *data, uint32_t size)
{
    (void)can;
    if (size == sizeof(frame)) memcpy(frame, data, size);
    frame_id = id;
    ++sends;
}
static void DM_Motor_Ctrl(hcan_t *can, int *motor, float pos, float vel,
                          float kp, float kd, float torque, uint16_t delay)
{
    (void)can; (void)motor; (void)pos; (void)vel;
    (void)kp; (void)kd; (void)torque; (void)delay;
}
static void LESO_Feedback(float left, float right, float left_tp, float right_tp)
{
    feedback[0] = left; feedback[1] = right;
    feedback[2] = left_tp; feedback[3] = right_tp;
}
#include "wheel_implementation.inc"

#define CHECK(condition) do { ++checks; if (!(condition)) { \
    ++failures; fprintf(stderr, "WHEEL FAIL line %d: %s\n", __LINE__, #condition); \
} } while (0)
static int16_t SentCurrent(unsigned int index)
{
    return (int16_t)(((uint16_t)frame[index * 2] << 8) | frame[index * 2 + 1]);
}
static float Limited(float value)
{
    return fmaxf(-4.8f, fminf(4.8f, value));
}
static void CheckOutput(float left, float right, int enabled)
{
    int16_t expected_left = enabled ? (int16_t)(Limited(left) * LeftWheelT_TO_Current) : 0;
    int16_t expected_right = enabled ? (int16_t)(Limited(right) * RightWheelT_TO_Current) : 0;
    Chassis.chassis_enable = enabled;
    Chassis.Wheel_Motor[RIGHT_Wheel].ID_Set.TxIdentifier = Chassis_3508_MotorA_TxID;
    Chassis.Wheel_Motor[LEFT_Wheel].wheel_T = left;
    Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T = right;
    sends = 0;
    Chassis_CanTransimit();
    printf("request=(%+.4f,%+.4f) current=(%d,%d) expected=(%d,%d) online=%d\n",
           left, right, SentCurrent(LEFT_Wheel), SentCurrent(RIGHT_Wheel), expected_left, expected_right, enabled);
    CHECK(sends == 1);
    CHECK(frame_id == 0x200);
    CHECK(SentCurrent(LEFT_Wheel) == expected_left);
    CHECK(SentCurrent(RIGHT_Wheel) == expected_right);
    CHECK(SentCurrent(2) == 0 && SentCurrent(3) == 0);
    if (enabled)
    {
        CHECK(fabsf(feedback[0] - Limited(left)) < 0.0001f);
        CHECK(fabsf(feedback[1] - Limited(right)) < 0.0001f);
    }
}
int main(void)
{
    const float cases[][2] = {
        {1.0f, 1.0f}, {-1.0f, -1.0f}, {1.0f, -1.0f}, {-1.0f, 1.0f},
        {4.8f, -4.8f}, {4.81f, 4.81f}, {9.83f, 8.0f}, {9.85f, 8.0f},
        {10.0f, 8.0f}, {-10.0f, -8.0f}, {8.0f, 10.0f}, {-8.0f, -10.0f},
        {20.0f, 20.0f}, {-20.0f, -20.0f}, {0.0f, 0.0f}
    };
    unsigned int i;
    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i)
        CheckOutput(cases[i][0], cases[i][1], ONLINE);
    CheckOutput(10.0f, 8.0f, OFFLINE);

    /* An admissible handoff pose with residual yaw/body/leg angular rates.
     * Real LQR outputs must retain direction through the real CAN path. */
    memset(&Chassis, 0, sizeof(Chassis));
    INS.Yaw = 0.4f;
    Chassis.body_state.theta = 0.19f;
    Chassis.body_state.d_theta = 1.5f;
    Chassis.body_state.d_yaw = 0.2f;
    for (i = 0; i < 2; ++i)
    {
        Chassis.leg_situation[i].vmc.L0 = 0.139f;
        Chassis.leg_situation[i].vmc.theta = 1.7f - PI / 2.0f + Chassis.body_state.theta;
        Chassis.leg_situation[i].vmc.d_theta = 0.5f;
    }
    LQR_Calc(0.139f, 0.139f);
    printf("actual LQR handoff sample: ");
    CheckOutput(Chassis.Wheel_Motor[LEFT_Wheel].wheel_T,
                Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T, ONLINE);
    printf("wheel output: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
