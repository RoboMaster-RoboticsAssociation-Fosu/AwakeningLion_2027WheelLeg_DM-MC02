#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

typedef struct { float Yaw; } INS_t;
#include "leso_api.inc"

enum { OFFLINE = 0, ONLINE = 1, NORMAL = 0, FALLING_DOWN = 1, ZERO_FORCE = 2 };
typedef struct {
    int chassis_enable, chassis_mode;
    struct { float x, Estimate_dx, d_yaw, theta, d_theta; } body_state;
    struct { struct { float L0, theta, d_theta, F0, Tp; } vmc; } leg_situation[2];
    struct { float wheel_T; } Wheel_Motor[2];
} TestChassis;
TestChassis Chassis;
INS_t INS;
static int fault_channel = -1;
static unsigned checks, failures;
static void verify_LESO_Update(LESO_t *o, const float *Ad, const float *Bd, const float *L,
                               const float *y, const float *u, const float *dlim);
#include "leso_implementation.inc"

/* All tests run the real update; only validity tests inject a post-update fault. */
static void verify_LESO_Update(LESO_t *o, const float *Ad, const float *Bd, const float *L,
                               const float *y, const float *u, const float *dlim)
{
    LESO_Update(o, Ad, Bd, L, y, u, dlim);
    if (fault_channel >= 0) o->dh[fault_channel] = NAN;
}

#ifdef VERIFY_LEGACY
static TestChassis legacy_Chassis;
static INS_t legacy_INS;
#include "leso_legacy.inc"
#endif

static float saved_Ad[100][6], saved_Bd[40][6], saved_L[140][6];
static void check(int pass, const char *message)
{
    ++checks;
    if (!pass) { ++failures; fprintf(stderr, "FAIL: %s\n", message); }
}
static void near(float actual, float expected, float tolerance, const char *message)
{
    check(isfinite(actual) && isfinite(expected) && fabsf(actual - expected) <= tolerance, message);
}
static void outputs(void)
{
    Chassis.Wheel_Motor[LEFT_Wheel].wheel_T = 1.1f;
    Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T = -2.2f;
    Chassis.leg_situation[LEFT_Leg].vmc.Tp = 3.3f;
    Chassis.leg_situation[RIGHT_Leg].vmc.Tp = -4.4f;
    Chassis.leg_situation[LEFT_Leg].vmc.F0 = 55.0f;
    Chassis.leg_situation[RIGHT_Leg].vmc.F0 = 66.0f;
}
static void tick(void)
{
    outputs();
    LESO_Service();
    LESO_Feedback(Chassis.Wheel_Motor[LEFT_Wheel].wheel_T,
                  Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T,
                  Chassis.leg_situation[LEFT_Leg].vmc.Tp,
                  Chassis.leg_situation[RIGHT_Leg].vmc.Tp);
}
static void reset(int real_model)
{
    Chassis.chassis_enable = OFFLINE;
    LESO_Service();
    memset(&Chassis, 0, sizeof(Chassis));
    memset(&INS, 0, sizeof(INS));
    Chassis.chassis_enable = ONLINE;
    Chassis.chassis_mode = NORMAL;
    Chassis.leg_situation[LEFT_Leg].vmc.L0 = 0.15f;
    Chassis.leg_situation[RIGHT_Leg].vmc.L0 = 0.20f;
    leso_wheel_scale = leso_leg_scale = 0.0f;
    leso_wheel_mode = LESO_WHEEL_FULL;
    fault_channel = -1;
    if (real_model) {
        memcpy(AdP, saved_Ad, sizeof(AdP));
        memcpy(BdP, saved_Bd, sizeof(BdP));
        memcpy(LP, saved_L, sizeof(LP));
    } else {
        /* Hold known disturbance values to isolate actuator routing and ramps. */
        memset(AdP, 0, sizeof(AdP));
        memset(BdP, 0, sizeof(BdP));
        memset(LP, 0, sizeof(LP));
    }
    tick();
    leso.dh[0] = 0.4f; leso.dh[1] = -0.7f;
    leso.dh[2] = 2.0f; leso.dh[3] = -3.0f;
}
static void run_ticks(int count)
{
    while (count-- > 0) tick();
}
static void expect_outputs(float wheel, float leg)
{
    near(Chassis.Wheel_Motor[LEFT_Wheel].wheel_T, 1.1f - wheel * 0.4f, 2e-6f, "left wheel routing/sign");
    near(Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T, -2.2f + wheel * 0.7f, 2e-6f, "right wheel routing/sign");
    near(Chassis.leg_situation[LEFT_Leg].vmc.Tp, 3.3f - leg * 2.0f, 2e-6f, "left hip routing/sign");
    near(Chassis.leg_situation[RIGHT_Leg].vmc.Tp, -4.4f + leg * 3.0f, 2e-6f, "right hip routing/sign");
    near(Chassis.leg_situation[LEFT_Leg].vmc.F0, 55, 0, "left support force unchanged");
    near(Chassis.leg_situation[RIGHT_Leg].vmc.F0, 66, 0, "right support force unchanged");
}
static void routing(void)
{
    int group;
    for (group = 0; group < 4; ++group) {
        reset(0);
        leso_wheel_scale = (group & 1) ? 0.2f : 0.0f;
        leso_leg_scale = (group & 2) ? 0.6f : 0.0f;
        run_ticks(4000);
        near(leso_dbg_comp_wheel, leso_wheel_scale, 2e-5f, "wheel target settles");
        near(leso_dbg_comp_leg, leso_leg_scale, 2e-5f, "hip target settles");
        expect_outputs(leso_dbg_comp_wheel, leso_dbg_comp_leg);
        check(leso_active == (group != 0), "any group active");
        near(leso_dbg_dh[0], 0.4f, 0, "left wheel estimate remains visible");
        near(leso_dbg_dh[3], -3.0f, 0, "right hip estimate remains visible");
        near(u_last[0], Chassis.Wheel_Motor[LEFT_Wheel].wheel_T, 0, "left wheel feedback order");
        near(u_last[1], Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T, 0, "right wheel feedback order");
        near(u_last[2], Chassis.leg_situation[LEFT_Leg].vmc.Tp, 0, "left hip feedback retained");
        near(u_last[3], Chassis.leg_situation[RIGHT_Leg].vmc.Tp, 0, "right hip feedback retained");
    }
}
static void ramps_and_modes(void)
{
    float last, wheel_before;
    int n, mode;
    reset(0);
    leso_wheel_scale = 0.8f;
    run_ticks(490);
    near(leso_dbg_comp_wheel, 0, 0, "no injection before gate time");
    for (n = 0; n < 32 && leso_dbg_comp_wheel == 0.0f; ++n) tick();
    near(leso_dbg_comp_wheel, 0.8f * LESO_COMP_RATE, 1e-8f, "first gated wheel step");
    near(leso_dbg_comp_leg, 0, 0, "disabled hip does not ramp");
    run_ticks(1000);
    wheel_before = leso_dbg_comp_wheel;
    leso_leg_scale = 0.3f;
    tick();
    near(leso_dbg_comp_leg, 0.3f * LESO_COMP_RATE, 1e-8f, "hip enabled independently without reseed");
    check(leso_dbg_comp_wheel >= wheel_before, "wheel ramp not reset by hip target");
    run_ticks(2000);
    last = leso_dbg_comp_wheel;
    leso_wheel_scale = 0;
    tick();
    near(leso_dbg_comp_wheel, last * (1 - LESO_COMP_RATE), 1e-7f, "wheel shutdown ramps");
    run_ticks(6000);
    near(leso_dbg_comp_wheel, 0, 0, "wheel shutdown reaches exact zero");
    check(leso_dbg_comp_leg > 0.29f && leso_active, "hip remains active after wheel shuts down");
    leso_leg_scale = 0;
    run_ticks(6000);
    near(leso_dbg_comp_leg, 0, 0, "hip shutdown reaches exact zero");
    check(!leso_active, "no lingering active flag");
    expect_outputs(0, 0);
    for (mode = 0; mode < 3; ++mode) {
        reset(0); leso_wheel_scale = leso_leg_scale = 0.5f; run_ticks(1000);
        if (mode == 0) Chassis.chassis_enable = OFFLINE;
        else Chassis.chassis_mode = mode == 1 ? FALLING_DOWN : ZERO_FORCE;
        tick();
        near(leso_dbg_comp_wheel, 0, 0, "inactive mode clears wheel display");
        near(leso_dbg_comp_leg, 0, 0, "inactive mode clears hip display");
        near(leso_dbg_comp_wheel_common, 0, 0, "inactive mode clears common display");
        near(leso_dbg_comp_wheel_diff, 0, 0, "inactive mode clears differential display");
        near(leso_dbg_dh_wheel_common, 0, 0, "inactive mode clears common estimate display");
        near(leso_dbg_dh_wheel_diff, 0, 0, "inactive mode clears differential estimate display");
        check(!leso_active, "inactive mode clears activity");
        expect_outputs(0, 0);
        Chassis.chassis_enable = ONLINE; Chassis.chassis_mode = NORMAL;
        Chassis.body_state.theta = 0.12f;
        tick();
        near(leso.e[8], 0, 0, "normal reentry seeds current feedback");
        for (n = 0; n < 4; ++n) near(leso.dh[n], 0, 0, "normal reentry clears disturbance history");
        near(leso_dbg_comp_wheel, 0, 0, "normal reentry waits for gate again");
    }
}
static void invalid_values(void)
{
    const float values[] = {-1, 2, NAN, INFINITY, -INFINITY};
    unsigned i;
    int channel, group;
    for (i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
        reset(0); leso_wheel_scale = values[i]; leso_leg_scale = 0.25f;
        run_ticks(6000);
        near(leso_dbg_comp_wheel, i == 1 ? 1.0f : 0.0f, 2e-5f, "invalid wheel target normalized");
        near(leso_dbg_comp_leg, 0.25f, 2e-5f, "invalid wheel target leaves hip target independent");
        reset(0); leso_leg_scale = values[i]; leso_wheel_scale = 0.25f;
        run_ticks(6000);
        near(leso_dbg_comp_leg, i == 1 ? 1.0f : 0.0f, 2e-5f, "invalid hip target normalized");
        near(leso_dbg_comp_wheel, 0.25f, 2e-5f, "invalid hip target leaves wheel target independent");
    }
    reset(0); leso_wheel_scale = leso_leg_scale = 0.6f; run_ticks(3000);
    leso_wheel_scale = NAN; leso_leg_scale = INFINITY; run_ticks(6000);
    expect_outputs(0, 0);
    check(!leso_active, "non-finite targets fade active groups to zero");
    for (group = 0; group < 4; ++group) {
        for (channel = 0; channel < 4; ++channel) {
            reset(0);
            leso_wheel_scale = (group & 1) ? 0.5f : 0;
            leso_leg_scale = (group & 2) ? 0.5f : 0;
            run_ticks(1000);
            fault_channel = channel;
            tick();
            expect_outputs(0, 0);
            check(!leso_active, "original shared disturbance validity gate retained");
        }
    }
}
static void observer_without_injection(void)
{
    LESO_t expected;
    float y[10] = {0};
    const float input[4] = {0.6f, -0.4f, 1.2f, -0.8f};
    int i, frame;
    reset(1);
    leso_wheel_mode = LESO_WHEEL_OFF;
    leso_wheel_scale = 1.0f;
    run_ticks(600);
    LESO_Seed(&leso, y);
    for (frame = 0; frame < 3; ++frame) {
        LESO_Feedback(input[0], input[1], input[2], input[3]);
        expected = leso;
        LESO_Update(&expected, Ad_f, Bd_f, L_f, y, input, leso_dlim);
        outputs(); LESO_Service();
        for (i = 0; i < 10; ++i) near(leso.xh[i], expected.xh[i], 1e-7f, "uncompensated observer uses all input channels");
        for (i = 0; i < 4; ++i) near(leso.dh[i], expected.dh[i], 1e-7f, "uncompensated observer disturbance update continues");
        expect_outputs(0, 0);
    }
    check(fabsf(leso.xh[5]) > 1e-7f, "observer actually predicted a moving state");
    check(fabsf(leso.dh[2]) > 1e-7f, "observer actually estimated a disturbance while injection off");
}


/* Independent signed fixtures specify the common/differential decomposition. */
static void component_routing(void)
{
    const struct { float left, right, common, diff; } cases[] = {
        {0.7f, 0.2f, 0.45f, 0.25f}, {0.7f, -0.2f, 0.25f, 0.45f},
        {0.5f, 0.5f, 0.5f, 0}, {0.5f, -0.5f, 0, 0.5f}, {0, 0, 0, 0},
        {-0.8f, -0.1f, -0.45f, -0.35f}, {-0.2f, 0.7f, 0.25f, -0.45f}
    };
    unsigned c;
    int mode;
    for (c = 0; c < sizeof(cases) / sizeof(cases[0]); ++c) {
        for (mode = 0; mode < 4; ++mode) {
            float common, diff;
            reset(0);
            leso.dh[0] = cases[c].left; leso.dh[1] = cases[c].right;
            leso_wheel_mode = (uint8_t)mode;
            leso_wheel_scale = 0.5f; leso_leg_scale = 0.3f;
            run_ticks(4000);
            common = leso_dbg_comp_wheel_common;
            diff = leso_dbg_comp_wheel_diff;
            near(common, (mode & 1) ? 0.5f : 0, 2e-5f, "common mode selection");
            near(diff, (mode & 2) ? 0.5f : 0, 2e-5f, "differential mode selection");
            near(leso_dbg_dh_wheel_common, cases[c].common, 1e-7f, "signed common estimate");
            near(leso_dbg_dh_wheel_diff, cases[c].diff, 1e-7f, "signed differential estimate");
            near(Chassis.Wheel_Motor[LEFT_Wheel].wheel_T,
                 1.1f - common * cases[c].common - diff * cases[c].diff, 2e-6f, "component left wheel routing");
            near(Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T,
                 -2.2f - common * cases[c].common + diff * cases[c].diff, 2e-6f, "component right wheel routing");
            near(leso_dbg_comp_wheel, fmaxf(common, diff), 0, "summary shows maximum component scale");
            near(Chassis.leg_situation[LEFT_Leg].vmc.Tp, 3.3f - leso_dbg_comp_leg * 2, 2e-6f, "wheel mode leaves left hip injection independent");
            near(Chassis.leg_situation[RIGHT_Leg].vmc.Tp, -4.4f + leso_dbg_comp_leg * 3, 2e-6f, "wheel mode leaves right hip injection independent");
            near(u_last[0], Chassis.Wheel_Motor[LEFT_Wheel].wheel_T, 0, "projected left wheel input feedback");
            near(u_last[1], Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T, 0, "projected right wheel input feedback");
            if (mode == 3) {
                near(Chassis.Wheel_Motor[LEFT_Wheel].wheel_T, 1.1f - common * cases[c].left, 0, "full mode exact original left arithmetic");
                near(Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T, -2.2f - diff * cases[c].right, 0, "full mode exact original right arithmetic");
            }
        }
    }
}

static void component_switching(void)
{
    const uint8_t invalid_modes[] = {4, 16, 255};
    float common, diff, hip, left, right;
    unsigned i;
    int channel, mode;
    reset(0);
    leso_wheel_scale = 0.8f; leso_leg_scale = 0.3f;
    leso_wheel_mode = LESO_WHEEL_COMMON;
    run_ticks(4000);
    common = leso_dbg_comp_wheel_common; hip = leso_dbg_comp_leg;
    left = Chassis.Wheel_Motor[LEFT_Wheel].wheel_T;
    right = Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T;
    leso_wheel_mode = LESO_WHEEL_DIFF;
    tick();
    near(leso_dbg_comp_wheel_common, common * (1 - LESO_COMP_RATE), 1e-7f, "common-to-diff fades common smoothly");
    near(leso_dbg_comp_wheel_diff, 0.8f * LESO_COMP_RATE, 1e-8f, "common-to-diff starts differential smoothly");
    near(leso_dbg_comp_leg, hip, 1e-6f, "wheel switch does not restart hip ramp");
    near(leso.dh[2], 2, 0, "wheel switch does not reseed observer");
    check(fabsf(Chassis.Wheel_Motor[LEFT_Wheel].wheel_T - left) <= 0.8f * LESO_COMP_RATE * 0.7f + 1e-6f,
          "left wheel mode-switch output step bounded by ramp");
    check(fabsf(Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T - right) <= 0.8f * LESO_COMP_RATE * 0.7f + 1e-6f,
          "right wheel mode-switch output step bounded by ramp");
    run_ticks(6000);
    near(leso_dbg_comp_wheel_common, 0, 0, "deselected common reaches exact zero");
    near(leso_dbg_comp_wheel_diff, 0.8f, 2e-5f, "selected differential settles");
    diff = leso_dbg_comp_wheel_diff;
    leso_wheel_mode = LESO_WHEEL_FULL;
    tick();
    near(leso_dbg_comp_wheel_common, 0.8f * LESO_COMP_RATE, 1e-8f, "full mode resumes common without gate restart");
    near(leso_dbg_comp_wheel_diff, diff, 1e-6f, "full mode retains differential history");
    leso_wheel_mode = LESO_WHEEL_OFF;
    tick();
    check(leso_dbg_comp_wheel_common > 0 && leso_dbg_comp_wheel_diff > 0, "mode off fades both rather than switching abruptly");
    run_ticks(6000);
    near(leso_dbg_comp_wheel_common, 0, 0, "mode off common zero");
    near(leso_dbg_comp_wheel_diff, 0, 0, "mode off differential zero");
    near(leso_dbg_comp_wheel, 0, 0, "mode off summary zero");
    check(leso_active && leso_dbg_comp_leg > 0.29f, "mode off preserves hip injection");
    leso_leg_scale = 0; run_ticks(6000);
    expect_outputs(0, 0); check(!leso_active, "mode off plus hip off clears activity");
    for (i = 0; i < sizeof(invalid_modes) / sizeof(invalid_modes[0]); ++i) {
        reset(0); leso_wheel_scale = 0.5f; run_ticks(4000);
        common = leso_dbg_comp_wheel_common;
        leso_wheel_mode = invalid_modes[i]; tick();
        near(leso_dbg_comp_wheel_common, common * (1 - LESO_COMP_RATE), 1e-7f, "invalid mode fades common");
        near(leso_dbg_comp_wheel_diff, common * (1 - LESO_COMP_RATE), 1e-7f, "invalid mode fades differential");
        run_ticks(6000); expect_outputs(0, 0);
        check(!leso_active, "invalid mode cannot activate wheel injection");
    }
    reset(0); leso_wheel_scale = 0.8f; leso_wheel_mode = LESO_WHEEL_COMMON;
    run_ticks(2000); leso_wheel_mode = LESO_WHEEL_DIFF; run_ticks(100);
    Chassis.chassis_enable = OFFLINE; tick();
    near(leso_dbg_comp_wheel_common, 0, 0, "offline clears unequal common state");
    near(leso_dbg_comp_wheel_diff, 0, 0, "offline clears unequal differential state");
    near(leso_dbg_dh_wheel_common, 0, 0, "offline clears common estimate display");
    near(leso_dbg_dh_wheel_diff, 0, 0, "offline clears differential estimate display");
    Chassis.chassis_enable = ONLINE; tick();
    check(leso_wheel_mode == LESO_WHEEL_DIFF, "mode selection persists across enable cycle");
    near(leso_dbg_comp_wheel_diff, 0, 0, "reentry restarts gate for differential mode");
    run_ticks(600);
    near(leso_dbg_comp_wheel_common, 0, 0, "reentry keeps common deselected");
    check(leso_dbg_comp_wheel_diff > 0, "reentry permits selected differential after gate");
    for (mode = 0; mode < 4; ++mode) {
        for (channel = 0; channel < 4; ++channel) {
            reset(0); leso_wheel_mode = (uint8_t)mode; leso_wheel_scale = leso_leg_scale = 0.5f;
            run_ticks(1000); fault_channel = channel; tick();
            expect_outputs(0, 0); check(!leso_active, "shared validity gate applies to projected wheel modes");
        }
    }
}

#ifdef VERIFY_LEGACY
static void legacy_equivalence(void)
{
    const float scales[] = {0, 0.2f, 0.8f, 1};
    unsigned group;
    int frame, i;
    for (group = 0; group < sizeof(scales) / sizeof(scales[0]); ++group) {
        reset(1);
        LESO_Init(&leso);
        online_last = 0;
        memcpy(legacy_AdP, saved_Ad, sizeof(saved_Ad));
        memcpy(legacy_BdP, saved_Bd, sizeof(saved_Bd));
        memcpy(legacy_LP, saved_L, sizeof(saved_L));
        legacy_Chassis = Chassis; legacy_INS = INS;
        legacy_online_last = 0;
        leso_wheel_scale = leso_leg_scale = scales[group];
#ifdef VERIFY_LEGACY_SPLIT
        leso_leg_scale = scales[3 - group];
        legacy_leso_wheel_scale = leso_wheel_scale;
        legacy_leso_leg_scale = leso_leg_scale;
#else
        legacy_leso_comp_scale = scales[group];
#endif
        for (frame = 0; frame < 2500; ++frame) {
            float t = frame * 0.001f;
            Chassis.body_state.theta = 0.08f * sinf(3 * t);
            Chassis.body_state.d_theta = 0.24f * cosf(3 * t);
            Chassis.body_state.x = 0.02f * sinf(t);
            Chassis.body_state.Estimate_dx = 0.02f * cosf(t);
            INS.Yaw = 0.04f * sinf(2 * t);
            Chassis.body_state.d_yaw = 0.08f * cosf(2 * t);
            Chassis.leg_situation[LEFT_Leg].vmc.theta = 0.1f * sinf(2 * t);
            Chassis.leg_situation[LEFT_Leg].vmc.d_theta = 0.2f * cosf(2 * t);
            Chassis.leg_situation[RIGHT_Leg].vmc.theta = 0.06f * sinf(2 * t + 0.2f);
            Chassis.leg_situation[RIGHT_Leg].vmc.d_theta = 0.12f * cosf(2 * t + 0.2f);
            Chassis.leg_situation[LEFT_Leg].vmc.L0 = 0.18f + 0.02f * sinf(t);
            Chassis.leg_situation[RIGHT_Leg].vmc.L0 = 0.22f + 0.02f * cosf(t);
            outputs(); legacy_Chassis = Chassis; legacy_INS = INS;
            LESO_Service(); legacy_LESO_Service();
#ifdef VERIFY_LEGACY_SPLIT
            near(leso_dbg_comp_wheel, legacy_leso_dbg_comp_wheel, 1e-7f, "full mode reproduces preceding wheel ramp");
            near(leso_dbg_comp_leg, legacy_leso_dbg_comp_leg, 1e-7f, "full mode reproduces preceding hip ramp");
#else
            near(leso_dbg_comp_wheel, legacy_leso_dbg_comp, 1e-7f, "equal scales reproduce legacy wheel ramp");
            near(leso_dbg_comp_leg, legacy_leso_dbg_comp, 1e-7f, "equal scales reproduce legacy hip ramp");
#endif
            for (i = 0; i < 2; ++i) {
                near(Chassis.Wheel_Motor[i].wheel_T, legacy_Chassis.Wheel_Motor[i].wheel_T, 1e-6f, "legacy wheel output equivalence");
                near(Chassis.leg_situation[i].vmc.Tp, legacy_Chassis.leg_situation[i].vmc.Tp, 1e-6f, "legacy hip output equivalence");
            }
            for (i = 0; i < 10; ++i) near(leso.xh[i], legacy_leso.xh[i], 1e-6f, "legacy observer state equivalence");
            for (i = 0; i < 4; ++i) near(leso.dh[i], legacy_leso.dh[i], 1e-6f, "legacy disturbance equivalence");
            LESO_Feedback(Chassis.Wheel_Motor[LEFT_Wheel].wheel_T, Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T,
                          Chassis.leg_situation[LEFT_Leg].vmc.Tp, Chassis.leg_situation[RIGHT_Leg].vmc.Tp);
            legacy_LESO_Feedback(legacy_Chassis.Wheel_Motor[LEFT_Wheel].wheel_T, legacy_Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T,
                                 legacy_Chassis.leg_situation[LEFT_Leg].vmc.Tp, legacy_Chassis.leg_situation[RIGHT_Leg].vmc.Tp);
        }
    }
}
#endif

int main(void)
{
    near(leso_wheel_scale, 0, 0, "default wheel target is off");
    near(leso_leg_scale, 1, 0, "default hip target preserves standing baseline");
    check(leso_wheel_mode == 3, "default wheel mode is full");
    memcpy(saved_Ad, AdP, sizeof(AdP));
    memcpy(saved_Bd, BdP, sizeof(BdP));
    memcpy(saved_L, LP, sizeof(LP));
    routing(); ramps_and_modes(); invalid_values(); observer_without_injection();
    component_routing(); component_switching();
#ifdef VERIFY_LEGACY
    legacy_equivalence();
#else
    puts("Legacy differential comparison not requested (pass original LESO.c as argument).");
#endif
    printf("LESO split: %u checks, %u failures\n", checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
