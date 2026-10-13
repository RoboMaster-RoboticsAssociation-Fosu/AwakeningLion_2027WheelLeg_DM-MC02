/* 真正的固件实现由脚本提取；数值参照、场景输入及硬件替身放在本文件。 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define __packed
#define Chassis_Time 1
#define mirror (-1.0f)
typedef int hcan_t;
typedef int Pid_Set_Typedef;
typedef int PidTypedef;
#include "ground_api.inc"
Chassis_Info_Typedef Chassis;
INS_t INS;
float fb_dt = 0.001f, F_roll;
hcan_t hfdcan1, hfdcan2;
static uint32_t tick_ms, primask;
static float test_dt = 0.001f;
static unsigned checks, failures, unlock_count, inject_after_unlock;
static int16_t sent_current[2];
static float sent_joint[4];
static float arm_sin_f32(float x) { return sinf(x); }
static float arm_cos_f32(float x) { return cosf(x); }
static uint32_t HAL_GetTick(void) { return tick_ms; }
static uint32_t __get_PRIMASK(void) { return primask; }
static void __disable_irq(void) { primask = 1U; }
static void __set_PRIMASK(uint32_t value)
{
    primask = value;
    if (++unlock_count == inject_after_unlock) {
        INS.Pitch = 0.24f; INS.MotionAccel_n[2] = 30.0f;
        Chassis.Joint_Motor[LEFT_BACK_id].Data.pos += 0.12f;
        Chassis.Joint_Motor[LEFT_BACK_id].Data.tor += 12.0f;
    }
}
static float DWT_GetDeltaT(uint32_t *last) { (void)last; return test_dt; }
uint16_t DM_Enable_Motor(hcan_t *can, uint16_t id, uint16_t mode, uint8_t delay)
{ (void)can; (void)id; (void)mode; (void)delay; return 0; }
void DM_Motor_Ctrl(hcan_t *can, volatile DM_Motor_Info_Typedef *m, float p, float v, float kp, float kd, float torque, uint8_t delay)
{
    (void)can; (void)p; (void)v; (void)kp; (void)kd; (void)delay;
    sent_joint[m - Chassis.Joint_Motor] = torque;
}
void DJI_Motor_ctrl(DJI_Motor_Info_Typedef *m, hcan_t *can, uint16_t delay)
{ (void)can; (void)delay; sent_current[0] = m[0].Data.SET_Current; sent_current[1] = m[1].Data.SET_Current; }
#include "ground_implementation.inc"
#ifdef VERIFY_GROUND_BASELINE
#include "ground_baseline.inc"
#endif
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); } } while(0)
#define NEAR(a,b,e) CHECK(isfinite(a) && fabsf((a)-(b)) <= (e))
static float StaticLoad(void) { return body_mg * 0.5f + (GROUND_LEG_MASS_KG + GROUND_WHEEL_MASS_KG)*GROUND_GRAVITY_M_S2; }
static void Fresh(void) { unsigned i; for(i=0;i<CHASSIS_MOTOR_COUNT;++i) Chassis_MotorFeedbackReceived((Chassis_MotorFeedback_e)i); }
static void Pose(unsigned leg, float length, float angle)
{
    vmc_leg_t *v = &Chassis.leg_situation[leg].vmc;
    float a;
    VMC_init(v);
    a = acosf((v->l1*v->l1 + length*length - v->l2*v->l2)/(2.0f*v->l1*length));
    v->phi1 = pi*0.5f + angle + a;
    v->phi4 = pi*0.5f + angle - a;
    v->d_phi1 = v->d_phi4 = 0;
    VMC_calc_1(v,&INS,0.001f);
}
static void FeedbackForce(unsigned leg, float axial, float swing, float out[2])
{
    vmc_leg_t *v = &Chassis.leg_situation[leg].vmc;
    float oldF=v->F0, oldTp=v->Tp;
    v->F0=axial; v->Tp=swing; VMC_calc_2(v);
    out[0]=v->torque_set[0]*(leg==RIGHT_Leg?-1.0f:1.0f);
    out[1]=v->torque_set[1]*(leg==RIGHT_Leg?-1.0f:1.0f);
    v->F0=oldF; v->Tp=oldTp;
}
static void Loads(float left, float right, float out[2][2])
{
    unsigned i; float ratio[2]={left,right};
    for(i=0;i<2;++i) {
        vmc_leg_t *v=&Chassis.leg_situation[i].vmc;
        float vertical=ratio[i]*StaticLoad()-(GROUND_LEG_MASS_KG+GROUND_WHEEL_MASS_KG)*GROUND_GRAVITY_M_S2;
        FeedbackForce(i,vertical*cosf(v->theta),-vertical*v->L0*sinf(v->theta),out[i]);
    }
}
static void Update(float dt, float accel, float torque[2][2])
{ GroundContact_Update(dt,accel,(const float (*)[2])torque); }
static void Reset(unsigned enabled)
{
    float t[2][2]={{0}};
    Chassis.chassis_enable=OFFLINE; LESO_Service();
    memset(&Chassis,0,sizeof(Chassis)); memset(&INS,0,sizeof(INS));
    for (unsigned i=0; i<CHASSIS_MOTOR_COUNT; ++i) motor_feedback_seen[i]=0U;
    tick_ms=0; test_dt=0.001f; unlock_count=inject_after_unlock=0;
    ground_control_enable=(uint8_t)enabled;
    ground_offline_debug_enable=0U;
    Chassis.chassis_enable=OFFLINE; Update(0.001f,0,t);
    Chassis.chassis_enable=ONLINE; Chassis.chassis_mode=NORMAL; Chassis.all_motors_online=1;
    Pose(0,0.20f,0); Pose(1,0.20f,0);
    Chassis.set_goal.set_L0_Left=Chassis.set_goal.set_L0_Right=0.20f;
    leg_length_ramp_initialized=0; fb_dt=0.001f; chassis_leg_length_reference_update();
    Fresh();
    leso_wheel_scale=0.2f; leso_leg_scale=1.0f; leso_wheel_mode=LESO_WHEEL_FULL;
}
/* 双精度位置正解作为有限差分参照，避免单精度差分消减误差掩盖雅可比结论。 */
static void PositionDouble(const vmc_leg_t *v, double p1, double p4, double *length, double *angle)
{
    double xb=v->l1*cos(p1), yb=v->l1*sin(p1);
    double xd=v->l5+v->l4*cos(p4), yd=v->l4*sin(p4);
    double bd2=(xd-xb)*(xd-xb)+(yd-yb)*(yd-yb);
    double A=2*v->l2*(xd-xb), B=2*v->l2*(yd-yb);
    double C=(double)v->l2*v->l2+bd2-(double)v->l3*v->l3;
    double p2=2*atan2(B+sqrt(A*A+B*B-C*C),A+C);
    double x=xb+v->l2*cos(p2)-v->l5*0.5, y=yb+v->l2*sin(p2);
    *length=hypot(x,y); *angle=atan2(y,x);
}
static void InverseAndProjection(void)
{
    unsigned leg,k; float lengths[]={0.139f,0.15f,0.20f,0.25f};
    Reset(0);
    for(leg=0;leg<2;++leg) for(k=0;k<4;++k) {
        int n;
        for(n=-3;n<=3;++n) {
            float t[2],F,T,angle=n*0.1f;
            vmc_leg_t *v=&Chassis.leg_situation[leg].vmc;
            Pose(leg,lengths[k],angle); FeedbackForce(leg,72.0f,3.5f,t);
            CHECK(VMC_Force_Calc(v,t[0]*(leg? -1:1),t[1]*(leg?-1:1),&F,&T));
            NEAR(F,72,3e-4f); NEAR(T,3.5f,2e-5f);
            FeedbackForce(leg,80*cosf(v->theta),-80*v->L0*sinf(v->theta),t);
            CHECK(VMC_Force_Calc(v,t[0]*(leg?-1:1),t[1]*(leg?-1:1),&F,&T));
            NEAR(F*cosf(v->theta)-T*sinf(v->theta)/v->L0,80,4e-4f);
            /* 中心差分独立核对 J 的径向/角向列，覆盖本车实际杆长。 */
            { double lp,lm,ap,am,h=1.0e-6;
              PositionDouble(v,v->phi1+h,v->phi4,&lp,&ap);
              PositionDouble(v,v->phi1-h,v->phi4,&lm,&am);
              NEAR(v->j11,(float)((lp-lm)/(2*h)),1e-6f);
              NEAR(v->j12,(float)((ap-am)/(2*h)),2e-6f);
              PositionDouble(v,v->phi1,v->phi4+h,&lp,&ap);
              PositionDouble(v,v->phi1,v->phi4-h,&lm,&am);
              NEAR(v->j21,(float)((lp-lm)/(2*h)),1e-6f);
              NEAR(v->j22,(float)((ap-am)/(2*h)),2e-6f);
            }
        }
    }
    { vmc_leg_t v=Chassis.leg_situation[0].vmc; float F=123,T=456;
      v.L0=0; CHECK(!VMC_Force_Calc(&v,0,0,&F,&T)); NEAR(F,123,0);
      v=Chassis.leg_situation[0].vmc; v.phi3=v.phi2; CHECK(!VMC_Force_Calc(&v,0,0,&F,&T));
      v=Chassis.leg_situation[0].vmc; v.phi1=v.phi2; CHECK(!VMC_Force_Calc(&v,0,0,&F,&T));
      v=Chassis.leg_situation[0].vmc; CHECK(!VMC_Force_Calc(&v,NAN,0,&F,&T));
    }
}
static void TimingAndInvalid(void)
{
    float t[2][2]; unsigned n; Reset(1); Loads(0,0,t);
    Update(0.001f,0,t); CHECK(Chassis.ground.initialized[0]); CHECK(!Chassis.ground.all_off_flag);
    NEAR(Chassis.ground.off_time_s[0],0,0);
    for(n=0;n<29;++n) Update(0.001f,0,t);
    CHECK(!Chassis.ground.all_off_flag); Update(0.001f,0,t); CHECK(Chassis.ground.all_off_flag);
    { float bad[]={0,-0.1f,0.051f,NAN,INFINITY};
      for(n=0;n<sizeof(bad)/sizeof(bad[0]);++n) {
        Update(bad[n],0,t); CHECK(!Chassis.ground.valid[0]); CHECK(Chassis.ground.all_off_flag);
        NEAR(Chassis.ground.off_time_s[0],0,0); NEAR(Chassis.ground.land_time_s[0],0,0);
      }
    }
    Loads(1,1,t); Update(0.001f,0,t); CHECK(Chassis.ground.all_off_flag);
    for(n=0;n<49;++n) Update(0.001f,0,t); CHECK(Chassis.ground.all_off_flag);
    Update(0.001f,0,t); CHECK(!Chassis.ground.all_off_flag);
    Reset(1); Loads(0,1,t); Update(0.001f,0,t);
    for(n=0;n<40;++n) Update(0.001f,0,t);
    CHECK(Chassis.ground.off_flag[0] && !Chassis.ground.off_flag[1]); CHECK(!Chassis.ground.all_off_flag);
    t[0][0]=NAN; Update(0.001f,0,t); CHECK(!Chassis.ground.valid[0] && Chassis.ground.off_flag[0]);
    Chassis.all_motors_online=0; Update(0.001f,0,t); CHECK(!Chassis.ground.valid[1]);
    Reset(1); Loads(0,0,t); Update(0.001f,0,t);
    for(n=0;n<6;++n) Update(n%2?0.003f:0.007f,0,t);
    CHECK(Chassis.ground.all_off_flag);
    ground_control_enable=0; Update(0.001f,0,t); CHECK(Chassis.ground.control_enabled);
    Chassis.chassis_enable=OFFLINE; Update(0.001f,0,t); CHECK(!Chassis.ground.control_enabled && !Chassis.ground.all_off_flag);
    Chassis.chassis_enable=ONLINE; Chassis.chassis_mode=FALLING_DOWN; Update(0.001f,0,t); CHECK(!Chassis.ground.valid[0]);
}
static void ThresholdEdgesAndReset(void)
{
    float t[2][2]; unsigned n;
    Reset(1); Loads(0.2001f,0.2001f,t); Update(0.001f,0,t);
    for(n=0;n<80;++n) Update(0.001f,0,t);
    CHECK(!Chassis.ground.all_off_flag); NEAR(Chassis.ground.off_time_s[0],0,0);
    Reset(1); Loads(0.1999f,0.1999f,t); Update(0.001f,0,t);
    for(n=0;n<30;++n) Update(0.001f,0,t);
    CHECK(Chassis.ground.all_off_flag);
    Loads(0.3499f,0.3499f,t);
    for(n=0;n<300;++n) Update(0.001f,0,t);
    CHECK(Chassis.ground.all_off_flag); NEAR(Chassis.ground.land_time_s[0],0,0);
    Loads(0.3501f,0.3501f,t);
    for(n=0;n<300;++n) Update(0.001f,0,t);
    CHECK(!Chassis.ground.all_off_flag);
    { int modes[]={FALLING_DOWN,FALLING_TO_NORMAL,ZERO_FORCE};
      for(n=0;n<3;++n) {
        Chassis.ground.all_off_flag=1; Chassis.ground.off_flag[0]=1; Chassis.ground.valid[0]=1;
        Chassis.chassis_mode=(Chassis_Mode_e)modes[n]; Update(0.001f,0,t);
        CHECK(!Chassis.ground.all_off_flag && !Chassis.ground.valid[0]); CHECK(Chassis.ground.control_enabled);
      }
    }
}
static double LegZ(double t,double L,double dL,double ddL,double theta,double dtheta,double ddtheta)
{ return -(L+dL*t+0.5*ddL*t*t)*cos(theta+dtheta*t+0.5*ddtheta*t*t); }
static void FiltersAndDynamics(void)
{
    float t[2][2]; Reset(1); Loads(1,1,t); Update(0.001f,0,t);
    NEAR(Chassis.ground.Fn_ratio[0],1,1e-6f);
    Loads(0,0,t); Update(0.001f,0,t);
    NEAR(Chassis.ground.Fn[0],StaticLoad()*(1-0.001f/0.021f),2e-5f);
    { vmc_leg_t *v=&Chassis.leg_situation[0].vmc; double h=0.0001,ddz; float mass=GROUND_LEG_MASS_KG+GROUND_WHEEL_MASS_KG;
      Pose(0,0.20f,0.25f); v->d_L0=0.08f; v->d_theta=0.4f; v->dd_L0=0.7f; v->dd_theta=-1.0f;
      Chassis.ground.dd_L0[0]=v->dd_L0; Chassis.ground.dd_theta[0]=v->dd_theta;
      FeedbackForce(0,80*cosf(v->theta),-80*v->L0*sinf(v->theta),t[0]);
      ddz=(LegZ(h,v->L0,v->d_L0,v->dd_L0,v->theta,v->d_theta,v->dd_theta)-2*LegZ(0,v->L0,v->d_L0,v->dd_L0,v->theta,v->d_theta,v->dd_theta)+LegZ(-h,v->L0,v->d_L0,v->dd_L0,v->theta,v->d_theta,v->dd_theta))/(h*h);
      Update(0.001f,2,t); NEAR(Chassis.ground.Fn_raw[0],80+mass*(GROUND_GRAVITY_M_S2+2+(float)ddz),3e-4f);
    }
    Reset(1); Loads(0.19f,0.19f,t); Update(0.001f,0,t);
    { unsigned n; for(n=0;n<20;++n) Update(0.001f,0,t); CHECK(!Chassis.ground.all_off_flag);
      Loads(1,1,t); Update(0.001f,0,t); CHECK(Chassis.ground.off_time_s[0]==0);
    }
}
static void ControlAndObserver(void)
{
    float groundF[2],groundT[4]; unsigned n; Reset(0);
    Chassis.body_state.roll=0.04f; Chassis.body_state.d_roll=-0.01f;
    Chassis.set_goal.v_set=0.5f; Chassis.set_goal.yaw_set=-0.5f; INS.Yaw=0.2f;
    LEG_Lenth_Control(); LQR_Calc(0.20f,0.20f);
    groundF[0]=Chassis.leg_situation[0].vmc.F0; groundF[1]=Chassis.leg_situation[1].vmc.F0;
    groundT[0]=Chassis.Wheel_Motor[LEFT_Wheel].wheel_T; groundT[1]=Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T;
    groundT[2]=Chassis.leg_situation[0].vmc.Tp; groundT[3]=Chassis.leg_situation[1].vmc.Tp;
    Chassis.ground.all_off_flag=1; LEG_Lenth_Control(); LQR_Calc(0.20f,0.20f);
    NEAR(Chassis.leg_situation[0].vmc.F0,groundF[0],0); NEAR(Chassis.Wheel_Motor[LEFT_Wheel].wheel_T,groundT[0],0);
#ifdef VERIFY_GROUND_BASELINE
    Baseline_LEG_Lenth_Control(); Baseline_LQR_Calc(0.20f,0.20f);
    NEAR(Chassis.leg_situation[0].vmc.F0,groundF[0],0); NEAR(Chassis.leg_situation[1].vmc.F0,groundF[1],0);
    NEAR(Chassis.Wheel_Motor[LEFT_Wheel].wheel_T,groundT[0],0); NEAR(Chassis.leg_situation[1].vmc.Tp,groundT[3],0);
#endif
    /* 只观测时，悬空标志和无效估算均不改变既有 LESO 行为。 */
    for(n=0;n<650;++n) LESO_Service(); CHECK(leso_active);
    Chassis.ground.control_enabled=1; Pose(0,0.2f,0.2f); Pose(1,0.2f,-0.1f);
    LEG_Lenth_Control(); NEAR(F_roll,0,0); NEAR(Chassis.leg_situation[0].vmc.F0,0,3e-5f);
    leg_length_ramp[0].out=Chassis.leg_situation[0].vmc.L0+0.005f;
    leg_length_ramp[1].out=Chassis.leg_situation[1].vmc.L0-0.006f;
    Chassis.leg_situation[0].vmc.d_L0=0.01f; Chassis.leg_situation[1].vmc.d_L0=-0.02f;
    LEG_Lenth_Control();
    NEAR(Chassis.leg_situation[0].vmc.F0,7.0f,4e-5f);
    NEAR(Chassis.leg_situation[1].vmc.F0,-6.0f,4e-5f);
    LQR_Calc(0.20f,0.20f); NEAR(Chassis.Wheel_Motor[LEFT_Wheel].wheel_T,0,0); NEAR(Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T,0,0);
    for(n=2;n<4;++n) { float expect=u[4]*Fitting_K[n][4]+u[5]*Fitting_K[n][5]+u[6]*Fitting_K[n][6]+u[7]*Fitting_K[n][7];
      NEAR(Chassis.leg_situation[n-2].vmc.Tp,expect,2e-5f);
    }
    Chassis.Wheel_Motor[LEFT_Wheel].wheel_T=3; Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T=-2; Fresh();
    Chassis_CanTransimit(); CHECK(sent_current[0]==0 && sent_current[1]==0);
    Chassis.ground.all_off_flag=0; LEG_Lenth_Control(); CHECK(F_roll!=0);
    CHECK(Chassis.leg_situation[0].vmc.F0>50);
    Chassis.ground.valid[0]=Chassis.ground.valid[1]=1;
    Chassis.ground.off_flag[0]=Chassis.ground.off_flag[1]=0;
    LESO_Service(); for(n=0;n<650;++n) LESO_Service(); CHECK(leso_active);
    Chassis.ground.off_flag[0]=1; LESO_Service(); CHECK(!leso_active); NEAR(leso_dbg_comp_leg,0,0);
    Chassis.ground.off_flag[0]=0; Chassis.ground.valid[1]=0; LESO_Service(); CHECK(!leso_active);
    Chassis.ground.valid[1]=1; LESO_Service(); NEAR(stable_t,0.001f,1e-7f);
    NEAR(leso.dh[0],0,1e-6f); for(n=0;n<490;++n) LESO_Service(); CHECK(!leso_active);
    for(n=0;n<160;++n) LESO_Service(); CHECK(leso_active);
}
static void PublishFeedback(float left_ratio,float right_ratio)
{
    float t[2][2]; unsigned leg; unsigned back[2]={LEFT_BACK_id,RIGHT_BACK_id},front[2]={LEFT_FRONT_id,RIGHT_FRONT_id};
    Loads(left_ratio,right_ratio,t);
    for(leg=0;leg<2;++leg) {
        float direction=leg?-1:1; vmc_leg_t *v=&Chassis.leg_situation[leg].vmc;
        Chassis.Joint_Motor[back[leg]].Data.pos=(v->phi1-pi*0.5f)*direction;
        Chassis.Joint_Motor[front[leg]].Data.pos=(v->phi4-pi*0.5f)*direction;
        Chassis.Joint_Motor[back[leg]].Data.vel=Chassis.Joint_Motor[front[leg]].Data.vel=0;
        Chassis.Joint_Motor[back[leg]].Data.tor=t[leg][0]; Chassis.Joint_Motor[front[leg]].Data.tor=t[leg][1];
        Chassis.Joint_Motor[back[leg]].Data.state=Chassis.Joint_Motor[front[leg]].Data.state=1;
    }
    Fresh();
}
static void SnapshotAndLanding(void)
{
    unsigned n; Reset(1); PublishFeedback(1,1);
    inject_after_unlock=2; chassis_feedback_update();
    NEAR(Chassis.ground.Fn_ratio[0],1,2e-6f); NEAR(Chassis.body_state.theta,0,0);
    CHECK(INS.Pitch==0.24f); inject_after_unlock=0; INS.Pitch=0; INS.MotionAccel_n[2]=0;
    Reset(1); PublishFeedback(0,0);
    for(n=0;n<80;++n) { tick_ms++; Fresh(); chassis_feedback_update(); }
    CHECK(Chassis.ground.all_off_flag); Chassis.body_state.x=4; Chassis.chassis_flag.start_situate_flag=1;
    Fresh(); chassis_feedback_update(); NEAR(Chassis.body_state.x,0,0); CHECK(!Chassis.chassis_flag.start_situate_flag);
    INS.Yaw=0.9f; Chassis.set_goal.yaw_set=-2; PublishFeedback(1,0);
    for(n=0;n<150 && Chassis.ground.all_off_flag;++n) { tick_ms++; Fresh(); chassis_feedback_update(); }
    CHECK(!Chassis.ground.all_off_flag && Chassis.ground.off_flag[RIGHT_Leg]);
    NEAR(Chassis.set_goal.yaw_set,0.9f,0); NEAR(Chassis.body_state.x,0,0);
    Chassis.Joint_Motor[LEFT_BACK_id].Data.state=0; Fresh(); chassis_feedback_update(); CHECK(!Chassis.ground.valid[LEFT_Leg]);
}
/* 检验 OFFLINE 能采样而输出始终为零；输入是合成反馈，不代表硬件接触验证。 */
static void OfflineObservation(void)
{
    unsigned n; Reset(0); PublishFeedback(0,0);
    Chassis.chassis_enable=OFFLINE; Chassis.chassis_mode=ZERO_FORCE;
    for(n=0;n<80;++n) { Fresh(); chassis_feedback_update(); }
    CHECK(!Chassis.ground.valid[0] && !Chassis.ground.all_off_flag);
    ground_offline_debug_enable=1U;
    for(n=0;n<80;++n) { tick_ms++; Fresh(); chassis_feedback_update(); }
    CHECK(Chassis.ground.valid[0] && Chassis.ground.valid[1]);
    CHECK(Chassis.ground.all_off_flag && Chassis.ground.off_flag[0] && Chassis.ground.off_flag[1]);
    for(n=0;n<2;++n) {
        Chassis.Wheel_Motor[n].wheel_T=3.0f;
        Chassis.leg_situation[n].vmc.torque_set[0]=10.0f;
        Chassis.leg_situation[n].vmc.torque_set[1]=-10.0f;
    }
    Fresh(); Chassis_CanTransimit();
    CHECK(sent_current[0]==0 && sent_current[1]==0);
    for(n=0;n<4;++n) NEAR(sent_joint[n],0,0);
    /* 即便离地控制请求为 1，OFFLINE 发送仍为零。 */
    ground_control_enable=1U; Fresh(); chassis_feedback_update();
    CHECK(Chassis.ground.control_enabled); Chassis_CanTransimit();
    CHECK(sent_current[0]==0 && sent_current[1]==0);
    for(n=0;n<4;++n) NEAR(sent_joint[n],0,0);
    Chassis.all_motors_online=0;
    { float torque[2][2]={{0}}; Update(0.001f,0,torque); }
    CHECK(!Chassis.ground.valid[0] && Chassis.ground.all_off_flag);
    ground_offline_debug_enable=0U; Fresh(); chassis_feedback_update();
    CHECK(!Chassis.ground.valid[0] && !Chassis.ground.all_off_flag);
    /* 零力矩且静止时，自重项约 21.45 N，比例约 0.215，不能硬认为抱起必然置位。 */
    ground_offline_debug_enable=1U;
    for(n=0;n<4;++n) Chassis.Joint_Motor[n].Data.tor=0;
    for(n=0;n<100;++n) { tick_ms++; Fresh(); chassis_feedback_update(); }
    NEAR(Chassis.ground.Fn[0],(GROUND_LEG_MASS_KG+GROUND_WHEEL_MASS_KG)*GROUND_GRAVITY_M_S2,2e-5f);
    CHECK(Chassis.ground.Fn_ratio[0]>GROUND_OFF_RATIO && !Chassis.ground.all_off_flag);
    Chassis.Joint_Motor[LEFT_BACK_id].Data.state=0; Fresh(); chassis_feedback_update();
    CHECK(!Chassis.ground.valid[LEFT_Leg]);
    ground_offline_debug_enable=0U;
}
int main(void)
{
    InverseAndProjection(); TimingAndInvalid(); ThresholdEdgesAndReset(); FiltersAndDynamics(); ControlAndObserver(); SnapshotAndLanding(); OfflineObservation();
    printf("ground contact: %u checks, %u failures\n",checks,failures);
    return failures?1:0;
}
