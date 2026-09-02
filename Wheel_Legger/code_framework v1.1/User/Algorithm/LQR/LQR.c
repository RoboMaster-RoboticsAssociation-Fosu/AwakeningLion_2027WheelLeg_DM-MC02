#include "LQR.h"
#include "main.h"
#include "chassis_task.h"
#include "some_para.h"

extern INS_t INS;
extern Chassis_Info_Typedef Chassis;

float P[40][6] = 
{ 
		-1.3336,  -3.8343,  3.1463,  5.8912,  -1.8903,  -2.582,
    -2.1995,  -3.8713,  5.6525,  7.0062,  -4.4707,  -5.1055,
    -1.031,  1.9599,  -1.326,  -1.5828,  2.1496,  1.6167,
    -0.4223,  1.0021,  -0.73197,  -0.80233,  1.1441,  0.91669,
    -2.1584,  -30.908,  9.0403,  20.346,  -10.045,  -11.689,
    -0.38693,  -3.829,  1.5593,  1.1962,  -0.96434,  -2.019,
    -2.0086,  5.5337,  -16.413,  -5.9851,  28.233,  18.941,
    -0.382,  0.78616,  -1.8053,  -0.17966,  1.5439,  1.8292,
    -8.6169,  12.74,  13.027,  -0.66453,  -17.092,  -12.718,
    -0.90651,  1.2807,  1.6373,  0.16464,  -2.1831,  -1.6229,
    -1.3336,  3.1463,  -3.8343,  -2.582,  -1.8903,  5.8912,
    -2.1995,  5.6525,  -3.8713,  -5.1055,  -4.4707,  7.0062,
    1.031,  1.326,  -1.9599,  -1.6167,  -2.1496,  1.5828,
    0.4223,  0.73197,  -1.0021,  -0.91669,  -1.1441,  0.80233,
    -2.0086,  -16.413,  5.5337,  18.941,  28.233,  -5.9851,
    -0.382,  -1.8053,  0.78616,  1.8292,  1.5439,  -0.17966,
    -2.1584,  9.0403,  -30.908,  -11.689,  -10.045,  20.346,
    -0.38693,  1.5593,  -3.829,  -2.019,  -0.96434,  1.1962,
    -8.6169,  13.027,  12.74,  -12.718,  -17.092,  -0.66453,
    -0.90651,  1.6373,  1.2807,  -1.6229,  -2.1831,  0.16464,
    -1.5831,  12.507,  -0.13083,  -24.104,  -4.4764,  4.2764,
    -2.1125,  17.958,  -0.47204,  -34.067,  -7.1683,  6.4726,
    -0.020996,  -5.8591,  -1.8926,  8.2426,  -5.1405,  3.9294,
    0.020975,  -3.0341,  -1.0688,  4.2565,  -2.793,  2.1563,
    5.4991,  56.703,  8.4034,  -69.136,  38.838,  -19.961,
    0.67221,  5.6401,  1.229,  -7.6622,  4.1925,  -2.175,
    -0.41859,  -18.395,  -23.032,  31.299,  -64.687,  40.603,
    -0.33897,  -1.7047,  -2.0188,  1.4503,  -6.0675,  3.3084,
    -15.724,  -33.18,  15.599,  24.214,  11.766,  -19.555,
    -1.3225,  -3.4717,  1.9574,  1.9636,  1.3233,  -2.3592,
    -1.5831,  -0.13083,  12.507,  4.2764,  -4.4764,  -24.104,
    -2.1125,  -0.47204,  17.958,  6.4726,  -7.1683,  -34.067,
    0.020996,  1.8926,  5.8591,  -3.9294,  5.1405,  -8.2426,
    -0.020975,  1.0688,  3.0341,  -2.1563,  2.793,  -4.2565,
    -0.41859,  -23.032,  -18.395,  40.603,  -64.687,  31.299,
    -0.33897,  -2.0188,  -1.7047,  3.3084,  -6.0675,  1.4503,
    5.4991,  8.4034,  56.703,  -19.961,  38.838,  -69.136,
    0.67221,  1.229,  5.6401,  -2.175,  4.1925,  -7.6622,
    -15.724,  15.599,  -33.18,  -19.555,  11.766,  24.214,
    -1.3225,  1.9574,  -3.4717,  -2.3592,  1.3233,  1.9636
};

float u[10];
float Fitting_K[4][10];

void Fitting_K_Calc(float (*fitting_k)[10],float (*p)[6],float L_l,float L_r)
{
	static unsigned short int i = 0;
	static unsigned short int j = 0;
	
	for(i=0;i<=3;i++)
	{
		for(j=0;j<=9;j++)
		{
			fitting_k[i][j] = p[i*10+j][0] + p[i*10+j][1]*L_l + p[i*10+j][2]*L_r + p[i*10+j][3]*(L_l*L_l) + p[i*10+j][4]*L_l*L_r + p[i*10+j][5]*(L_r*L_r);
		}
	}
}

void LQR_Calc(float L_l,float L_r)
{
    Fitting_K_Calc(Fitting_K,P,L_l,L_r);
    static float T[4];
    uint8_t i;

    u[0] = 0.0f                             -           Chassis.body_state.x;
    u[1] = Chassis.set_goal.v_set           -           Chassis.body_state.Estimate_dx;
    u[2] = Find_Min_RADIAN(INS.Yaw, Chassis.set_goal.yaw_set);
    u[3] = Chassis.set_goal.yaw_set_v       -           Chassis.body_state.d_yaw;
    u[4] = 0.0f                             -           Chassis.leg_situation[LEFT_Leg].vmc.theta;
    u[5] = 0.0F                             -           Chassis.leg_situation[LEFT_Leg].vmc.d_theta;
    u[6] = 0.0f                             -           Chassis.leg_situation[RIGHT_Leg].vmc.theta;
    u[7] = 0.0F                             -           Chassis.leg_situation[RIGHT_Leg].vmc.d_theta;
    u[8] = 0.0f                             -           Chassis.body_state.theta;
    u[9] = 0.0F                             -           Chassis.body_state.d_theta;

    mySaturate(&u[0],-X_MAX, X_MAX);
    mySaturate(&u[2],-Yaw_MAX, Yaw_MAX);
    mySaturate(&u[4],-Theta_L_MAX, Theta_L_MAX);
    mySaturate(&u[6],-Theta_R_MAX, Theta_R_MAX);
    mySaturate(&u[8],-Theta_B_MAX, Theta_B_MAX);

    for(i=0; i<4; i++)
    {
        T[i] = u[0]*Fitting_K[i][0] + u[1]*Fitting_K[i][1]
             - u[2]*Fitting_K[i][2] + u[3]*Fitting_K[i][3]     // 注意这个 - 号是拟合约定，别改
             + u[4]*Fitting_K[i][4] + u[5]*Fitting_K[i][5]
             + u[6]*Fitting_K[i][6] + u[7]*Fitting_K[i][7]
             + u[8]*Fitting_K[i][8] + u[9]*Fitting_K[i][9];
    }

    Chassis.Wheel_Motor[LEFT_Wheel].wheel_T   = T[0];
    Chassis.Wheel_Motor[RIGHT_Wheel].wheel_T  = T[1];
    Chassis.leg_situation[LEFT_Leg].vmc.Tp    = T[2];
    Chassis.leg_situation[RIGHT_Leg].vmc.Tp   = T[3];
}