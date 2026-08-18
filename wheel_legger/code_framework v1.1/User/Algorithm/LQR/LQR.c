#include "LQR.h"
#include "main.h"
#include "chassis_task.h"

extern INS_t INS;
extern Chassis_Info_Typedef Chassis;

float P[40][6] = 
{ 
		 -0.8773,  -6.6633,  5.637,  10.728,  -4.8341,  -5.0894       ,
     -2.7445,  -14.28,  16.342,  27.193,  -18.672,  -13.913       ,
     -10.73,  45.182,  -11.792,  -57.676,  27.376,  13.344        ,
     -1.9148,  8.6392,  -3.4349,  -9.8042,  6.0135,  4.3875       ,
     -7.9236,  -89.163,  9.3534,  111.35,  -10.234,  -11.247      ,
     -0.44683,  -3.3359,  1.2689,  -0.012558,  0.73617,  -1.6261  ,
     -3.391,  7.378,  -6.0813,  -7.0378,  18.429,  3.4081         ,
     -0.22929,  -0.35612,  0.043282,  2.3568,  -4.8036,  1.0387   ,
     -23.316,  22.933,  38.731,  28.636,  -46.138,  -42.099       ,
     -3.3543,  2.9314,  7.3208,  4.513,  -8.7168,  -7.9397        ,
     -0.8773,  5.637,  -6.6633,  -5.0894,  -4.8341,  10.728       ,
     -2.7445,  16.342,  -14.28,  -13.913,  -18.672,  27.193       ,
     10.73,  11.792,  -45.182,  -13.344,  -27.376,  57.676        ,
     1.9148,  3.4349,  -8.6392,  -4.3875,  -6.0135,  9.8042       ,
     -3.391,  -6.0813,  7.378,  3.4081,  18.429,  -7.0378         ,
     -0.22929,  0.043282,  -0.35612,  1.0387,  -4.8036,  2.3568   ,
     -7.9236,  9.3534,  -89.163,  -11.247,  -10.234,  111.35      ,
     -0.44683,  1.2689,  -3.3359,  -1.6261,  0.73617,  -0.012558  ,
     -23.316,  38.731,  22.933,  -42.099,  -46.138,  28.636       ,
     -3.3543,  7.3208,  2.9314,  -7.9397,  -8.7168,  4.513        ,
     2.2009,  4.5389,  -7.17,  -23.108,  15.766,  5.3039          ,
     6.3146,  6.6661,  -21.555,  -53.654,  50.562,  14.199        ,
     -7.17,  -91.059,  -39.557,  175.63,  -35.61,  63.877         ,
     -1.0206,  -21.23,  -8.4847,  38.867,  -11.787,  13.746       ,
     55.526,  -58.474,  9.9243,  4.1003,  42.061,  -22.235        ,
     2.1758,  -1.856,  -0.79487,  0.44382,  3.6947,  0.12407      ,
     -1.2983,  -37.631,  -15.034,  58.735,  -23.958,  -0.74682    ,
     -0.23695,  -0.46652,  2.3683,  -3.0126,  2.2552,  -4.8017    ,
     -30.778,  -238.41,  65.697,  277.13,  55.174,  -90.17        ,
     -3.0219,  -29.845,  7.8708,  30.549,  11.901,  -11.313       ,
     2.2009,  -7.17,  4.5389,  5.3039,  15.766,  -23.108          ,
     6.3146,  -21.555,  6.6661,  14.199,  50.562,  -53.654        ,
     7.17,  39.557,  91.059,  -63.877,  35.61,  -175.63           ,
     1.0206,  8.4847,  21.23,  -13.746,  11.787,  -38.867         ,
     -1.2983,  -15.034,  -37.631,  -0.74682,  -23.958,  58.735    ,
     -0.23695,  2.3683,  -0.46652,  -4.8017,  2.2552,  -3.0126    ,
     55.526,  9.9243,  -58.474,  -22.235,  42.061,  4.1003        ,
     2.1758,  -0.79487,  -1.856,  0.12407,  3.6947,  0.44382      ,
     -30.778,  65.697,  -238.41,  -90.17,  55.174,  277.13        ,
     -3.0219,  7.8708,  -29.845,  -11.313,  11.901,  30.549
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

    u[0] = mySaturate(                       0.0f -        Chassis.body_state.x,-X_MAX, X_MAX);
    u[1] =   Chassis.set_goal.v_set -       Chassis.body_state.dx;
    u[2] = mySaturate(Find_Min_RADIAN(INS.Yaw, Chassis.set_goal.yaw_set),-Yaw_MAX, Yaw_MAX);
    u[3] = Chassis.set_goal.yaw_set_v -     Chassis.body_state.d_yaw;
    u[4] = mySaturate(                       0.0f - Chassis.leg_situation[LEFT_Leg].vmc.theta, -Theta_L_MAX, Theta_L_MAX);
    u[5] =                         0 - Chassis.leg_situation[LEFT_Leg].vmc.d_theta;
    u[6] = mySaturate(                       0.0f - Chassis.leg_situation[RIGHT_Leg].vmc.theta, -Theta_R_MAX, Theta_R_MAX);
    u[7] =                         0 - Chassis.leg_situation[RIGHT_Leg].vmc.d_theta;
    u[8] = mySaturate(                       0 -    Chassis.body_state.theta, -Theta_B_MAX, Theta_B_MAX);
    u[9] =                         0 - Chassis.body_state.d_theta;

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