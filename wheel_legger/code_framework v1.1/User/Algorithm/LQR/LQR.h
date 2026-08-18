#ifndef __LQR_H
#define __LQR_H

#include "main.h"
#include "INS_task.h"

#define X_MAX       6.00f
#define Yaw_MAX     1.25f
#define Theta_L_MAX 0.45f
#define Theta_R_MAX 0.45f
#define Theta_B_MAX 0.25f

extern void Fitting_K_Calc(float (*fitting_k)[10],float (*p)[6],float L_l,float L_r);
extern void LQR_Calc(float L_l,float L_r);

extern float Fitting_K[4][10];
extern float P[40][6];
extern float u[10];


#endif


