#ifndef __SOME_PARA__
#define __SOME_PARA__

#include "main.h"
#include "user_lib.h"



//角度转弧度
#define Ang_PI 0.01745329f
//弧度转角度
#define PI_Ang 57.2957805f

#define Gear_Ratio 268.0f/17.0f
#define wheel_R   0.052f //轮子半径(m)
#define L_wheel  0.488f //两个驱动轮之间距离(m)



extern float Find_Min_RADIAN(float measure, float ref);
extern void mySaturate(float *in,float min,float max);

#endif
