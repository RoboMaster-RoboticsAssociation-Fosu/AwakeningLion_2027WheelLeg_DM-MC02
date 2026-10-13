#include "VMC_calc.h"
#include <math.h>

void VMC_init(vmc_leg_t *vmc)//给杆长赋值
{
	vmc->l5=0.0f;//AE长度 //单位为m
	vmc->l1=0.208f;//单位为m
	vmc->l2=0.25212f;//单位为m
	vmc->l3=0.25212f;//2单位为m
	vmc->l4=0.208f;//单位为m
}


//void VMC_calc_1(vmc_leg_t *vmc,INS_t *ins,float dt)//计算theta和d_theta给lqr用，同时也计算腿长L0
//{		
//		static float PitchR=0.0f;
//	  static float PithGyroR=0.0f;
//	  PitchR=ins->Pitch;
//	  PithGyroR=ins->Gyro[1];
//	
//	  vmc->YD = vmc->l4*arm_sin_f32(vmc->phi4);//D的y坐标
//	  vmc->YB = vmc->l1*arm_sin_f32(vmc->phi1);//B的y坐标
//	  vmc->XD = vmc->l5 + vmc->l4*arm_cos_f32(vmc->phi4);//D的x坐标
//	  vmc->XB = vmc->l1*arm_cos_f32(vmc->phi1); //B的x坐标
//			
//		vmc->lBD = sqrt((vmc->XD - vmc->XB)*(vmc->XD - vmc->XB) + (vmc->YD -vmc-> YB)*(vmc->YD - vmc->YB));
//	
//	  vmc->A0 = 2*vmc->l2*(vmc->XD - vmc->XB);
//		vmc->B0 = 2*vmc->l2*(vmc->YD - vmc->YB);
//		vmc->C0 = vmc->l2*vmc->l2 + vmc->lBD*vmc->lBD - vmc->l3*vmc->l3;
//		vmc->phi2 = 2*atan2f((vmc->B0 + sqrt(vmc->A0*vmc->A0 + vmc->B0*vmc->B0 - vmc->C0*vmc->C0)),vmc->A0 + vmc->C0);			
//	  vmc->phi3 = atan2f(vmc->YB-vmc->YD+vmc->l2*arm_sin_f32(vmc->phi2),vmc->XB-vmc->XD+vmc->l2*arm_cos_f32(vmc->phi2));
//	  //C点直角坐标
//		vmc->XC = vmc->l1*arm_cos_f32(vmc->phi1) + vmc->l2*arm_cos_f32(vmc->phi2);//xc = xb+l2*cos(phi2);
//		vmc->YC = vmc->l1*arm_sin_f32(vmc->phi1) + vmc->l2*arm_sin_f32(vmc->phi2);//yc = yb+l2*sin(phi2);
//		//C点极坐标
//		vmc->L0 = sqrt((vmc->XC - vmc->l5/2.0f)*(vmc->XC - vmc->l5/2.0f) + vmc->YC*vmc->YC);
//		
//	  vmc->phi0 = atan2f(vmc->YC,(vmc->XC - vmc->l5/2.0f));//phi0用于计算lqr需要的theta		
//	  vmc->alpha=pi/2.0f-vmc->phi0 ;
//		
//		if(vmc->first_flag==0)
//		{
//			vmc->last_phi0=vmc->phi0 ;
//			vmc->first_flag=1;
//		}
//		vmc->d_phi0=(vmc->phi0-vmc->last_phi0)/dt;//计算phi0变化率，d_phi0用于计算lqr需要的d_theta
//		vmc->d_alpha=0.0f-vmc->d_phi0 ;
//		
//		
//		vmc->theta=-(pi/2.0f-PitchR-vmc->phi0);//得到状态变量1
//		vmc->d_theta=(-PithGyroR-vmc->d_phi0);//得到状态变量2
//		
//		vmc->last_phi0=vmc->phi0 ;
//    
//		vmc->d_L0=(vmc->L0-vmc->last_L0)/dt;//腿长L0的一阶导数
//    vmc->dd_L0=(vmc->d_L0-vmc->last_d_L0)/dt;//腿长L0的二阶导数
//		
//		vmc->last_d_L0=vmc->d_L0;
//		vmc->last_L0=vmc->L0;
//		
//		vmc->dd_theta=(vmc->d_theta-vmc->last_d_theta)/dt;
//		vmc->last_d_theta=vmc->d_theta;
//}
void VMC_calc_1(vmc_leg_t *vmc,INS_t *ins,float dt)//计算theta和d_theta给lqr用，同时也计算腿长L0
{		
		static float PitchR=0.0f;
	  static float PithGyroR=0.0f;
	  PitchR=ins->Pitch;
	  PithGyroR=-ins->Gyro[1];
	
	  vmc->YD = vmc->l4*arm_sin_f32(vmc->phi4);//D的y坐标
	  vmc->YB = vmc->l1*arm_sin_f32(vmc->phi1);//B的y坐标
	  vmc->XD = vmc->l5 + vmc->l4*arm_cos_f32(vmc->phi4);//D的x坐标
	  vmc->XB = vmc->l1*arm_cos_f32(vmc->phi1); //B的x坐标
			
		vmc->lBD = sqrt((vmc->XD - vmc->XB)*(vmc->XD - vmc->XB) + (vmc->YD -vmc-> YB)*(vmc->YD - vmc->YB));
	
	  vmc->A0 = 2*vmc->l2*(vmc->XD - vmc->XB);
		vmc->B0 = 2*vmc->l2*(vmc->YD - vmc->YB);
		vmc->C0 = vmc->l2*vmc->l2 + vmc->lBD*vmc->lBD - vmc->l3*vmc->l3;
		vmc->phi2 = 2*atan2f((vmc->B0 + sqrt(vmc->A0*vmc->A0 + vmc->B0*vmc->B0 - vmc->C0*vmc->C0)),vmc->A0 + vmc->C0);			
	  vmc->phi3 = atan2f(vmc->YB-vmc->YD+vmc->l2*arm_sin_f32(vmc->phi2),vmc->XB-vmc->XD+vmc->l2*arm_cos_f32(vmc->phi2));
	  //C点直角坐标
		vmc->XC = vmc->l1*arm_cos_f32(vmc->phi1) + vmc->l2*arm_cos_f32(vmc->phi2);//xc = xb+l2*cos(phi2);
		vmc->YC = vmc->l1*arm_sin_f32(vmc->phi1) + vmc->l2*arm_sin_f32(vmc->phi2);//yc = yb+l2*sin(phi2);
		//C点极坐标
		vmc->L0 = sqrt((vmc->XC - vmc->l5/2.0f)*(vmc->XC - vmc->l5/2.0f) + vmc->YC*vmc->YC);
		
	  vmc->phi0 = atan2f(vmc->YC,(vmc->XC - vmc->l5/2.0f));//phi0用于计算lqr需要的theta		
	  vmc->alpha=pi/2.0f-vmc->phi0 ;
		
		
		
		
		
		vmc->A1 = (vmc->l1*vmc->d_phi1*arm_sin_f32(vmc->phi1 - vmc->phi3)
         + vmc->l4*vmc->d_phi4*arm_sin_f32(vmc->phi3 - vmc->phi4))
         / arm_sin_f32(vmc->phi3 - vmc->phi2);
		float d_x_b = -vmc->l1*vmc->d_phi1*arm_sin_f32(vmc->phi1);
		float d_y_b =  vmc->l1*vmc->d_phi1*arm_cos_f32(vmc->phi1);
		float x_c = vmc->XC - vmc->l5/2.0f;
		float y_c = vmc->YC;

		vmc->d_L0   = (y_c*(d_y_b + vmc->A1*arm_cos_f32(vmc->phi2))
             + x_c*(d_x_b - vmc->A1*arm_sin_f32(vmc->phi2))) / vmc->L0;
		vmc->d_phi0 = (x_c*(d_y_b + vmc->A1*arm_cos_f32(vmc->phi2))
             - y_c*(d_x_b - vmc->A1*arm_sin_f32(vmc->phi2))) / (vmc->L0*vmc->L0);

		vmc->theta=-(pi/2.0f-PitchR-vmc->phi0);//得到状态变量1
		vmc->d_theta = ( PithGyroR + vmc->d_phi0 );
		
		vmc->d_alpha=0.0f-vmc->d_phi0 ;
    
		
    vmc->dd_L0=(vmc->d_L0-vmc->last_d_L0)/dt;//腿长L0的二阶导数
		
		vmc->last_d_L0=vmc->d_L0;
		
		
		vmc->dd_theta=(vmc->d_theta-vmc->last_d_theta)/dt;
		vmc->last_d_theta=vmc->d_theta;
}


/* F0 与 dL0、Tp 与 dphi0 共轭；正向映射与反馈逆解共用本拍几何。 */
static void VMC_Jacobian_Calc(vmc_leg_t *vmc)
{
    float denominator = arm_sin_f32(vmc->phi3 - vmc->phi2);
    vmc->j11 = vmc->l1 * arm_sin_f32(vmc->phi0 - vmc->phi3)
             * arm_sin_f32(vmc->phi1 - vmc->phi2) / denominator;
    vmc->j12 = vmc->l1 * arm_cos_f32(vmc->phi0 - vmc->phi3)
             * arm_sin_f32(vmc->phi1 - vmc->phi2) / (vmc->L0 * denominator);
    vmc->j21 = vmc->l4 * arm_sin_f32(vmc->phi0 - vmc->phi2)
             * arm_sin_f32(vmc->phi3 - vmc->phi4) / denominator;
    vmc->j22 = vmc->l4 * arm_cos_f32(vmc->phi0 - vmc->phi2)
             * arm_sin_f32(vmc->phi3 - vmc->phi4) / (vmc->L0 * denominator);
}

void VMC_calc_2(vmc_leg_t *vmc)
{
    VMC_Jacobian_Calc(vmc);
    vmc->torque_set[0] = vmc->j11 * vmc->F0 + vmc->j12 * vmc->Tp; /* 后关节 phi1，N*m。 */
    vmc->torque_set[1] = vmc->j21 * vmc->F0 + vmc->j22 * vmc->Tp; /* 前关节 phi4，N*m。 */
}

/* 输入为消除电机镜像后的反馈力矩，N*m；输出只用于检测，不改 F0/Tp 指令。 */
uint8_t VMC_Force_Calc(vmc_leg_t *vmc, float torque_back, float torque_front,
                       float *F0_feedback, float *Tp_feedback)
{
    float det, force, torque;
    if (!isfinite(torque_back) || !isfinite(torque_front) ||
        !isfinite(vmc->L0) || vmc->L0 < 1.0e-4f ||
        !isfinite(vmc->phi0) || !isfinite(vmc->phi1) || !isfinite(vmc->phi2) ||
        !isfinite(vmc->phi3) || !isfinite(vmc->phi4) ||
        fabsf(arm_sin_f32(vmc->phi3 - vmc->phi2)) < 1.0e-6f)
    {
        return 0U;
    }
    VMC_Jacobian_Calc(vmc);
    det = vmc->j11 * vmc->j22 - vmc->j12 * vmc->j21;
    if (!isfinite(det) || fabsf(det) < 1.0e-6f) return 0U;
    /* [T_back, T_front] = J*[F0, Tp]；固定 2x2 直接逆解，单位为 N、N*m。 */
    force = (vmc->j22 * torque_back - vmc->j12 * torque_front) / det;
    torque = (vmc->j11 * torque_front - vmc->j21 * torque_back) / det;
    if (!isfinite(force) || !isfinite(torque)) return 0U;
    *F0_feedback = force;
    *Tp_feedback = torque;
    return 1U;
}

float LQR_K_calc(float *coe,float len)
{
   
  return coe[0]*len*len*len+coe[1]*len*len+coe[2]*len+coe[3];
}


