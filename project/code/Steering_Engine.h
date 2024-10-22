#ifndef _STEERING_ENGINE_H_
#define _STEERING_ENGINE_H_
#include "zf_common_headfile.h"
#define MAX_Angle 2500//舵机角度最大时，PWM值 占空比为2.5ms
#define Mid_Angle 1500//舵机中值PWM值，1.5ms
#define Min_Angle 500//舵机最小值PWM值，0.5ms

#define Engine_Pin_1 PWM1_MODULE3_CHB_D1//PWM2_MODULE2_CHA_C10 
#define Engine_Pin_2 PWM4_MODULE2_CHA_C30
#define Engine_Pin_3 PWM1_MODULE3_CHA_D0  
#define Engine_Pin_4 PWM2_MODULE3_CHB_D3
#define Engine_Pin_5 PWM4_MODULE3_CHA_C31 
void Steering_Engine_Start();
int AngleTurnPwm1(float angle);
int AngleTurnPwm2(float angle);
void Set_Angle(unsigned char num,int PWMvalue);

#endif