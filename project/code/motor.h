#ifndef _MOTOR_H
#define _MOTOR_H

#include "zf_common_headfile.h"

#define ENFR (B10) //右前轮改为B10
#define ENFL (B14)
#define ENBR (C9)
#define ENBL (C7)
#define PWMFR (PWM2_MODULE2_CHA_C10)//(PWM2_MODULE3_CHB_D3) //右前
#define PWMFL (PWM2_MODULE2_CHB_C11)  
#define PWMBR (PWM2_MODULE1_CHA_C8)
#define PWMBL (PWM2_MODULE0_CHA_C6)

#define ENCOEDER_FR_DIR		                  (QTIMER2_ENCODER1)                     // 带方向编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER2
#define ENCOEDER_FR_DIR_PULSE               (QTIMER2_ENCODER1_CH1_C3)              // PULSE 对应的引脚
#define ENCOEDER_FR_DIR_DIR                 (QTIMER2_ENCODER1_CH2_C4) 						//DIR 对应引脚

#define ENCOEDER_FL_DIR		                  (QTIMER2_ENCODER2) 					                   
#define ENCOEDER_FL_DIR_PULSE               (QTIMER2_ENCODER2_CH1_C5)             
#define ENCOEDER_FL_DIR_DIR                 (QTIMER2_ENCODER2_CH2_C25) 				

#define ENCOEDER_BR_DIR		                  (QTIMER1_ENCODER2)                   
#define ENCOEDER_BR_DIR_PULSE               (QTIMER1_ENCODER2_CH1_C2)             
#define ENCOEDER_BR_DIR_DIR                 (QTIMER1_ENCODER2_CH2_C24) 					

#define ENCOEDER_BL_DIR		                  (QTIMER1_ENCODER1)                    
#define ENCOEDER_BL_DIR_PULSE               (QTIMER1_ENCODER1_CH1_C0)           
#define ENCOEDER_BL_DIR_DIR                 (QTIMER1_ENCODER1_CH2_C1) 		 		

#define A_pulse_x 0.851//单位cm/脉冲
#define CarR 100//车身机械中点到轮子的半径
extern float CarSpeedX;
extern float CarSpeedY;
extern float CarSpeedZ;

extern float encoder[4];
extern float P_encoder[4]; //累计位移

void motor_init(void);
void Read_Encoder(void);
void encoder_init(void);
void motor_control(float pid_motor[4]);
void tran_speed(float Vx,float Vy,float Vz);
void Encoder_odometer();
void Stop_Car(float Tar_angle_Z);
void car_stop();
void SpeedForwardSolution();
void Encoder_odometer(float P_encoder[4],float * out_distanceX,float * out_distanceY);
#endif