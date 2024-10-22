#ifndef _IMU_H
#define _IMU_H

#include "zf_common_headfile.h"

extern float Acc_x,Acc_y,Gyro_z,Angle_z;
extern float fil_Acc_x,fil_Acc_y,fil_Gyro_z;
extern float Angle_z,Angle_Z;
extern float Last_tar_angle;
extern double A_acc[2],V_acc[2],X_acc[2];
extern int gyro_i;
extern int moto_flag;
extern int Angle;
void Get_accdata(void);
void Get_gyro(void);
void Icm20602_odometer(float Acc_x,float Acc_y);
void Get_angle(void);
void Imu_Init(void);


//void ICM20602_newValues(void);
//float Turn_optimize(float Tar_angle,float Last_tar_angle);

//typedef struct
//{
//	float gyro_x;//陀螺仪参数
//	float gyro_y;
//	float gyro_z;
//	float acc_x;//加速度参数
//	float acc_y;
//	float acc_z;
//}imu_param_t;//传感器参数

//typedef struct
//{
//	float q0;
//	float q1;
//	float q2;
//	float q3;
//}quater_param_t;//四元数结构体

//typedef struct
//{
//	float pitch;//俯仰角
//	float roll;//翻滚角
//	float yaw;//偏航角
//}euler_param_t;//欧拉角参数

//typedef struct {
//    float Xdata;
//    float Ydata;
//    float Zdata;
//} gyro_param_t;

//extern quater_param_t Q_init;
//extern euler_param_t eulerAngle;

//float fast_sqrt(float x);
//void gyroOffset_init(void);
//void IMU_getValues(void);
//void IMU_AHRSupdate(float gx,float gy,float gz,float ax,float ay,float az);
//void IMU_getEulerianAngles(void);

#endif