#ifndef __ANGLE_H_
#define __ANGLE_H_

#include "zf_common_headfile.h"

#define AngleX_bias -4.1
#define AngleY_bias -1.6
typedef struct {
	float angle;//姿态角度
	float bias;//陀螺仪角速度偏差
	
}State;
//卡尔曼滤波器
typedef struct
{
	State state;//状态变量
	float P[2][2];//协方差矩阵
	float Q_angle;//角度过程噪声协方差
	float Q_bias;//偏差噪声协方差
	float R_mea;//观测噪声协方差
	float dt;//时间步长

}KalmanFilter;

extern float AngleX;
extern float AngleY;
extern float AngleZ;

extern float AngleZ_2;//用于返回退出使用的

extern float AngleZ_bias;
void GainAngleSpeed();
void GyroscopeInit();
void GetAngleData();
#endif