#ifndef _ROBOTARM_H_
#define _ROBOTARM_H_
#include "zf_common_headfile.h"

#define L1 17.0	//机械臂连杆长度
#define L2 16.1
#define H  8.5//机械臂高度

#define Angle_Base_Bias (12.0/180.0)*3.1415926//机械臂基坐标系和全局坐标系偏差角度
#define Base_X_Bias -13.035
#define Base_Y_Bias -11.465
#define Robot_End_Bias 2 //机械臂末端和中轴线偏差

#define angle1_bias 90//机械臂基础角度
#define angle2_bias 135//舵机中点
#define angle3_bias 180
#define angle4_bias 135



//机械臂末端各个关节角度
typedef struct 
{
	float angle1;
	float angle2;
	float angle3;
	float angle4;
}RobotPosition; 

extern float RobotAngle1;
extern float RobotAngle2;
extern float RobotAngle3;
extern uint16 RobotTime;//机械臂时间坐标计数
extern RobotPosition targetPosition[7];//机械臂各关节目标角度
extern RobotPosition LastPosition;//机械臂各关节当前角度
extern uint8 RobotStartFlag;
extern uint8 RobotArmControlFlag;//机械臂启动标志位
extern uint8 RobotArmTargetPosition;//机械臂目标坐标更新标志位


void RobotArmInit();
void InverseKinematics(float x ,float y ,float z);
void LeftCameraTurnPosition(uint16 camera_x,uint16 camera_y,float *out_x,float *out_y);
void RightCameraTurnPosition(uint16 camera_x,uint16 camera_y,int *out_x,int *out_y);
void RobotContro(float angle1,float angle2,float angle3,float angle4);
void RobotPickUpCard();
void ElectromagnetEnable();
void ElectromagnetDisable();
void BaseCoordinates_Turn_RobotCoordinates(float *x,float *y,uint16 *z);
#endif