#ifndef _ROBOTARM_H_
#define _ROBOTARM_H_
#include "zf_common_headfile.h"

#define L1 17.0	//��е�����˳���
#define L2 16.1
#define H  8.5//��е�۸߶�

#define Angle_Base_Bias (12.0/180.0)*3.1415926//��е�ۻ�����ϵ��ȫ������ϵƫ��Ƕ�
#define Base_X_Bias -13.035
#define Base_Y_Bias -11.465
#define Robot_End_Bias 2 //��е��ĩ�˺�������ƫ��

#define angle1_bias 90//��е�ۻ����Ƕ�
#define angle2_bias 135//����е�
#define angle3_bias 180
#define angle4_bias 135



//��е��ĩ�˸����ؽڽǶ�
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
extern uint16 RobotTime;//��е��ʱ���������
extern RobotPosition targetPosition[7];//��е�۸��ؽ�Ŀ��Ƕ�
extern RobotPosition LastPosition;//��е�۸��ؽڵ�ǰ�Ƕ�
extern uint8 RobotStartFlag;
extern uint8 RobotArmControlFlag;//��е��������־λ
extern uint8 RobotArmTargetPosition;//��е��Ŀ��������±�־λ


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