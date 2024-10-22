#ifndef __ANGLE_H_
#define __ANGLE_H_

#include "zf_common_headfile.h"

#define AngleX_bias -4.1
#define AngleY_bias -1.6
typedef struct {
	float angle;//��̬�Ƕ�
	float bias;//�����ǽ��ٶ�ƫ��
	
}State;
//�������˲���
typedef struct
{
	State state;//״̬����
	float P[2][2];//Э�������
	float Q_angle;//�Ƕȹ�������Э����
	float Q_bias;//ƫ������Э����
	float R_mea;//�۲�����Э����
	float dt;//ʱ�䲽��

}KalmanFilter;

extern float AngleX;
extern float AngleY;
extern float AngleZ;

extern float AngleZ_2;//���ڷ����˳�ʹ�õ�

extern float AngleZ_bias;
void GainAngleSpeed();
void GyroscopeInit();
void GetAngleData();
#endif