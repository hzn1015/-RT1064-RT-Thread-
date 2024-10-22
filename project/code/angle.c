#include "angle.h"
float AngleSpeed_x=0;
float AngleSpeed_y=0;
float AngleSpeed_z=0;

float AngleX=0;
float AngleY=0;
float AngleZ=0;
float AngleZ_2=0;//���ڷ����˳�ʹ�õ�
float AngleSpeedX_bias=0;
float AngleSpeedY_bias=0;
float AngleSpeedZ_bias=0;

KalmanFilter Kx,Ky ;//���忨�����˲���
/**********��ȡ���ٶ�**************/
void GainAngleSpeed()
{
	float a=0.5;

		imu963ra_get_gyro();
		imu963ra_gyro_x=imu963ra_gyro_transition(imu963ra_gyro_x);
		imu963ra_gyro_y=imu963ra_gyro_transition(imu963ra_gyro_y);
		imu963ra_gyro_z=imu963ra_gyro_transition(imu963ra_gyro_z);
		AngleSpeed_x=a*imu963ra_gyro_x+(1-a)*AngleSpeed_x;
		AngleSpeed_y=a*imu963ra_gyro_y+(1-a)*AngleSpeed_y;
		AngleSpeed_z=0.3*imu963ra_gyro_z+(1-0.3)*AngleSpeed_z;
		

}
/**********�������˲�����Ƕ�*************/
//ע�⣬ƫ����ֻ��ͨ�����ٶȻ��ֵõ������ٶȼ��޷���ȡƫ����


/**********��ʼ���������˲���*************/
void KalmanInit(KalmanFilter *Kf,float Q_angle,float Q_bias,float R_mea,float dt)
{
	Kf->state.angle=0;
	Kf->state.bias=0;
	Kf->P[0][0]=0;
	Kf->P[0][1]=0;
	Kf->P[1][0]=0;
	Kf->P[1][1]=0;
	
	Kf->Q_angle=Q_angle;
	Kf->Q_bias=Q_bias;
	Kf->R_mea=R_mea;
	
	
	Kf->dt=dt;

}

/************�������˲��㷨****************/

float Kalman_filter(KalmanFilter *Kf,float newAngle,float new_gyro)
{
	float err,S;
	float K[2];//����������
	//�������
	Kf->state.angle=Kf->state.angle+Kf->dt*(new_gyro-Kf->state.bias);

	//Э����������
	Kf->P[0][0] += Kf->dt * (Kf->dt*Kf->P[1][1] - Kf->P[0][1]
                             + Kf->P[1][0] )+ Kf->Q_angle;
  Kf->P[0][1] -= Kf->dt * Kf->P[1][1];
  Kf->P[1][0] -= Kf->dt * Kf->P[1][1];
  Kf->P[1][1] += Kf->Q_bias;
	
	//���¿���������
	err=newAngle-Kf->state.angle;//����ƫ���������
	
	S=Kf->P[0][0]+Kf->R_mea;
	
	K[0]=Kf->P[0][0]/S;
	K[1]=Kf->P[1][0]/S;
	
	//����״̬����
	Kf->state.angle=Kf->state.angle+K[0]*err;
	Kf->state.bias=Kf->state.bias+K[1]*err;
	
	//����Э�������
	Kf->P[0][0] -= K[0] * Kf->P[0][0];
  Kf->P[0][1] -= K[0] * Kf->P[0][1];
  Kf->P[1][0] -= K[1] * Kf->P[0][0];
  Kf->P[1][1] -= K[1] * Kf->P[0][1];
	
	return Kf->state.angle;//���ؽǶ�
}

/********�����ǳ�ʼ��**************/
void GyroscopeInit()
{
	imu963ra_init();//��ʼ��������

	for(int i=0;i<30000;i++)
	{
		imu963ra_get_gyro();
		imu963ra_gyro_x=imu963ra_gyro_transition(imu963ra_gyro_x);
		imu963ra_gyro_y=imu963ra_gyro_transition(imu963ra_gyro_y);
		imu963ra_gyro_z=imu963ra_gyro_transition(imu963ra_gyro_z);
		AngleSpeedX_bias=AngleSpeedX_bias+imu963ra_gyro_x;
		AngleSpeedY_bias=AngleSpeedY_bias+imu963ra_gyro_y;
		AngleSpeedZ_bias=AngleSpeedZ_bias+imu963ra_gyro_z;
	}
	AngleSpeedX_bias=AngleSpeedX_bias/30000;
	AngleSpeedY_bias=AngleSpeedY_bias/30000;
	AngleSpeedZ_bias=AngleSpeedZ_bias/30000;
	KalmanInit(&Kx,0.01,0.05,0.01,0.01);//��ʼ���������˲�
	KalmanInit(&Ky,0.01,0.05,0.01,0.01);//��ʼ���������˲�
	Kx.state.bias=AngleSpeedX_bias;
	Ky.state.bias=AngleSpeedY_bias;
}
//��ȡ�Ƕ�����
void GetAngleData()
{
	float angle_x,angle_y;
	
	imu963ra_get_acc();//��ȡ���ٶ�����
	GainAngleSpeed();//��ȡ���ٶ�
	angle_x=atan2(imu963ra_acc_y,imu963ra_acc_z)*180/3.1415926;
	angle_y=atan2(imu963ra_acc_x,imu963ra_acc_z)*180/3.1415926;
	
	AngleX=Kalman_filter(&Kx,angle_x,AngleSpeed_x)-AngleX_bias;
	AngleY=Kalman_filter(&Ky,angle_y,AngleSpeed_y)-AngleY_bias;
	
	AngleZ=AngleZ+(AngleSpeed_z-AngleSpeedZ_bias)*0.01;//�����ǻ���
	
	AngleZ_2=AngleZ_2+(AngleSpeed_z-AngleSpeedZ_bias)*0.01;//�����ǻ���


}