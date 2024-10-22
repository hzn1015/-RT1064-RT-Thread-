#include "angle.h"
float AngleSpeed_x=0;
float AngleSpeed_y=0;
float AngleSpeed_z=0;

float AngleX=0;
float AngleY=0;
float AngleZ=0;
float AngleZ_2=0;//用于返回退出使用的
float AngleSpeedX_bias=0;
float AngleSpeedY_bias=0;
float AngleSpeedZ_bias=0;

KalmanFilter Kx,Ky ;//定义卡尔曼滤波器
/**********获取角速度**************/
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
/**********卡尔曼滤波估算角度*************/
//注意，偏航角只能通过角速度积分得到，加速度计无法获取偏航角


/**********初始化卡尔曼滤波器*************/
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

/************卡尔曼滤波算法****************/

float Kalman_filter(KalmanFilter *Kf,float newAngle,float new_gyro)
{
	float err,S;
	float K[2];//卡尔曼增益
	//先验估计
	Kf->state.angle=Kf->state.angle+Kf->dt*(new_gyro-Kf->state.bias);

	//协方差矩阵更新
	Kf->P[0][0] += Kf->dt * (Kf->dt*Kf->P[1][1] - Kf->P[0][1]
                             + Kf->P[1][0] )+ Kf->Q_angle;
  Kf->P[0][1] -= Kf->dt * Kf->P[1][1];
  Kf->P[1][0] -= Kf->dt * Kf->P[1][1];
  Kf->P[1][1] += Kf->Q_bias;
	
	//更新卡尔曼增益
	err=newAngle-Kf->state.angle;//计算偏差，测量方程
	
	S=Kf->P[0][0]+Kf->R_mea;
	
	K[0]=Kf->P[0][0]/S;
	K[1]=Kf->P[1][0]/S;
	
	//后验状态更新
	Kf->state.angle=Kf->state.angle+K[0]*err;
	Kf->state.bias=Kf->state.bias+K[1]*err;
	
	//后验协方差更新
	Kf->P[0][0] -= K[0] * Kf->P[0][0];
  Kf->P[0][1] -= K[0] * Kf->P[0][1];
  Kf->P[1][0] -= K[1] * Kf->P[0][0];
  Kf->P[1][1] -= K[1] * Kf->P[0][1];
	
	return Kf->state.angle;//返回角度
}

/********陀螺仪初始化**************/
void GyroscopeInit()
{
	imu963ra_init();//初始化陀螺仪

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
	KalmanInit(&Kx,0.01,0.05,0.01,0.01);//初始化卡尔曼滤波
	KalmanInit(&Ky,0.01,0.05,0.01,0.01);//初始化卡尔曼滤波
	Kx.state.bias=AngleSpeedX_bias;
	Ky.state.bias=AngleSpeedY_bias;
}
//获取角度数据
void GetAngleData()
{
	float angle_x,angle_y;
	
	imu963ra_get_acc();//获取加速度数据
	GainAngleSpeed();//获取角速度
	angle_x=atan2(imu963ra_acc_y,imu963ra_acc_z)*180/3.1415926;
	angle_y=atan2(imu963ra_acc_x,imu963ra_acc_z)*180/3.1415926;
	
	AngleX=Kalman_filter(&Kx,angle_x,AngleSpeed_x)-AngleX_bias;
	AngleY=Kalman_filter(&Ky,angle_y,AngleSpeed_y)-AngleY_bias;
	
	AngleZ=AngleZ+(AngleSpeed_z-AngleSpeedZ_bias)*0.01;//陀螺仪积分
	
	AngleZ_2=AngleZ_2+(AngleSpeed_z-AngleSpeedZ_bias)*0.01;//陀螺仪积分


}