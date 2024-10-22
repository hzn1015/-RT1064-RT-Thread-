#include "zf_common_headfile.h"
#include <math.h>

float Gyro_z=0;
float fil_Acc_x,fil_Acc_y,fil_Gyro_z;
float Angle_z=0;
float Angle_z,Angle_Z=90;
float coe_Gyro_z=0.2;
float ICM20602_FIFO[11];
int moto_flag=0;
int gyro_i=0;
int Angle = 0;
#define dt 0.01
/**************************************************************************
�������ܣ�����ƽ���˲��㷨 ������ٶ�
��ڲ�������
����  ֵ����
**************************************************************************/
void ICM20602_newValues()
{
	 float sum=0;
	 static float gyro[100],sum_gyro;
	 static int gyro_flag=0,Gyro_flag;
	 
	 icm20602_get_gyro();		
		if(gyro_flag==0)
	 {		 
		  gyro[gyro_i]=icm20602_gyro_z;
		  fil_Gyro_z=0.0;
		  gyro_i++;
		 if(gyro_i==99)
		 {
			 moto_flag=1;
			 for(gyro_i=0;gyro_i<100;gyro_i++)
			 {
				 sum_gyro+=gyro[gyro_i];
			 }
			 gyro_flag=1;
		 }
	 } 
	 if(gyro_flag==1)
	 {
    Gyro_z = (float)(icm20602_gyro_z-sum_gyro/100)/16.3835;
	  if(abs(Gyro_z)<1)//���ٶ�С��3ʱ  Ĭ��ΪС����ֹ  
	  {
		  Gyro_z=0;
	  }
	  for(Gyro_flag=1;Gyro_flag<10;Gyro_flag++)
		{	
		  ICM20602_FIFO[Gyro_flag-1]=ICM20602_FIFO[Gyro_flag];//FIFO ����
		}
	  ICM20602_FIFO[9]=Gyro_z;
	  for(Gyro_flag=0;Gyro_flag<10;Gyro_flag++)
		{	            
			sum+=ICM20602_FIFO[Gyro_flag];//��ǰ����ĺϣ���ȡƽ��ֵ
		}
	  fil_Gyro_z=sum/10;
	}
}		
/**************************************************************************
�������ܣ��Խ��ٶȻ��� �õ��Ƕ�
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_angle()
{
   ICM20602_newValues();
	 Angle_Z-=fil_Gyro_z*dt;
	 if(Angle_Z>=360) Angle_Z=Angle_Z-360;
	 if(Angle_Z<=-360) Angle_Z=Angle_Z+360;
}

void Imu_Init(void)
{
	while(1)
    {
			// �˴���д��Ҫѭ��ִ�еĴ���
			if(icm20602_init())
			{
				ips200_show_string(0,0,"no ok");  
			}
			else
			{
				//gyroOffset_init();
				break;
			}
		} 
}

//#define delta_T 0.001f  //1ms����һ��
//#define M_PI        3.1415926f

//float I_ex, I_ey, I_ez;  // ������

//float param_Kp = 0.18;   // ���ٶȼƵ��������ʱ�������(δ���е���)
//float param_Ki = 0.003;   //�������������ʵĻ������� 0.004(δ���е���)


//quater_param_t Q_init = {1,0,0,0};
//euler_param_t eulerAngle;//ŷ����
//imu_param_t imu_data;
//gyro_param_t GyroOffset;

//bool GyroOffset_init = 0;//��������Ʈ��ʼ����־λ

////���ٿ�������
//float fast_sqrt(float x)
//{		float halfx = 0.5f * x;
//    float y = x;
//    long i = *(long *) &y;
//    i = 0x5f3759df - (i >> 1);
//    y = *(float *) &i;
//    y = y * (1.5f - (halfx * y * y));
//    return y;
//}

//void gyroOffset_init(void)      /////////��������Ʈ��ʼ��
//{
//    GyroOffset.Xdata = 0;
//    GyroOffset.Ydata = 0;
//    GyroOffset.Zdata = 0;
//    for (uint16_t i = 0; i < 1000; ++i) {
//        icm20602_get_gyro();
//        icm20602_get_acc();
//        GyroOffset.Xdata += icm20602_gyro_x;
//        GyroOffset.Ydata += icm20602_gyro_y;
//        GyroOffset.Zdata += icm20602_gyro_z;
//        system_delay_ms(10);
//    }

//    GyroOffset.Xdata /= 100;
//    GyroOffset.Ydata /= 100;
//    GyroOffset.Zdata /= 100;

//    GyroOffset_init = 1;
//}

//#define alpha			0.9f     //�˲�ϵ����ȡֵ��ΧΪ0~1, ֵԽСԽ�ȶ���Խ��Խ����(δ���е���)
////���������ɼ���������ת��Ϊʵ������ֵ
//void IMU_getValues(void)
//{
//	//�Լ��ٶȼ����ݽ���һ�׵�ͨ�˲�  ���ٶȼ�������Ϊ2g
//	imu_data.acc_x = (((float) icm20602_acc_x) * alpha) /8192 + imu_data.acc_x * (1 - alpha);
//	imu_data.acc_y = (((float) icm20602_acc_y) * alpha) /8192 + imu_data.acc_y * (1 - alpha);
//	imu_data.acc_z = (((float) icm20602_acc_z) * alpha) /8192 + imu_data.acc_z * (1 - alpha);
//	
//	//������������Ϊ
//	imu_data.gyro_x = ((float)icm20602_gyro_x - GyroOffset.Xdata) * M_PI / 180 / 131.0f;
//	imu_data.gyro_y = ((float)icm20602_gyro_y - GyroOffset.Ydata) * M_PI / 180 / 131.0f;
//	imu_data.gyro_z = ((float)icm20602_gyro_z - GyroOffset.Zdata) * M_PI / 180 / 131.0f;
//}

////�����˲�
//void IMU_AHRSupdate(float gx,float gy,float gz,float ax,float ay,float az)
//{
//	float vx, vy, vz;    //��ǰ�Ļ�������ϵ�ϵ�������λ����
//	float ex, ey, ez;    //��Ԫ������ֵ����ٶȼƲ���ֵ�����
//	float halfT = 0.5 * delta_T;
//	float q0 = Q_init.q0;
//  float q1 = Q_init.q1;
//  float q2 = Q_init.q2;
//  float q3 = Q_init.q3;
//  float q0q0 = q0 * q0;
//  float q0q1 = q0 * q1;
//  float q0q2 = q0 * q2;
//  float q0q3 = q0 * q3;
//  float q1q1 = q1 * q1;
//  float q1q2 = q1 * q2;
//  float q1q3 = q1 * q3;
//  float q2q2 = q2 * q2;
//  float q2q3 = q2 * q3;
//  float q3q3 = q3 * q3;
//	
//	//�Լ��ٶ�ֵ���й�һ���õ���λ���ٶ�ֵ
//	float norm = fast_sqrt(ax * ax + ay * ay + az * az);
//	ax = ax * norm;
//	ay = ay * norm;
//	az = az * norm;
//	
//	//���ݵ�ǰ��Ԫ������ֵ̬����������������������ںͼ��ټ�ʵ�ʲ��������ĸ������������жԱȣ��Ӷ�ʵ�ֶ�������̬������
//	vx = 2 * (q1q3 - q0q2);
//  vy = 2 * (q0q1 + q2q3);
//  vz = q0q0 - q1q1 - q2q2 + q3q3;
//	
//	//�������������������ʵ�ʲ�����������������������֮�����
//    ex = ay * vz - az * vy;
//    ey = az * vx - ax * vz;
//    ez = ax * vy - ay * vx;
//		
//		//�ò���������PI����������ƫ��
//    //ͨ������ param_Kp��param_Ki ����������
//    //���Կ��Ƽ��ٶȼ����������ǻ�����̬���ٶȡ�
//    I_ex += halfT * ex;   // integral error scaled by Ki
//    I_ey += halfT * ey;
//    I_ez += halfT * ez;
//		
//		gx = gx + param_Kp * ex + param_Ki * I_ex;
//    gy = gy + param_Kp * ey + param_Ki * I_ey;
//    gz = gz + param_Kp * ez + param_Ki * I_ez;
//		
//		/*����������ɣ����������Ԫ��΢��*/
//		
//		
//		 //��Ԫ��΢�ַ��̣�����halfTΪ�������ڵ�1/2��gx gy gzΪ�����ǽ��ٶȣ����¶�����֪��������ʹ����һ��������������Ԫ��΢�ַ���
//    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
//    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
//    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
//    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
//		
//		//����Ԫ�����й�һ��
//		norm = fast_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
//		Q_init.q0 = q0 * norm;
//		Q_init.q1 = q1 * norm;
//		Q_init.q2 = q2 * norm;
//		Q_init.q3 = q3 * norm;
//}

////����Ԫ��ת��Ϊŷ����
//void IMU_getEulerianAngles(void)
//{
//	//�ɼ�����������
//	icm20602_get_gyro();
//  icm20602_get_acc();
//	
//	IMU_getValues();//������������ת��Ϊ����ֵ
//	IMU_AHRSupdate(imu_data.gyro_x , imu_data.gyro_y , imu_data.gyro_z , imu_data.acc_x , imu_data.acc_y , imu_data.acc_z);
//	
//	  float q0 = Q_init.q0;
//    float q1 = Q_init.q1;
//    float q2 = Q_init.q2;
//    float q3 = Q_init.q3;

//    //��Ԫ������ŷ����
////    eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / M_PI; // pitch
////    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / M_PI; // roll
//    eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI; // yaw

///*   ��̬����*/
////    if (eulerAngle.roll > 90 || eulerAngle.roll < -90) {
////        if (eulerAngle.pitch > 0) {
////            eulerAngle.pitch = 180 - eulerAngle.pitch;
////        }
////        if (eulerAngle.pitch < 0) {
////            eulerAngle.pitch = -(180 + eulerAngle.pitch);
////        }
////    }

//    if (eulerAngle.yaw > 360) {
//        eulerAngle.yaw -= 360;
//    } else if (eulerAngle.yaw < 0) {
//        eulerAngle.yaw += 360;
//    }
//}


