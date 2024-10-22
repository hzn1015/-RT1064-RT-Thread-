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
函数功能：递推平均滤波算法 处理角速度
入口参数：无
返回  值：无
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
	  if(abs(Gyro_z)<1)//角速度小于3时  默认为小车静止  
	  {
		  Gyro_z=0;
	  }
	  for(Gyro_flag=1;Gyro_flag<10;Gyro_flag++)
		{	
		  ICM20602_FIFO[Gyro_flag-1]=ICM20602_FIFO[Gyro_flag];//FIFO 操作
		}
	  ICM20602_FIFO[9]=Gyro_z;
	  for(Gyro_flag=0;Gyro_flag<10;Gyro_flag++)
		{	            
			sum+=ICM20602_FIFO[Gyro_flag];//求当前数组的合，再取平均值
		}
	  fil_Gyro_z=sum/10;
	}
}		
/**************************************************************************
函数功能：对角速度积分 得到角度
入口参数：无
返回  值：无
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
			// 此处编写需要循环执行的代码
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

//#define delta_T 0.001f  //1ms计算一次
//#define M_PI        3.1415926f

//float I_ex, I_ey, I_ez;  // 误差积分

//float param_Kp = 0.18;   // 加速度计的收敛速率比例增益(未进行调试)
//float param_Ki = 0.003;   //陀螺仪收敛速率的积分增益 0.004(未进行调试)


//quater_param_t Q_init = {1,0,0,0};
//euler_param_t eulerAngle;//欧拉角
//imu_param_t imu_data;
//gyro_param_t GyroOffset;

//bool GyroOffset_init = 0;//陀螺仪零飘初始化标志位

////快速开方函数
//float fast_sqrt(float x)
//{		float halfx = 0.5f * x;
//    float y = x;
//    long i = *(long *) &y;
//    i = 0x5f3759df - (i >> 1);
//    y = *(float *) &i;
//    y = y * (1.5f - (halfx * y * y));
//    return y;
//}

//void gyroOffset_init(void)      /////////陀螺仪零飘初始化
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

//#define alpha			0.9f     //滤波系数。取值范围为0~1, 值越小越稳定，越大越灵敏(未进行调试)
////将传感器采集到的数据转化为实际物理值
//void IMU_getValues(void)
//{
//	//对加速度计数据进行一阶低通滤波  加速度计量程设为2g
//	imu_data.acc_x = (((float) icm20602_acc_x) * alpha) /8192 + imu_data.acc_x * (1 - alpha);
//	imu_data.acc_y = (((float) icm20602_acc_y) * alpha) /8192 + imu_data.acc_y * (1 - alpha);
//	imu_data.acc_z = (((float) icm20602_acc_z) * alpha) /8192 + imu_data.acc_z * (1 - alpha);
//	
//	//陀螺仪量程设为
//	imu_data.gyro_x = ((float)icm20602_gyro_x - GyroOffset.Xdata) * M_PI / 180 / 131.0f;
//	imu_data.gyro_y = ((float)icm20602_gyro_y - GyroOffset.Ydata) * M_PI / 180 / 131.0f;
//	imu_data.gyro_z = ((float)icm20602_gyro_z - GyroOffset.Zdata) * M_PI / 180 / 131.0f;
//}

////互补滤波
//void IMU_AHRSupdate(float gx,float gy,float gz,float ax,float ay,float az)
//{
//	float vx, vy, vz;    //当前的机体坐标系上的重力单位向量
//	float ex, ey, ez;    //四元数计算值与加速度计测量值的误差
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
//	//对加速度值进行归一化得到单位加速度值
//	float norm = fast_sqrt(ax * ax + ay * ay + az * az);
//	ax = ax * norm;
//	ay = ay * norm;
//	az = az * norm;
//	
//	//根据当前四元数的姿态值来估算出各重力分量。用于和加速计实际测量出来的各重力分量进行对比，从而实现对四轴姿态的修正
//	vx = 2 * (q1q3 - q0q2);
//  vy = 2 * (q0q1 + q2q3);
//  vz = q0q0 - q1q1 - q2q2 + q3q3;
//	
//	//叉积来计算估算的重力和实际测量的重力这两个重力向量之间的误差。
//    ex = ay * vz - az * vy;
//    ey = az * vx - ax * vz;
//    ez = ax * vy - ay * vx;
//		
//		//用叉乘误差来做PI修正陀螺零偏，
//    //通过调节 param_Kp，param_Ki 两个参数，
//    //可以控制加速度计修正陀螺仪积分姿态的速度。
//    I_ex += halfT * ex;   // integral error scaled by Ki
//    I_ey += halfT * ey;
//    I_ez += halfT * ez;
//		
//		gx = gx + param_Kp * ex + param_Ki * I_ex;
//    gy = gy + param_Kp * ey + param_Ki * I_ey;
//    gz = gz + param_Kp * ez + param_Ki * I_ez;
//		
//		/*数据修正完成，下面进行四元数微分*/
//		
//		
//		 //四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为陀螺仪角速度，以下都是已知量，这里使用了一阶龙哥库塔求解四元数微分方程
//    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
//    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
//    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
//    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
//		
//		//将四元数进行归一化
//		norm = fast_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
//		Q_init.q0 = q0 * norm;
//		Q_init.q1 = q1 * norm;
//		Q_init.q2 = q2 * norm;
//		Q_init.q3 = q3 * norm;
//}

////把四元数转化为欧拉角
//void IMU_getEulerianAngles(void)
//{
//	//采集传感器数据
//	icm20602_get_gyro();
//  icm20602_get_acc();
//	
//	IMU_getValues();//将传感器数据转化为物理值
//	IMU_AHRSupdate(imu_data.gyro_x , imu_data.gyro_y , imu_data.gyro_z , imu_data.acc_x , imu_data.acc_y , imu_data.acc_z);
//	
//	  float q0 = Q_init.q0;
//    float q1 = Q_init.q1;
//    float q2 = Q_init.q2;
//    float q3 = Q_init.q3;

//    //四元数计算欧拉角
////    eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / M_PI; // pitch
////    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / M_PI; // roll
//    eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI; // yaw

///*   姿态限制*/
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


