#include "motor.h"
#define pi 3.141593

//0:左上FL 1:右上FR 2:左下BL 3:右下BR
//满占空比为50000

float CarSpeedX=0;//小车当前速度
float CarSpeedY=0;
float CarSpeedZ=0;



float encoder[4];//编码器读数
float encoderLast[4];//编码器上一次读数
float P_encoder[4]; //累计位移
float R = 0.0315;//车轮半径（m）


void tran_speed(float Vx,float Vy,float Vz)//解算出麦轮每个轮子转速
{
	Vx=Vx/A_pulse_x;
	Vy=Vy/A_pulse_x;
	Vz=Vz/A_pulse_x;
	
	
	target_motor[0] = +Vy + Vx - Vz;
	target_motor[1] = -Vy + Vx + Vz;
	target_motor[2] = -Vy + Vx - Vz;
	target_motor[3] = +Vy + Vx + Vz;
}

/**************************************************************************
函数功能：电机初始化
入口参数：
返回  值：无
**************************************************************************/
void motor_init(void)
{
	gpio_init(ENFR,GPO,0,GPO_PUSH_PULL);//电机引脚初始化为0
	gpio_init(ENFL,GPO,0,GPO_PUSH_PULL);
	pwm_init(PWMFR,17000,0);//引脚PWM初始化，占空比为0
	pwm_init(PWMFL,17000,0);
	gpio_init(ENBR,GPO,0,GPO_PUSH_PULL);
	gpio_init(ENBL,GPO,0,GPO_PUSH_PULL);
	pwm_init(PWMBR,17000,0);
	pwm_init(PWMBL,17000,0);
	
}

/**************************************************************************
函数功能：编码器初始化
入口参数：
返回  值：无
**************************************************************************/
void encoder_init(void)
{
	encoder_dir_init(ENCOEDER_FR_DIR,ENCOEDER_FR_DIR_PULSE,ENCOEDER_FR_DIR_DIR);
	encoder_dir_init(ENCOEDER_FL_DIR,ENCOEDER_FL_DIR_PULSE,ENCOEDER_FL_DIR_DIR);
	encoder_dir_init(ENCOEDER_BR_DIR,ENCOEDER_BR_DIR_PULSE,ENCOEDER_BR_DIR_DIR);
	encoder_dir_init(ENCOEDER_BL_DIR,ENCOEDER_BL_DIR_PULSE,ENCOEDER_BL_DIR_DIR);
}

/**************************************************************************
函数功能：电机驱动
入口参数：
返回  值：无
**************************************************************************/
void motor_control(float pid_motor[4])
{  
		
		
		int j;
		int Amplitude_motor=40000;    //===PWM满幅是10000 限制在5000 
		for(j=0;j<4;j++)//限幅
		{
		//		PRINTF("%f\n",pid_motor[0]);
		if(pid_motor[j]>Amplitude_motor) pid_motor[j]=Amplitude_motor;
		if(pid_motor[j]<-Amplitude_motor) pid_motor[j]=-Amplitude_motor;
		}				
		if(pid_motor[0]>0) //电机1   正转 设置占空比为 百分之 (1000/TIMER1_PWM_DUTY_MAX*100)
		{
				gpio_set_level(ENFR,0);
				pwm_set_duty(PWMFR,pid_motor[0]);
		}
		else                //电机1   反转
		{
				gpio_set_level(ENFR,1);
				pwm_set_duty(PWMFR,-pid_motor[0]);
		}
		if(pid_motor[1]>0) //电机2   正转
		{
				gpio_set_level(ENFL,1);
				pwm_set_duty(PWMFL,pid_motor[1]);
		}
		else                //电机2   反转
		{
				gpio_set_level(ENFL,0);
				pwm_set_duty(PWMFL,-pid_motor[1]);
		}

		if(pid_motor[2]>0) //电机3   正转
		{
				gpio_set_level(ENBR,0);
				pwm_set_duty(PWMBR,pid_motor[2]);
		}
		else                //电机3   反转
		{
				gpio_set_level(ENBR,1);
				pwm_set_duty(PWMBR,-pid_motor[2]);
		}

		if(pid_motor[3]>0) //电机4   正转
		{
				gpio_set_level(ENBL,1);
				pwm_set_duty(PWMBL,pid_motor[3]);
		}
		else                //电机4   反转
		{
				gpio_set_level(ENBL,0);
				pwm_set_duty(PWMBL,-pid_motor[3]);
		}
}


/**************************************************************************
函数功能：读取编码器脉冲
入口参数：
返回  值：无
**************************************************************************/
void Read_Encoder(void)
{
	float a=0.2;//滤波系数
	//编码器读取
	encoder[0] =  encoder_get_count(ENCOEDER_FR_DIR);
	encoder[1] = -encoder_get_count(ENCOEDER_FL_DIR);
	encoder[2] =  encoder_get_count(ENCOEDER_BR_DIR);
	encoder[3] = -encoder_get_count(ENCOEDER_BL_DIR);
	
	
	//编码器滤波
	encoder[0]=a*encoder[0]+(1-a)*encoderLast[0];
	encoder[1]=a*encoder[1]+(1-a)*encoderLast[1];
	encoder[2]=a*encoder[2]+(1-a)*encoderLast[2];
	encoder[3]=a*encoder[3]+(1-a)*encoderLast[3];
	
	//保存上一次的值
	encoderLast[0]=encoder[0];
	encoderLast[1]=encoder[1];//updata_mileage();
	encoderLast[2]=encoder[2];
	encoderLast[3]=encoder[3];
//	

	P_encoder[0] += encoder[0]*0.01*A_pulse_x;
	P_encoder[1] += encoder[1]*0.01*A_pulse_x;
	P_encoder[2] += encoder[2]*0.01*A_pulse_x;
	P_encoder[3] += encoder[3]*0.01*A_pulse_x;
	
	encoder_clear_count(ENCOEDER_FL_DIR);
	encoder_clear_count(ENCOEDER_FR_DIR);
	encoder_clear_count(ENCOEDER_BL_DIR);
	encoder_clear_count(ENCOEDER_BR_DIR);
}

/******************************************************
函数功能：解算小车实际速度
入口参数：编码器值
返回值：无
*******************************************************/
void SpeedForwardSolution()
{
	CarSpeedX=(encoder[0]+encoder[1]+encoder[2]+encoder[3])*A_pulse_x/4;
	CarSpeedY=(encoder[0]-encoder[1]+encoder[2]-encoder[3])*A_pulse_x/4;
	//CarSpeedZ=(encoder[0]-encoder[1]-encoder[2]+encoder[3])*A_pulse_x/4/CarR;

}
/**************************************************************************
函数功能：读取编码器数值并求出位移，单位m/s
入口参数：四个编码器位移值,输出量
返回值：无
**************************************************************************/
void Encoder_odometer(float P_encoder[4],float * out_distanceX,float * out_distanceY)
{
	float CarDistanceX=0;//小车x方向位移
	float CarDistanceY=0;//小车y方向位移
	
	CarDistanceX=(P_encoder[0]+P_encoder[1]+P_encoder[2]+P_encoder[3])/4;//计算
	CarDistanceY=(-P_encoder[0]+P_encoder[1]+P_encoder[2]-P_encoder[3])/4;//计算
	
	*out_distanceX=CarDistanceX;
	*out_distanceY=CarDistanceY;
	
}
