#include "Steering_Engine.h"
//启动舵机
void Steering_Engine_Start()
{
		pwm_init(Engine_Pin_1,50,0);//引脚PWM初始化，占空比为0
		pwm_init(Engine_Pin_2,50,0);//引脚PWM初始化，占空比为0
		pwm_init(Engine_Pin_3,50,0);//引脚PWM初始化，占空比为0
		pwm_init(Engine_Pin_4,50,0);//引脚PWM初始化，占空比为0
		pwm_init(Engine_Pin_5,50,0);//引脚PWM初始化，占空比为0
}
//舵机pwm占空比范围：1250-6250
//180°舵机 0.036°=1脉冲，270°舵机 0.054=1脉冲

/********将角度转换为PWM值***************
输入角度
输出PWM值
***************************************/
int AngleTurnPwm1(float angle)
{
	int PWM=0;
	PWM=1250+angle*27.778;
	return PWM;
}


int AngleTurnPwm2(float angle)
{
	int PWM=0;
	PWM=1250+angle*18.5185;
	return PWM;
}
//控制舵机角度
//num舵机编号
//angle_value舵机角度PWM值
void Set_Angle(unsigned char num,int PWMvalue)
{
	switch(num)
	{
		case 1:pwm_set_duty(Engine_Pin_1,PWMvalue);break;
		case 2:pwm_set_duty(Engine_Pin_2,PWMvalue);break;
		case 3:pwm_set_duty(Engine_Pin_3,PWMvalue);break;
		case 4:pwm_set_duty(Engine_Pin_4,PWMvalue);break;
		case 5:pwm_set_duty(Engine_Pin_5,PWMvalue);break;
	}

}
/******舵机pid控制********/


