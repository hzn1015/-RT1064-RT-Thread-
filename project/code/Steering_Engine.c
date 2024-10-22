#include "Steering_Engine.h"
//�������
void Steering_Engine_Start()
{
		pwm_init(Engine_Pin_1,50,0);//����PWM��ʼ����ռ�ձ�Ϊ0
		pwm_init(Engine_Pin_2,50,0);//����PWM��ʼ����ռ�ձ�Ϊ0
		pwm_init(Engine_Pin_3,50,0);//����PWM��ʼ����ռ�ձ�Ϊ0
		pwm_init(Engine_Pin_4,50,0);//����PWM��ʼ����ռ�ձ�Ϊ0
		pwm_init(Engine_Pin_5,50,0);//����PWM��ʼ����ռ�ձ�Ϊ0
}
//���pwmռ�ձȷ�Χ��1250-6250
//180���� 0.036��=1���壬270���� 0.054=1����

/********���Ƕ�ת��ΪPWMֵ***************
����Ƕ�
���PWMֵ
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
//���ƶ���Ƕ�
//num������
//angle_value����Ƕ�PWMֵ
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
/******���pid����********/


