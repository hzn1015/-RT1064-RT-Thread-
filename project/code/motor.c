#include "motor.h"
#define pi 3.141593

//0:����FL 1:����FR 2:����BL 3:����BR
//��ռ�ձ�Ϊ50000

float CarSpeedX=0;//С����ǰ�ٶ�
float CarSpeedY=0;
float CarSpeedZ=0;



float encoder[4];//����������
float encoderLast[4];//��������һ�ζ���
float P_encoder[4]; //�ۼ�λ��
float R = 0.0315;//���ְ뾶��m��


void tran_speed(float Vx,float Vy,float Vz)//���������ÿ������ת��
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
�������ܣ������ʼ��
��ڲ�����
����  ֵ����
**************************************************************************/
void motor_init(void)
{
	gpio_init(ENFR,GPO,0,GPO_PUSH_PULL);//������ų�ʼ��Ϊ0
	gpio_init(ENFL,GPO,0,GPO_PUSH_PULL);
	pwm_init(PWMFR,17000,0);//����PWM��ʼ����ռ�ձ�Ϊ0
	pwm_init(PWMFL,17000,0);
	gpio_init(ENBR,GPO,0,GPO_PUSH_PULL);
	gpio_init(ENBL,GPO,0,GPO_PUSH_PULL);
	pwm_init(PWMBR,17000,0);
	pwm_init(PWMBL,17000,0);
	
}

/**************************************************************************
�������ܣ���������ʼ��
��ڲ�����
����  ֵ����
**************************************************************************/
void encoder_init(void)
{
	encoder_dir_init(ENCOEDER_FR_DIR,ENCOEDER_FR_DIR_PULSE,ENCOEDER_FR_DIR_DIR);
	encoder_dir_init(ENCOEDER_FL_DIR,ENCOEDER_FL_DIR_PULSE,ENCOEDER_FL_DIR_DIR);
	encoder_dir_init(ENCOEDER_BR_DIR,ENCOEDER_BR_DIR_PULSE,ENCOEDER_BR_DIR_DIR);
	encoder_dir_init(ENCOEDER_BL_DIR,ENCOEDER_BL_DIR_PULSE,ENCOEDER_BL_DIR_DIR);
}

/**************************************************************************
�������ܣ��������
��ڲ�����
����  ֵ����
**************************************************************************/
void motor_control(float pid_motor[4])
{  
		
		
		int j;
		int Amplitude_motor=40000;    //===PWM������10000 ������5000 
		for(j=0;j<4;j++)//�޷�
		{
		//		PRINTF("%f\n",pid_motor[0]);
		if(pid_motor[j]>Amplitude_motor) pid_motor[j]=Amplitude_motor;
		if(pid_motor[j]<-Amplitude_motor) pid_motor[j]=-Amplitude_motor;
		}				
		if(pid_motor[0]>0) //���1   ��ת ����ռ�ձ�Ϊ �ٷ�֮ (1000/TIMER1_PWM_DUTY_MAX*100)
		{
				gpio_set_level(ENFR,0);
				pwm_set_duty(PWMFR,pid_motor[0]);
		}
		else                //���1   ��ת
		{
				gpio_set_level(ENFR,1);
				pwm_set_duty(PWMFR,-pid_motor[0]);
		}
		if(pid_motor[1]>0) //���2   ��ת
		{
				gpio_set_level(ENFL,1);
				pwm_set_duty(PWMFL,pid_motor[1]);
		}
		else                //���2   ��ת
		{
				gpio_set_level(ENFL,0);
				pwm_set_duty(PWMFL,-pid_motor[1]);
		}

		if(pid_motor[2]>0) //���3   ��ת
		{
				gpio_set_level(ENBR,0);
				pwm_set_duty(PWMBR,pid_motor[2]);
		}
		else                //���3   ��ת
		{
				gpio_set_level(ENBR,1);
				pwm_set_duty(PWMBR,-pid_motor[2]);
		}

		if(pid_motor[3]>0) //���4   ��ת
		{
				gpio_set_level(ENBL,1);
				pwm_set_duty(PWMBL,pid_motor[3]);
		}
		else                //���4   ��ת
		{
				gpio_set_level(ENBL,0);
				pwm_set_duty(PWMBL,-pid_motor[3]);
		}
}


/**************************************************************************
�������ܣ���ȡ����������
��ڲ�����
����  ֵ����
**************************************************************************/
void Read_Encoder(void)
{
	float a=0.2;//�˲�ϵ��
	//��������ȡ
	encoder[0] =  encoder_get_count(ENCOEDER_FR_DIR);
	encoder[1] = -encoder_get_count(ENCOEDER_FL_DIR);
	encoder[2] =  encoder_get_count(ENCOEDER_BR_DIR);
	encoder[3] = -encoder_get_count(ENCOEDER_BL_DIR);
	
	
	//�������˲�
	encoder[0]=a*encoder[0]+(1-a)*encoderLast[0];
	encoder[1]=a*encoder[1]+(1-a)*encoderLast[1];
	encoder[2]=a*encoder[2]+(1-a)*encoderLast[2];
	encoder[3]=a*encoder[3]+(1-a)*encoderLast[3];
	
	//������һ�ε�ֵ
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
�������ܣ�����С��ʵ���ٶ�
��ڲ�����������ֵ
����ֵ����
*******************************************************/
void SpeedForwardSolution()
{
	CarSpeedX=(encoder[0]+encoder[1]+encoder[2]+encoder[3])*A_pulse_x/4;
	CarSpeedY=(encoder[0]-encoder[1]+encoder[2]-encoder[3])*A_pulse_x/4;
	//CarSpeedZ=(encoder[0]-encoder[1]-encoder[2]+encoder[3])*A_pulse_x/4/CarR;

}
/**************************************************************************
�������ܣ���ȡ��������ֵ�����λ�ƣ���λm/s
��ڲ������ĸ�������λ��ֵ,�����
����ֵ����
**************************************************************************/
void Encoder_odometer(float P_encoder[4],float * out_distanceX,float * out_distanceY)
{
	float CarDistanceX=0;//С��x����λ��
	float CarDistanceY=0;//С��y����λ��
	
	CarDistanceX=(P_encoder[0]+P_encoder[1]+P_encoder[2]+P_encoder[3])/4;//����
	CarDistanceY=(-P_encoder[0]+P_encoder[1]+P_encoder[2]-P_encoder[3])/4;//����
	
	*out_distanceX=CarDistanceX;
	*out_distanceY=CarDistanceY;
	
}
