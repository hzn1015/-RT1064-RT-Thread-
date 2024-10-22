#include "zf_common_headfile.h"
#include "math.h"

#define SERVO_MOTOR_PWM1                (PWM4_MODULE2_CHA_C30)                      // 定义主板上舵机对应引脚
#define SERVO_MOTOR_PWM2                (PWM1_MODULE3_CHA_D0)                       // 定义主板上舵机对应引脚 
#define SERVO_MOTOR_FREQ                (50)                                       // 定义主板上舵机频率  请务必注意范围 50-300

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))    //舵机角度设置为0 - 180，90度为中值

uint16 servo1_duty = 0, servo2_duty = 0;

void Carry_Init(void)
{
	pwm_init(SERVO_MOTOR_PWM1, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(60));            
	pwm_init(SERVO_MOTOR_PWM2, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(40));     
	gpio_init(D1, GPO, 0, GPO_PUSH_PULL);
  //servo1_duty = 151;servo2_duty = 134;
}

//参数1：舵机1的目标角度 参数2：舵机2的目标角度  参数3：舵机连续控制间隔次数
void servo_slow_ctrl(uint16 _servo1_angle, uint16 _servo2_angle, float _step_count)
{
    float servo1_start = (float)servo1_duty, servo2_start = (float)servo2_duty;
    float servo1_step = (float)(_servo1_angle - servo1_duty)/_step_count, servo2_step = (float)(_servo2_angle - servo2_duty)/_step_count;
    while(1)
    {
        system_delay_ms(5);
        if(fabsf(servo1_start - (float)_servo1_angle) >= servo1_step)servo1_start += servo1_step;
        else servo1_start = _servo1_angle;
        pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)servo1_start));
        
        if(fabsf(servo2_start - (float)_servo2_angle) >= servo2_step)servo2_start += servo2_step;
        else servo2_start = _servo2_angle;
        pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY((uint16)servo2_start));
        
        if(fabsf(servo1_start - (float)_servo1_angle) < 1 && fabsf(servo2_start - (float)_servo2_angle) < 1)
        {
            servo1_duty = (uint16)_servo1_angle;
            servo2_duty = (uint16)_servo2_angle;
            return;
        }
    }
    
}

//0为搬，1为放, 2为放上车上
void Carry(int Servo_mode)
{
	if(Servo_mode==0)
	{
		gpio_set_level(D1, 1);
		if(servo1_duty != 168 || servo2_duty != 145)
		{
			servo_slow_ctrl(168, 145, 100);
		}
			servo_slow_ctrl(80, 90, 100);
	}
	if(Servo_mode==1)
	{
		if(servo1_duty != 160 || servo2_duty != 150)
		{
			servo_slow_ctrl(160, 150, 50);
		}
		gpio_set_level(D1, 0);
		servo_slow_ctrl(60, 40, 50);
	}
	if(Servo_mode==2)
	{
			servo_slow_ctrl(60, 10, 100);
			gpio_set_level(D1, 0);
			servo_slow_ctrl(60, 40, 100);	
	}
//		servo_slow_ctrl(70, 70, 50);
//		servo_slow_ctrl(60, 3, 100);
//		gpio_set_level(D1, 0);
}

//#define Servo PWM4_MODULE2_CHA_C30
//#define Servo_100_degree  5300
//#define Servo_0_degree  2800

//void Carry_Init(void)
//{
//	pwm_init(Servo,50,0);
//	pwm_set_duty(Servo,Servo_100_degree);
//	gpio_init(D0,GPO,0,GPO_PUSH_PULL);
//}

//void My_Delay(int time)
//{
//	static int Time;
//	Time=time*50000;//每50000为1ms
//	while(Time)
//	{
//		Time--;
//	}
//}
////0为搬  1为放 2为初始
//void Carry(int Carry_mode)
//{
//	if(Carry_mode==0)
//	{
//		gpio_init(D0,GPO,1,GPO_PUSH_PULL);
//		pwm_set_duty(Servo,Servo_0_degree);
//		My_Delay(25000);
//		pwm_set_duty(Servo,Servo_100_degree);
//	}
//}
void Open1_Door(void)
{ 
	 pwm_set_duty(SERVO_MOTOR_PWM1,SERVO_MOTOR_DUTY(0));    
	
}
void Close1_Door(void)
{
	 pwm_set_duty(SERVO_MOTOR_PWM1,SERVO_MOTOR_DUTY(90));    
}

void Open2_Door(void)
{ 
	 pwm_set_duty(SERVO_MOTOR_PWM2,SERVO_MOTOR_DUTY(60));    
	
}
void Close2_Door(void)
{
	 pwm_set_duty(SERVO_MOTOR_PWM2,SERVO_MOTOR_DUTY(0));    
}
