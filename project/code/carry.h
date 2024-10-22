#ifndef _CARRY_H
#define _CARRY_H

#include "zf_common_headfile.h"

extern uint16 servo1_duty; 
extern uint16 servo2_duty;

void Carry_Init(void);
void servo_slow_ctrl(uint16 _servo1_angle, uint16 _servo2_angle, float _step_count);
void Carry(int Servo_mode);
void Open1_Door(void);
void Close1_Door(void);
void Open2_Door(void);
void Close2_Door(void);
#endif