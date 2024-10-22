#ifndef __BUTTON_H__
#define __BUTTON_H__

#include "zf_common_headfile.h"

#define ButtonPin1 C12
#define ButtonPin2 C14
#define ButtonPin3 C15
#define ButtonPin4 C13

#define Screen_Enable 1
#define Debug_Enable 0
extern uint8 button1count;
extern uint8 button2count;
extern uint8 button3count;
extern uint8 button4count;


extern uint8 ScreenClearFlag;

void ButtonInit();
void UI_Screen1();
void UI_ScreenChoice();

#endif