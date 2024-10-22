#ifndef _ENCODER_H
#define _ENCODER_H

#include "zf_common_headfile.h"

#define PIT_CH                          (PIT_CH0 )                              // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY                    (PIT_IRQn)                              // 对应周期中断的中断编号 

#define ENCODER_FR_DIR		                (QTIMER2_ENCOEDER1)                     // 带方向编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER2
#define ENCODER_FR_DIR_PULSE               (QTIMER2_ENCOEDER1_CH1_C3)              // PULSE 对应的引脚
#define ENCODER_FR_DIR_DIR                 (QTIMER2_ENCOEDER1_CH2_C25) 						//DIR 对应引脚

#define ENCODER_FL_DIR		                (QTIMER2_ENCOEDER1) 					 //待改                   
#define ENCODER_FL_DIR_PULSE               (QTIMER2_ENCOEDER1_CH1_C3)   //待改           
#define ENCODER_FL_DIR_DIR                 (QTIMER2_ENCOEDER1_CH2_C25) 	//待改 				

#define ENCODER_BR_DIR		                (QTIMER2_ENCOEDER1)           //待改           
#define ENCODER_BR_DIR_PULSE               (QTIMER2_ENCOEDER1_CH1_C3)  //待改             
#define ENCODER_BR_DIR_DIR                 (QTIMER2_ENCOEDER1_CH2_C25) 		//待改 			

#define ENCODER_BL_DIR		                (QTIMER2_ENCOEDER1)           //待改           
#define ENCODER_BL_DIR_PULSE               (QTIMER2_ENCOEDER1_CH1_C3)   //待改            
#define ENCODER_BL_DIR_DIR                 (QTIMER2_ENCOEDER1_CH2_C25) 		//待改 			


extern int32 encoder_fr,encoder_fl,encoder_br,encoder_bl;//编码器的值

void encoder_init(void);
void encoder_get(void);

#endif