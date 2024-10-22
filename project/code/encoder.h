#ifndef _ENCODER_H
#define _ENCODER_H

#include "zf_common_headfile.h"

#define PIT_CH                          (PIT_CH0 )                              // ʹ�õ������жϱ�� ����޸� ��Ҫͬ����Ӧ�޸������жϱ���� isr.c �еĵ���
#define PIT_PRIORITY                    (PIT_IRQn)                              // ��Ӧ�����жϵ��жϱ�� 

#define ENCODER_FR_DIR		                (QTIMER2_ENCOEDER1)                     // �������������Ӧʹ�õı������ӿ� ����ʹ��QTIMER1��ENCOEDER2
#define ENCODER_FR_DIR_PULSE               (QTIMER2_ENCOEDER1_CH1_C3)              // PULSE ��Ӧ������
#define ENCODER_FR_DIR_DIR                 (QTIMER2_ENCOEDER1_CH2_C25) 						//DIR ��Ӧ����

#define ENCODER_FL_DIR		                (QTIMER2_ENCOEDER1) 					 //����                   
#define ENCODER_FL_DIR_PULSE               (QTIMER2_ENCOEDER1_CH1_C3)   //����           
#define ENCODER_FL_DIR_DIR                 (QTIMER2_ENCOEDER1_CH2_C25) 	//���� 				

#define ENCODER_BR_DIR		                (QTIMER2_ENCOEDER1)           //����           
#define ENCODER_BR_DIR_PULSE               (QTIMER2_ENCOEDER1_CH1_C3)  //����             
#define ENCODER_BR_DIR_DIR                 (QTIMER2_ENCOEDER1_CH2_C25) 		//���� 			

#define ENCODER_BL_DIR		                (QTIMER2_ENCOEDER1)           //����           
#define ENCODER_BL_DIR_PULSE               (QTIMER2_ENCOEDER1_CH1_C3)   //����            
#define ENCODER_BL_DIR_DIR                 (QTIMER2_ENCOEDER1_CH2_C25) 		//���� 			


extern int32 encoder_fr,encoder_fl,encoder_br,encoder_bl;//��������ֵ

void encoder_init(void);
void encoder_get(void);

#endif