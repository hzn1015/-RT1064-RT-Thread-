#ifndef _USART_H_
#define _USART_H_

#include "zf_common_headfile.h"
//Ұ�𴮿�PID��������Э��
//ͨ����ַ
#define PID_CH1 0x01
#define PID_CH2 0x02
#define PID_CH3 0x03
#define PID_CH4 0x04
#define PID_CH5 0x05

#define PID_PackHead 0x59485A53 //��ͷ
//��λ�����͸���λ��ָ��
#define Set_PID_Target 0x01 //������λ��ͨ��Ŀ��ֵ
#define Set_PID_present 0x02 //������λ��ͨ��ʵ��ֵ
#define Set_Computer_PID 0x03//������λ��PIDֵ
#define Set_Computer_Start 0x04 //������λ������ָ��
#define Set_Computer_Stop 0x05 //������λ��ָֹͣ��
#define Set_Computer_T 0x06//������λ������
//��λ�����͸���λ��ָ��
#define Set_PID 0x10//������λ��PIDֵ
#define Set_Target 0x11//������λ��Ŀ��ֵ
#define Set_Start 0x12 //����ָ��
#define Set_Stop 0x13 //ָֹͣ��
#define Set_Reset 0x14 //��λָ��
#define Set_T 0x15//������λ������
//PID���ݰ���ʽ
struct PID_Packet 
{
	int Packet_Head;//��ͷ
	unsigned char Packet_address;//ͨ����ַ
	int Packet_len;//������
	unsigned char Packet_cmd;//����
	//char * Packet_argument;//���ݰ�����
	unsigned char Cheek_sum;//У���

};
void Uart_PID_Start(unsigned char PID_CHANNEL);
void Sent_PID_Target(unsigned char PID_CHANNEL ,int target_value);
void Sent_PID_Present(unsigned char PID_CHANNEL ,int present_value);
void Sent_PID_Cycle(unsigned char PID_CHANNEL ,int Cycle_value);
void Sent_PID_Parameter(unsigned char PID_CHANNEL,float Kp,float Ki,float Kd);
void PID_UsartInit();
#endif