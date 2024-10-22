#ifndef _USART_H_
#define _USART_H_

#include "zf_common_headfile.h"
//野火串口PID调试助手协议
//通道地址
#define PID_CH1 0x01
#define PID_CH2 0x02
#define PID_CH3 0x03
#define PID_CH4 0x04
#define PID_CH5 0x05

#define PID_PackHead 0x59485A53 //包头
//下位机发送给上位机指令
#define Set_PID_Target 0x01 //设置上位机通道目标值
#define Set_PID_present 0x02 //设置上位机通道实际值
#define Set_Computer_PID 0x03//设置上位机PID值
#define Set_Computer_Start 0x04 //设置上位机启动指令
#define Set_Computer_Stop 0x05 //设置上位机停止指令
#define Set_Computer_T 0x06//设置上位机周期
//上位机发送给下位机指令
#define Set_PID 0x10//设置下位机PID值
#define Set_Target 0x11//设置下位机目标值
#define Set_Start 0x12 //启动指令
#define Set_Stop 0x13 //停止指令
#define Set_Reset 0x14 //复位指令
#define Set_T 0x15//设置下位机周期
//PID数据包格式
struct PID_Packet 
{
	int Packet_Head;//包头
	unsigned char Packet_address;//通道地址
	int Packet_len;//包长度
	unsigned char Packet_cmd;//命令
	//char * Packet_argument;//数据包参数
	unsigned char Cheek_sum;//校验和

};
void Uart_PID_Start(unsigned char PID_CHANNEL);
void Sent_PID_Target(unsigned char PID_CHANNEL ,int target_value);
void Sent_PID_Present(unsigned char PID_CHANNEL ,int present_value);
void Sent_PID_Cycle(unsigned char PID_CHANNEL ,int Cycle_value);
void Sent_PID_Parameter(unsigned char PID_CHANNEL,float Kp,float Ki,float Kd);
void PID_UsartInit();
#endif