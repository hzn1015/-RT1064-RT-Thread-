#include "pid_uart.h"
//构造串口打印函数
//huart为选择的串口号
//_format为要打印的字符串


//****************PID串口调试助手********************//
//数据发送太快会出现错误
//校验和计算
void Cheek_Sum(struct PID_Packet* packet,char* hex)
{
	unsigned char len=0;//数据长度
	for(unsigned char i=0;i<packet->Packet_len;i++)
	{
		packet->Cheek_sum+=hex[i];
		
	}
	packet->Cheek_sum=(packet->Cheek_sum);
	hex[packet->Packet_len-1]=packet->Cheek_sum;
}
//字符串数据转hex数据
//将包头数据初始化
void Str_turn_Hex(struct PID_Packet* packet,char * hex)
{
	
	
		hex[3]=((((packet->Packet_Head)&0xFF000000)>>24)&0xFF);
		hex[2]=(((packet->Packet_Head)&0x00FF0000)>>16)&0xFF;
		hex[1]=(((packet->Packet_Head)&0x0000FF00)>>8)&0xFF;
		hex[0]=(((packet->Packet_Head)&0x000000FF)&0xFF);
		//UART_Printf(&huart1,"%d",hex[0]);
		hex[4]=(packet->Packet_address);
	
		hex[8]=((((packet->Packet_len)&0xFF000000)>>24)&0xFF);
		hex[7]=(((packet->Packet_len)&0x00FF0000)>>16)&0xFF;
		hex[6]=(((packet->Packet_len)&0x0000FF00)>>8)&0xFF;
		hex[5]=(((packet->Packet_len)&0x000000FF)&0xFF);
	
		hex[9]=(packet->Packet_cmd);
		//对参数进行处理
	
		
//		hex[13]=(((argument&0xFF000000)>>24)&0xFF);
//		hex[12]=((argument&0x00FF0000)>>16)&0xFF;
//		hex[11]=((argument&0x0000FF00)>>8)&0xFF;
//		hex[10]=((argument&0x000000FF)&0xFF);
//	
//		hex[14]=(packet->Cheek_sum);
		
}
//启动PID通道
void Uart_PID_Start(unsigned char PID_CHANNEL)
{
	struct PID_Packet Packet;
	char str_buff[32],hex_buff[128];
	Packet.Packet_Head=PID_PackHead;
	Packet.Packet_address=PID_CHANNEL;
	Packet.Packet_cmd=Set_Computer_Start;
	Packet.Packet_len=0x0B;
	Packet.Cheek_sum=0x5E;
	
	Str_turn_Hex(&Packet,hex_buff);
	//Cheek_Sum(&Packet,hex_buff);//校验位求和
	hex_buff[Packet.Packet_len-1]=Packet.Cheek_sum=0x5E;
	
	wireless_uart_send_buffer((unsigned char*)hex_buff,Packet.Packet_len);
	//HAL_UART_Transmit(&huart1,(unsigned char*)hex_buff,Packet.Packet_len,10);

}
//使用联合体进行浮点到四字节转换
typedef union
{
	float fdata;
	unsigned long ldata;

}FloatLongType;
//串口发送PID参数,包括Kp，Ki，Kd
void Sent_PID_Parameter(unsigned char PID_CHANNEL,float Kp,float Ki,float Kd)
{
	char str_buff[32],hex_buff[128];
	struct PID_Packet Packet;
	FloatLongType FKp,FKd,FKi;
	FKp.fdata=Kp;
	FKd.fdata=Kd;
	FKi.fdata=Ki;
	
	Packet.Packet_Head=PID_PackHead;
	Packet.Packet_address=PID_CHANNEL;
	Packet.Packet_cmd=Set_Computer_PID;
	Packet.Packet_len=0x17;
	Packet.Cheek_sum=0;
	
	Str_turn_Hex(&Packet,hex_buff);
	
	//处理目标值数据
	for(unsigned char i=Packet.Packet_len-2;i>=18;i--)
		{
			hex_buff[i]=(unsigned char)(FKd.ldata>>(8*(i-18)));
		}
	for(unsigned char i=17;i>=14;i--)
	{
			hex_buff[i]=(unsigned char)(FKi.ldata>>(8*(i-14)));
	}
	for(unsigned char i=13;i>=10;i--)
	{
			hex_buff[i]=(unsigned char)(FKp.ldata>>(8*(i-10)));;
	}
	Cheek_Sum(&Packet,hex_buff);//校验位求和
	wireless_uart_send_buffer((unsigned char*)hex_buff,Packet.Packet_len);
	//HAL_UART_Transmit(&huart1,(unsigned char*)hex_buff,Packet.Packet_len,100);

}


//发送PID目标值
//PID_CHANNEL pid通道地址
//target_value目标值
void Sent_PID_Target(unsigned char PID_CHANNEL ,int target_value)
{
	char str_buff[32],hex_buff[128];
	struct PID_Packet Packet;
	Packet.Packet_Head=PID_PackHead;
	Packet.Packet_address=PID_CHANNEL;
	Packet.Packet_cmd=Set_PID_Target;
	Packet.Packet_len=0x0F;
	Packet.Cheek_sum=0;
	
	Str_turn_Hex(&Packet,hex_buff);//初始化包头数据
	
	//处理目标值数据
	for(unsigned char i=Packet.Packet_len-2;i>=10;i--)
		{
			hex_buff[i]=((target_value&(0xFF000000>>(8*(Packet.Packet_len-2-i))))>>(8*(i-10)))&0xFF;
		}
	Cheek_Sum(&Packet,hex_buff);//校验位求和
	wireless_uart_send_buffer((unsigned char*)hex_buff,Packet.Packet_len);
		//HAL_UART_Transmit(&huart1,(unsigned char*)hex_buff,Packet.Packet_len,100);
//	
}
//发送实际值
//PID_CHANNELpid通道
//present_value目前实际值
void Sent_PID_Present(unsigned char PID_CHANNEL ,int present_value)
{
	char str_buff[32],hex_buff[128];
	struct PID_Packet Packet;
	Packet.Packet_Head=PID_PackHead;
	Packet.Packet_address=PID_CHANNEL;
	Packet.Packet_cmd=Set_PID_present;
	Packet.Packet_len=0x0F;
	Packet.Cheek_sum=0;
	
	Str_turn_Hex(&Packet,hex_buff);
	
	//处理目标值数据
	for(unsigned char i=Packet.Packet_len-2;i>=10;i--)
		{
			hex_buff[i]=((present_value&(0xFF000000>>(8*(Packet.Packet_len-2-i))))>>(8*(i-10)))&0xFF;
		}
	Cheek_Sum(&Packet,hex_buff);//校验位求和
	wireless_uart_send_buffer((unsigned char*)hex_buff,Packet.Packet_len);
//	
}
//发送PID周期
void Sent_PID_Cycle(unsigned char PID_CHANNEL ,int Cycle_value)
{
	char str_buff[32],hex_buff[128];
	struct PID_Packet Packet;
	Packet.Packet_Head=PID_PackHead;
	Packet.Packet_address=PID_CHANNEL;
	Packet.Packet_cmd=Set_Computer_T;
	Packet.Packet_len=0x0F;
	Packet.Cheek_sum=0;
	
	Str_turn_Hex(&Packet,hex_buff);
	
	//处理目标值数据
	for(unsigned char i=Packet.Packet_len-2;i>=10;i--)
		{
			hex_buff[i]=((Cycle_value&(0xFF000000>>(8*(Packet.Packet_len-2-i))))>>(8*(i-10)))&0xFF;
		}
	Cheek_Sum(&Packet,hex_buff);//校验位求和
	wireless_uart_send_buffer((unsigned char*)hex_buff,Packet.Packet_len);
//	
}

void PID_UsartInit()
{
	  wireless_uart_init();
		Uart_PID_Start(PID_CH1);//开启pid通道1
		Uart_PID_Start(PID_CH2);//开启pid通道1
		Uart_PID_Start(PID_CH3);//开启pid通道1
		Uart_PID_Start(PID_CH4);//开启pid通道1

}
/******************************************************************************/