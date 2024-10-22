#include "zf_common_headfile.h"

//fifo执行先入先出，后入后出的原则
//需要从fifo中读入buff数据缓冲区
//外部--get_data--(写入fifo)--uart4_data_fifo--(读出fifo)--fifo_get_data

uint8 uart4_get_data[2048];                                                        // 串口接收数据缓冲区
uint8 fifo4_get_data[2048];                                                        // fifo 输出读出缓冲区

uint8 uart1_get_data[64];                                                        // 串口接收数据缓冲区
uint8 fifo1_get_data[64];                                                        // fifo 输出读出缓冲区



uint32 fifo_uart4data_count = 0;                                                     // fifo 数据个数
uint32 fifo_uart1data_count = 0;   
fifo_struct uart4_data_fifo;                                                    // uart4的数据接收缓冲
fifo_struct uart1_data_fifo;  

void rt1064_to_openartmini_init()                                               // 使用串口4作为1064与art的通信串口
{
	fifo_init(&uart4_data_fifo, FIFO_DATA_8BIT, uart4_get_data, 64);             // fifo初始化
	fifo_init(&uart1_data_fifo, FIFO_DATA_8BIT, uart1_get_data, 64);             // fifo初始化
	uart_init(UART_4, 115200, UART4_TX_C16, UART4_RX_C17);                      // uart4初始化，波特率为115200         
	uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);                      // uart1初始化，波特率为115200                                    	// 中断初始化
	uart_rx_interrupt(UART_4, ZF_ENABLE);                                       // 开启uart4的接收中断，在函数内部已对中断使能
	uart_rx_interrupt(UART_1, ZF_ENABLE);                                       // 开启uart4的接收中断，在函数内部已对中断使能    
	interrupt_set_priority(LPUART4_IRQn, 5);                                    // 设置中断优先级，0-15越小越高
	interrupt_set_priority(LPUART1_IRQn, 5);                                    // 设置中断优先级，0-15越小越高
}

void uart4_rx_interrupt_handler (void)   
{ 
    uint8 get_data = 0;                                                             // 接收数据变量
		uint8 flag=0;
		flag=uart_query_byte(UART_4, &get_data);//查询式 有数据会返回 TRUE 没有数据会返回 FALSE           
	//数据会通过地址存到get_data
		if(flag==1)
    {
			fifo_write_buffer(&uart4_data_fifo, &get_data, 1);   
		}                 // 将数据从get_data中写入 fifo 中

}

void uart1_rx_interrupt_handler (void)   
{ 
    uint8 get_data = 0;                                                             // 接收数据变量
		uint8 flag=0;
		flag=uart_query_byte(UART_1, &get_data);//查询式 有数据会返回 TRUE 没有数据会返回 FALSE           
	//数据会通过地址存到get_data
		if(flag==1)
    {
			fifo_write_buffer(&uart1_data_fifo, &get_data, 1);   
			
		}                 // 将数据从get_data中写入 fifo 中

}
//从串口解码获取,类别和像素坐标
void UartGetPosition(uint8 * data,uint16 Datalen,uint8 *Kind ,uint16 * out_camera_x,uint16 * out_camera_y)
{
	uint16 x=0,y=0;
	uint16 kind=0;
	union 
	{
		char byte[2];
		short value;
	}Decode;
	if(Datalen<9)//如果数据长度小于9直接不做解码
		return ;
	for(uint8 i=0;i<Datalen;i++)
	{
		if(data[i]==0x19&&data[i+1]==0x10&&data[i+2]==0x15)//帧头检测
		{
			i=i+2;//下标移动到数据
			//检查帧尾
			if(data[i+6]==0xff)
			{
				
				kind=data[i+1];
				
				Decode.byte[0]=data[i+2];
				Decode.byte[1]=data[i+3];
				x=Decode.value;
				
				Decode.byte[0]=data[i+4];
				Decode.byte[1]=data[i+5];
				y=Decode.value;
				
				break;			
			}
			
		}
		
	}
	*out_camera_x=x;
	*out_camera_y=y;
	*Kind=kind;//获取类别序号
	

}



