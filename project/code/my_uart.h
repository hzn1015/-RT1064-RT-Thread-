# ifndef _MY_UART_H
# define _MY_UART_H


void rt1064_to_openartmini_init();
void rt1064_ReadList(uint8 *receivebuff);

extern uint8 uart4_get_data[2048];                                                    // 串口接收数据缓冲区
extern uint8 fifo4_get_data[2048];                                                    // fifo 输出读出缓冲区
extern uint8 uart1_get_data[64];                                                    // 串口接收数据缓冲区
extern uint8 fifo1_get_data[64];                                                    // fifo 输出读出缓冲区
                                                           // 接收数据变量
extern uint32 fifo_uart4data_count;                                                     // fifo 数据个数
extern uint32 fifo_uart1data_count;        
extern fifo_struct uart4_data_fifo;                                                // 是fifo内的
extern fifo_struct uart1_data_fifo;  

typedef enum                                                                    // 枚举串口号 此枚举定义不允许用户修改
{
    Send_flag1 = 1,
    Send_flag2,
	Receive_flag1,
	Receive_flag2 = 4
}flag_enum;

void uart4_rx_interrupt_handler (void);
void uart1_rx_interrupt_handler (void) ;
void UartGetPosition(uint8 * data,uint16 Datalen,uint8 *Kind ,uint16 * out_camera_x,uint16 * out_camera_y);
# endif