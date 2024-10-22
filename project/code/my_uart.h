# ifndef _MY_UART_H
# define _MY_UART_H


void rt1064_to_openartmini_init();
void rt1064_ReadList(uint8 *receivebuff);

extern uint8 uart4_get_data[2048];                                                    // ���ڽ������ݻ�����
extern uint8 fifo4_get_data[2048];                                                    // fifo �������������
extern uint8 uart1_get_data[64];                                                    // ���ڽ������ݻ�����
extern uint8 fifo1_get_data[64];                                                    // fifo �������������
                                                           // �������ݱ���
extern uint32 fifo_uart4data_count;                                                     // fifo ���ݸ���
extern uint32 fifo_uart1data_count;        
extern fifo_struct uart4_data_fifo;                                                // ��fifo�ڵ�
extern fifo_struct uart1_data_fifo;  

typedef enum                                                                    // ö�ٴ��ں� ��ö�ٶ��岻�����û��޸�
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