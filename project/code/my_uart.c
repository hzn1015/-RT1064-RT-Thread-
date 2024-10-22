#include "zf_common_headfile.h"

//fifoִ�������ȳ�����������ԭ��
//��Ҫ��fifo�ж���buff���ݻ�����
//�ⲿ--get_data--(д��fifo)--uart4_data_fifo--(����fifo)--fifo_get_data

uint8 uart4_get_data[2048];                                                        // ���ڽ������ݻ�����
uint8 fifo4_get_data[2048];                                                        // fifo �������������

uint8 uart1_get_data[64];                                                        // ���ڽ������ݻ�����
uint8 fifo1_get_data[64];                                                        // fifo �������������



uint32 fifo_uart4data_count = 0;                                                     // fifo ���ݸ���
uint32 fifo_uart1data_count = 0;   
fifo_struct uart4_data_fifo;                                                    // uart4�����ݽ��ջ���
fifo_struct uart1_data_fifo;  

void rt1064_to_openartmini_init()                                               // ʹ�ô���4��Ϊ1064��art��ͨ�Ŵ���
{
	fifo_init(&uart4_data_fifo, FIFO_DATA_8BIT, uart4_get_data, 64);             // fifo��ʼ��
	fifo_init(&uart1_data_fifo, FIFO_DATA_8BIT, uart1_get_data, 64);             // fifo��ʼ��
	uart_init(UART_4, 115200, UART4_TX_C16, UART4_RX_C17);                      // uart4��ʼ����������Ϊ115200         
	uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);                      // uart1��ʼ����������Ϊ115200                                    	// �жϳ�ʼ��
	uart_rx_interrupt(UART_4, ZF_ENABLE);                                       // ����uart4�Ľ����жϣ��ں����ڲ��Ѷ��ж�ʹ��
	uart_rx_interrupt(UART_1, ZF_ENABLE);                                       // ����uart4�Ľ����жϣ��ں����ڲ��Ѷ��ж�ʹ��    
	interrupt_set_priority(LPUART4_IRQn, 5);                                    // �����ж����ȼ���0-15ԽСԽ��
	interrupt_set_priority(LPUART1_IRQn, 5);                                    // �����ж����ȼ���0-15ԽСԽ��
}

void uart4_rx_interrupt_handler (void)   
{ 
    uint8 get_data = 0;                                                             // �������ݱ���
		uint8 flag=0;
		flag=uart_query_byte(UART_4, &get_data);//��ѯʽ �����ݻ᷵�� TRUE û�����ݻ᷵�� FALSE           
	//���ݻ�ͨ����ַ�浽get_data
		if(flag==1)
    {
			fifo_write_buffer(&uart4_data_fifo, &get_data, 1);   
		}                 // �����ݴ�get_data��д�� fifo ��

}

void uart1_rx_interrupt_handler (void)   
{ 
    uint8 get_data = 0;                                                             // �������ݱ���
		uint8 flag=0;
		flag=uart_query_byte(UART_1, &get_data);//��ѯʽ �����ݻ᷵�� TRUE û�����ݻ᷵�� FALSE           
	//���ݻ�ͨ����ַ�浽get_data
		if(flag==1)
    {
			fifo_write_buffer(&uart1_data_fifo, &get_data, 1);   
			
		}                 // �����ݴ�get_data��д�� fifo ��

}
//�Ӵ��ڽ����ȡ,������������
void UartGetPosition(uint8 * data,uint16 Datalen,uint8 *Kind ,uint16 * out_camera_x,uint16 * out_camera_y)
{
	uint16 x=0,y=0;
	uint16 kind=0;
	union 
	{
		char byte[2];
		short value;
	}Decode;
	if(Datalen<9)//������ݳ���С��9ֱ�Ӳ�������
		return ;
	for(uint8 i=0;i<Datalen;i++)
	{
		if(data[i]==0x19&&data[i+1]==0x10&&data[i+2]==0x15)//֡ͷ���
		{
			i=i+2;//�±��ƶ�������
			//���֡β
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
	*Kind=kind;//��ȡ������
	

}



