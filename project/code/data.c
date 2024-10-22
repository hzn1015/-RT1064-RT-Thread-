#include "data.h"



//��е�۽Ƕ�

uint16 Camera_x=0;//��������
uint16 Camera_y=0;
uint16 CartKind=0;
int real_x=0;//ʵ������
int real_y=0;

struct Chess_Data ChessBoard[3][3];//����һ��3*3�����̣�ÿ������λ���ж�Ӧ������Ϣ
uint8 board[3][3];//������ʱ���������ڴ���һ�ε�����
float BoardPoints[9][2];

uint8 board_Buff[3][3];//���̻���
float BoardBuffPoints[9][2];//�������껺��


uint8 ChessState=0;//����״̬��0�����ˣ�1��
uint8 ChessPositionIndex=1;//ѡ������λ�ã�1-9��Ĭ��Ϊ1

uint8 Max_Robot=Black_Chess;//����Ĭ�Ϻ�ɫ����
uint8 Min_People=White_Chess;//����Ĭ�Ϻ�ɫ����
float BlacePiece[10][2];
uint8 BlackPieceCount=0;
float WhitePiece[10][2];
uint8 WhitePieceCount=0;
float InitChessPosition[9][2];

float ChessAngle=0;
//���Ʋ�����Χ����,
int clip(int x, int min, int max) {
	
	if(x>=max)
		x=max;
	else if(x<=min)
		x=min;
    return x;
}

// �����Ϸ�Ƿ����  
int checkWin() { 

	
	 for (int i = 0; i < 3; i++) 
		{
        if (ChessBoard[i][0].Chess_State == ChessBoard[i][1].Chess_State && ChessBoard[i][1].Chess_State == ChessBoard[i][2].Chess_State && ChessBoard[i][0].Chess_State != Empty) {
            return ChessBoard[i][0].Chess_State;
        }
				 if (ChessBoard[0][i].Chess_State == ChessBoard[1][i].Chess_State && ChessBoard[1][i].Chess_State == ChessBoard[2][i].Chess_State && ChessBoard[0][i].Chess_State != Empty) {
            return ChessBoard[0][i].Chess_State;
        }
				 
    }
		
    if (ChessBoard[0][0].Chess_State == ChessBoard[1][1].Chess_State && ChessBoard[1][1].Chess_State == ChessBoard[2][2].Chess_State && ChessBoard[0][0].Chess_State != Empty) 
		{
        return ChessBoard[0][0].Chess_State;
    }
    if (ChessBoard[0][2].Chess_State == ChessBoard[1][1].Chess_State && ChessBoard[1][1].Chess_State == ChessBoard[2][0].Chess_State && ChessBoard[0][2].Chess_State != Empty) {
        return ChessBoard[0][2].Chess_State;
    }
    return Empty;
	
}
//����Ƿ�ƽ��
uint8 isDraw() 
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (ChessBoard[i][j].Chess_State == Empty) {
                return 0;
            }
        }
    }
    return 1;
}
// ������ǰ״̬������ʤ���ߣ�-1, 0, 1��  
int evaluate(char player) {  
    if (checkWin(Black_Chess)) return Black_Chess;  
    if (checkWin(White_Chess)) return White_Chess;  
    for (int i = 0; i < 3; i++)  
        for (int j = 0; j < 3; j++)  
            if (ChessBoard[i][j].Chess_State == Empty) return 0;//��ʾƽ��  
    return 0; // ƽ��  
} 

// MiniMax�㷨
int minimax(uint8 isMaximizing) {
    char winner = checkWin();
    if (winner == Min_People) return -10;
    if (winner == Max_Robot) return 10;
    if (isDraw()) return 0;

    if (isMaximizing) {
        int bestScore = -1000;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (ChessBoard[i][j].Chess_State == Empty) {
                    ChessBoard[i][j].Chess_State = Max_Robot;
                    int score = minimax(false);
                    ChessBoard[i][j].Chess_State = Empty;
                    if (score > bestScore) {
                        bestScore = score;
                    }
                }
            }
        }
        return bestScore;
    }
    else {
        int bestScore = 1000;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (ChessBoard[i][j].Chess_State == Empty) {
                    ChessBoard[i][j].Chess_State = Min_People;
                    int score = minimax(true);
                    ChessBoard[i][j].Chess_State = Empty;
                    if (score < bestScore) {
                        bestScore = score;
                    }
                }
            }
        }
        return bestScore;
    }
}
// ��������ƶ�
uint8 bestMove() {
    int bestScore = -1000;
    int move[2];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (ChessBoard[i][j].Chess_State == Empty) {
                ChessBoard[i][j].Chess_State = Max_Robot;
                int score = minimax(false);
                ChessBoard[i][j].Chess_State = Empty;
                if (score > bestScore) {
                    bestScore = score;
                    move[0] = i;
                    move[1] = j;
                }
            }
        }
    }
		
   // ChessBoard[move[0]][move[1]].Chess_State = Max_Robot;
		return move[0]*3+move[1]+1;//�����������
		
}

/*��ʼ������***********/
void InitChessBoard()
{

	unsigned char Index=1;
	float x_bias=-1,y_bias=-0.2;
	/***��һ����������****/
	ChessBoard[0][0].Y_Position=y_bias+27.805;
	ChessBoard[0][0].X_Position=x_bias+29.290;
	                             
	ChessBoard[0][1].Y_Position=y_bias+29.64;
	ChessBoard[0][1].X_Position=x_bias+26.668;
	                             

	ChessBoard[0][2].Y_Position=y_bias+31.476;
	ChessBoard[0][2].X_Position=x_bias+24.047;
/***********************************/
	/****�ڶ�����������***************/
	ChessBoard[1][2].Y_Position=y_bias+28.855;
	ChessBoard[1][2].X_Position=x_bias+22.212;
	                             
	ChessBoard[1][1].Y_Position=y_bias+27.019;
	ChessBoard[1][1].X_Position=x_bias+24.833;
	                             
	ChessBoard[1][0].Y_Position=y_bias+25.184;
	ChessBoard[1][0].X_Position=x_bias+27.454;
	/***********************************/
	/****��������������***************/
	ChessBoard[2][2].Y_Position=y_bias+26.233;
	ChessBoard[2][2].X_Position=x_bias+20.376;
	                             
	ChessBoard[2][1].Y_Position=y_bias+24.398;
	ChessBoard[2][1].X_Position=x_bias+22.998;
	                             
	ChessBoard[2][0].Y_Position=y_bias+22.562;
	ChessBoard[2][0].X_Position=x_bias+25.619;
	for(uint8 i=0;i<3;i++)
		for(uint8 j=0;j<3;j++)
		{
//			ChessBoard[i][j].X_Position=0;
//			ChessBoard[i][j].Y_Position=0;
			ChessBoard[i][j].Chess_State=Empty;
			ChessBoard[i][j].Chess_Index=Index;
			InitChessPosition[Index-1][0]=ChessBoard[i][j].X_Position;
			InitChessPosition[Index-1][1]=ChessBoard[i][j].Y_Position;
			Index++;
			
		}
		
		/*����һ����ɫ����λ��**/
		BlacePiece[0][0]=36;
		BlacePiece[0][1]=25;
		
		BlacePiece[1][0]=30;
		BlacePiece[1][1]=20;
		
		WhitePiece[0][0]=25;
		WhitePiece[0][1]=25;
		
		WhitePiece[1][0]=20;
		WhitePiece[1][1]=30;
		

		
}
//���庯��
//��������ǰ������ɫ�����ӱ��
uint8 PlayChess(uint8 State,uint8 Index)
{
	for(uint8 i=0;i<3;i++)
	{
		for(uint8 j=0;j<3;j++)
		{
			if(ChessBoard[i][j].Chess_Index==Index&&ChessBoard[i][j].Chess_State==Empty)//��Ŷ�Ӧ���Ҹ�λ��Ϊ��λ
			{
				ChessBoard[i][j].Chess_State=State;
				return 1;
			}
		
		}
	}
	return 0;

}
//���ȫ������
uint8 ClearAllChess()
{
		for(uint8 i=0;i<3;i++)
	{
		for(uint8 j=0;j<3;j++)
		{
				ChessBoard[i][j].Chess_State=Empty;	
		}
	}

}

//�������״̬������λ��
uint8 CheckChessPiece()
{
	for(uint8 i=0;i<3;i++)
	{
		for(uint8 j=0;j<3;j++)
		{
				if(ChessBoard[i][j].Chess_State==Empty)//��������λ���ǿյĲ��ܸ�������
				{
					ChessBoard[i][j].Chess_State=board[i][j];//���������������
				}
		}
	}

}

//��������״̬�����ڸ���λ��
uint8 CheckSavePieceBuff()
{
	for(uint8 i=0;i<3;i++)
	{
		for(uint8 j=0;j<3;j++)
		{
				board[i][j]=ChessBoard[i][j].Chess_State;//���������������
		}
	}

}
//�������ǰ���״̬
uint8 CheckChess()
{


}
//��ȡ���ߵ����ӣ����壬����ĳ�ʼλ��
uint8 GainSidePiece(uint8 * data,uint16 Datalen)
{
	uint16 BlackCameraBuffer[5][2];//��ɫ�����������껺��
	uint16 WhiteCameraBuffer[5][2];//��ɫ�����������껺��
	
	uint16 BlackCount=0;//��ɫ��������
	uint16 WhiteCount=0;//��ɫ��������
	
	uint16 DataCount=0;
	
	float BlackBuffer[5][2];//��ɫ�������껺��
	float WhiteBuffer[5][2];//��ɫ�������껺��
	//��ȡ������������
	union 
	{
		char byte[2];
		short value;
	}Decode;
	//rt_kprintf("A");
	while(DataCount<=Datalen)
	{
	//	rt_kprintf("%c",data[DataCount]);
		if(data[DataCount]==0x19&&data[DataCount+1]==0x10&&data[DataCount+2]==0x15)//У��֡ͷ���������֡ͷҪ��
		{
		//	rt_kprintf("B");
			DataCount=DataCount+3;//����ǰ����λ
			//��ʼ��ȡ���꣬�����껺�浽����������
			if(data[DataCount]==0x88)//��ʾ��������������
			{
			//	rt_kprintf("C");
				DataCount++;
				while(data[DataCount]!=0xFF)//ֱ����ȡ��֡β
				{
					//rt_kprintf("D");
					if(DataCount>Datalen)
						break;
					if(data[DataCount]=='B')//��ɫ
					{
						uint16 x=0,y=0;
						Decode.byte[0]=data[DataCount+1];
						Decode.byte[1]=data[DataCount+2];
						x=Decode.value;
				
						Decode.byte[0]=data[DataCount+3];
						Decode.byte[1]=data[DataCount+4];
						y=Decode.value;
						//rt_kprintf("BlackPoints,x=%d,y=%d\n",x,y);
						BlackCameraBuffer[BlackCount][0]=x;
						BlackCameraBuffer[BlackCount][1]=y;
						BlackCount++;
						DataCount=DataCount+5;
					}
					else if(data[DataCount]=='W')
					{
						uint16 x=0,y=0;
						Decode.byte[0]=data[DataCount+1];
						Decode.byte[1]=data[DataCount+2];
						x=Decode.value;
				
						Decode.byte[0]=data[DataCount+3];
						Decode.byte[1]=data[DataCount+4];
						y=Decode.value;
						
						//rt_kprintf("WhitePoints,x=%d,y=%d\n",x,y);
						WhiteCameraBuffer[WhiteCount][0]=x;
						WhiteCameraBuffer[WhiteCount][1]=y;
						WhiteCount++;
						DataCount=DataCount+5;
					}
					else
						DataCount++;
				}
				
			}
			
		
		}
		if(data[DataCount]==0xff)
			break;
		DataCount++;
		
	
	}
	//�����������Ƿ񶼻�����

	
//	//����������ϵתΪȫ������ϵ
	if(BlackCount>BlackPieceCount||GradeFlag==1)//�������������ϴ������Ÿ���
	{
			for(uint8 i=0;i<BlackCount;i++)//ת��������
		{
		LeftCameraTurnPosition(BlackCameraBuffer[i][0],BlackCameraBuffer[i][1],&BlackBuffer[i][0],&BlackBuffer[i][1]);
		BlacePiece[i][0]=BlackBuffer[i][0];
		BlacePiece[i][1]=BlackBuffer[i][1];
		}
		
			BlackPieceCount=BlackCount;
	}
	if(WhiteCount>WhitePieceCount||GradeFlag==1)
	{
			for(uint8 i=0;i<WhiteCount;i++)//ת��������
		{
			LeftCameraTurnPosition(WhiteCameraBuffer[i][0],WhiteCameraBuffer[i][1],&WhiteBuffer[i][0],&WhiteBuffer[i][1]);
			WhitePiece[i][0]=WhiteBuffer[i][0];
			WhitePiece[i][1]=WhiteBuffer[i][1];
		}
			WhitePieceCount=WhiteCount;
	}



//	for(uint8 i=0;i<BlackCount;i++)
//	{
//		rt_kprintf("BlackBuffer[%d],x=%d,y=%d\n",i,(int)BlackBuffer[i][0],(int)BlackBuffer[i][1]);
//	}
//	
//	for(uint8 i=0;i<WhiteCount;i++)
//	{
//		rt_kprintf("WhiteBuffer[%d],x=%d,y=%d\n",i,(int)WhiteBuffer[i][0],(int)WhiteBuffer[i][1]);
//	}
//	

}

//���ݻ�����ӵ�ȫ������ϵ��ȡ�����������̵�λ��
void GainChessIndex(uint8 board[3][3],float boardpoints[9][2] ,float Point[2],uint8 State)
{
	//������������λ��
	float Dis=0;
	
	for(uint8 i=0;i<3;i++)
	{
		for(uint8 j=0;j<3;j++)
		{

					Dis=(Point[0]-ChessBoard[i][j].X_Position)*(Point[0]-ChessBoard[i][j].X_Position)+(Point[1]-ChessBoard[i][j].Y_Position)*(Point[1]-ChessBoard[i][j].Y_Position);
					Dis=sqrt(Dis);
					//rt_kprintf("Dis=%d\n",(int)Dis);
					//���������Ƿ�������λ�õ����ķ�Χ��
					if(Dis<Chess_width*1.414*0.5)//�������ڸ÷�Χ��
					{
						board[i][j]=State;//����λ����Ϊ���ӵ���ɫ
						//������һ�����̵�����
						boardpoints[i*3+j][0]=Point[0];
						boardpoints[i*3+j][1]=Point[1];
						
					}				
		}
	}
	

}
/***��ȡ�����м������***************/
uint8 GainChessPiece(uint8 * data,uint16 Datalen)
{
	
	uint16 BlackCameraBuffer[200][2];//��ɫ�����������껺��
	uint16 WhiteCameraBuffer[200][2];//��ɫ�����������껺��
	
	uint16 BlackCount=0;//��ɫ��������
	uint16 WhiteCount=0;//��ɫ��������
	
	uint16 DataCount=0;
	
	uint8 board_buff[3][3]={0};//����״̬����
	float board_buff_points[9][2];
	float BlackBuffer[200][2];//��ɫ�������껺��
	float WhiteBuffer[200][2];//��ɫ�������껺��
	//��ȡ������������
	union 
	{
		char byte[2];
		short value;
	}Decode;
	while(DataCount<=Datalen)
	{
		if(data[DataCount]==0x19&&data[DataCount+1]==0x10&&data[DataCount+2]==0x15)//У��֡ͷ���������֡ͷҪ��
		{
			DataCount=DataCount+3;//����ǰ����λ
			if(data[DataCount]==0x66)//��ʾ��������������
			{
				DataCount++;
				while(data[DataCount]!=0xFF)//ֱ����ȡ��֡β
				{
					if(DataCount>Datalen)
						break;
					if(data[DataCount]=='B')//��ɫ
					{
						uint16 x=0,y=0;
						Decode.byte[0]=data[DataCount+1];
						Decode.byte[1]=data[DataCount+2];
						x=Decode.value;
				
						Decode.byte[0]=data[DataCount+3];
						Decode.byte[1]=data[DataCount+4];
						y=Decode.value;
						//rt_kprintf("BlackPoints,x=%d,y=%d\n",x,y);
						BlackCameraBuffer[BlackCount][0]=x;
						BlackCameraBuffer[BlackCount][1]=y;
						BlackCount++;
						DataCount=DataCount+5;
					}
					else if(data[DataCount]=='W')
					{
						uint16 x=0,y=0;
						Decode.byte[0]=data[DataCount+1];
						Decode.byte[1]=data[DataCount+2];
						x=Decode.value;
				
						Decode.byte[0]=data[DataCount+3];
						Decode.byte[1]=data[DataCount+4];
						y=Decode.value;
						
						WhiteCameraBuffer[WhiteCount][0]=x;
						WhiteCameraBuffer[WhiteCount][1]=y;
						WhiteCount++;
						DataCount=DataCount+5;
					}
					else
						DataCount++;
				}
				
			}
			
		
		}
		DataCount++;
		
	
	}
	
	
//	//�����������Ƿ񶼻�����
//	
//	for(uint8 i=0;i<BlackCount;i++)
//	{
//		rt_kprintf("BlackCameraBuffer[%d],x=%d,y=%d\n",i,(int)BlackCameraBuffer[i][0],(int)BlackCameraBuffer[i][1]);
//	}
//	
//	for(uint8 i=0;i<WhiteCount;i++)
//	{
//		rt_kprintf("WhiteCameraBuffer[%d],x=%d,y=%d\n",i,(int)WhiteCameraBuffer[i][0],(int)WhiteCameraBuffer[i][1]);
//	}
//	
	
//	//����������ϵתΪȫ������ϵ
		for(uint8 i=0;i<BlackCount;i++)//ת��������
		{
			LeftCameraTurnPosition(BlackCameraBuffer[i][0],BlackCameraBuffer[i][1],&BlackBuffer[i][0],&BlackBuffer[i][1]);
			GainChessIndex(board_buff,board_buff_points,BlackBuffer[i],Black_Chess);
			
		}
			for(uint8 i=0;i<WhiteCount;i++)//ת��������
		{
			LeftCameraTurnPosition(WhiteCameraBuffer[i][0],WhiteCameraBuffer[i][1],&WhiteBuffer[i][0],&WhiteBuffer[i][1]);
			GainChessIndex(board_buff,board_buff_points,WhiteBuffer[i],White_Chess);
		}
		
		//�������̵�ǰ����״̬
		for(uint8 i=0;i<3;i++)
		{
			for(uint8 j=0;j<3;j++)
			{
				//rt_kprintf("board_buff[%d][%d]=%d\n",i,j,(int)board_buff[i][j]);
				board_Buff[i][j]=board_buff[i][j];		
				BoardBuffPoints[i*3+j][0]=board_buff_points[i*3+j][0];//�����ȡ����������				
				BoardBuffPoints[i*3+j][1]=board_buff_points[i*3+j][1];//�����ȡ����������		
			}
		}
		
	
}

uint8 Communication_State=1;//ͨ��״̬��0���У�1��ʼ�����źŸ�openart���֣�2���ֳɹ���ʼ��ȡ���������������꣬3�������������ĵ�����
uint8 GradeFlag=1;//���±�־λ������Ŀ���ص�����״̬ʱ���Ը�������
uint8 count=0;
void GainCartPosition()
{
		if(Communication_State==1)//����0xBB��openart�������̻�ȡ
		{
			count++;
			if(count==1)
			{
				count=0;
				uart_write_byte(UART_4,0xBB);
			}

		}
		fifo_uart4data_count = fifo_used(&uart4_data_fifo);                           // �鿴 fifo �Ƿ�������                       // �鿴 fifo �Ƿ�������
		fifo_read_buffer(&uart4_data_fifo, fifo4_get_data, &fifo_uart4data_count, FIFO_READ_AND_CLEAN);    // �� fifo �����ݶ�������� fifo ���صĻ���

		if(Communication_State==1)//����artӦ��λ������һ��״̬
		{
			
			if(fifo_uart4data_count>0)
			{
				
				for(uint8 i=0;i<fifo_uart4data_count;i++)
				{
				//	rt_kprintf("%c",fifo4_get_data[i]);
					
					if(fifo4_get_data[i]==0xee)//������յ�Ӧ������ȡ����λ��״̬
					{
					//	rt_kprintf("Please OK\n");
						Communication_State=2;
						break;
					}
				}
				//rt_kprintf("\n");
			
			}
		
		}
		else if(Communication_State==2)
		{
			if(fifo_uart4data_count!=0)//��ʾ���յ�����
			{
				if(TopicIndex==1||TopicIndex==2||TopicIndex==3)
				{
						GainSidePiece(fifo4_get_data,fifo_uart4data_count);//��ȡ����
						//GainChessPiece(fifo4_get_data,fifo_uart4data_count);
				}
				else if(TopicIndex==4)
				{
					if(Topic_4_Flag==0)//��Ŀû��ʼ��ʱ��Ż�ȡ���������
					{
						GainSidePiece(fifo4_get_data,fifo_uart4data_count);//��ȡ����
					}
					else
					{
						if((Topic_4_Chess_Order==0&&Topic_4_PeopleStartFlag==1)||(Topic_4_Chess_Order==1))//�ֵ�����,��ȡ��������
						{
						GainChessPiece(fifo4_get_data,fifo_uart4data_count);
						
						}
					}

						
				}
				else if(TopicIndex==5)//������
				{
					if(Topic_5_Flag==0)//�����⻹û��ʼ��ʱ���ȡ�ⲿ��������
					{
						GainSidePiece(fifo4_get_data,fifo_uart4data_count);//��ȡ����
					}
					else//�����Ŀ5�Ѿ���ʼ�ˣ���ֻ��������ȷ�Ϻ��ȡ�����м�����
					{
						if(Topic_5_Chess_Order==0&&Topic_5_PeopleStartFlag==1)//�ֵ������£��������Ѿ�������
						{
							GainChessPiece(fifo4_get_data,fifo_uart4data_count);
						}
						
					}
				}

			}
		
		}
		if(fifo_uart4data_count==9)//��ʾ���յ�����
		{
				UartGetPosition(fifo4_get_data,fifo_uart4data_count,&CartKind,&Camera_x,&Camera_y);//�����ȡ��Ƭ�����Լ�����		
				//����ƫ�����������Ƭ�ڻ�е�۷�Χ����Ҫƫ��
			
		}


}

float ADC_TurnAngle(uint16 ADC)
{
	float K=-0.09483;
	float Angle=0;
	Angle=K*(ADC-ADC_Angle_Middle);
	return Angle;

}

//����ѡ�����ֵ��������λ������
void GainChessPosition(float Angle)
{
	Angle=-Angle*3.1415916/180;

	uint8 Index=0;
		for(uint8 i=0;i<3;i++)
		for(uint8 j=0;j<3;j++)
		{
			
			ChessBoard[i][j].X_Position=(InitChessPosition[Index][0]-InitChessPosition[4][0])*(cos(Angle))+(InitChessPosition[Index][1]-InitChessPosition[4][1])*sin(Angle)+InitChessPosition[4][0];
			ChessBoard[i][j].Y_Position=(InitChessPosition[Index][1]-InitChessPosition[4][1])*(cos(Angle))-(InitChessPosition[Index][0]-InitChessPosition[4][0])*sin(Angle)+InitChessPosition[4][1];
			Index++;
			
		}

}
