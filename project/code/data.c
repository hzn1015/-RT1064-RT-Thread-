#include "data.h"



//机械臂角度

uint16 Camera_x=0;//像素坐标
uint16 Camera_y=0;
uint16 CartKind=0;
int real_x=0;//实际坐标
int real_y=0;

struct Chess_Data ChessBoard[3][3];//定义一个3*3的棋盘，每个棋盘位置有对应棋盘信息
uint8 board[3][3];//棋盘临时变量，用于存上一次的坐标
float BoardPoints[9][2];

uint8 board_Buff[3][3];//棋盘缓存
float BoardBuffPoints[9][2];//棋盘坐标缓存


uint8 ChessState=0;//下棋状态，0机器人，1人
uint8 ChessPositionIndex=1;//选择棋盘位置，1-9，默认为1

uint8 Max_Robot=Black_Chess;//机器默认黑色先手
uint8 Min_People=White_Chess;//机器默认黑色先手
float BlacePiece[10][2];
uint8 BlackPieceCount=0;
float WhitePiece[10][2];
uint8 WhitePieceCount=0;
float InitChessPosition[9][2];

float ChessAngle=0;
//限制参数范围函数,
int clip(int x, int min, int max) {
	
	if(x>=max)
		x=max;
	else if(x<=min)
		x=min;
    return x;
}

// 检查游戏是否结束  
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
//检查是否平局
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
// 评估当前状态，返回胜利者（-1, 0, 1）  
int evaluate(char player) {  
    if (checkWin(Black_Chess)) return Black_Chess;  
    if (checkWin(White_Chess)) return White_Chess;  
    for (int i = 0; i < 3; i++)  
        for (int j = 0; j < 3; j++)  
            if (ChessBoard[i][j].Chess_State == Empty) return 0;//表示平局  
    return 0; // 平局  
} 

// MiniMax算法
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
// 计算最佳移动
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
		return move[0]*3+move[1]+1;//返回棋盘序号
		
}

/*初始化棋盘***********/
void InitChessBoard()
{

	unsigned char Index=1;
	float x_bias=-1,y_bias=-0.2;
	/***第一行棋盘坐标****/
	ChessBoard[0][0].Y_Position=y_bias+27.805;
	ChessBoard[0][0].X_Position=x_bias+29.290;
	                             
	ChessBoard[0][1].Y_Position=y_bias+29.64;
	ChessBoard[0][1].X_Position=x_bias+26.668;
	                             

	ChessBoard[0][2].Y_Position=y_bias+31.476;
	ChessBoard[0][2].X_Position=x_bias+24.047;
/***********************************/
	/****第二行棋盘坐标***************/
	ChessBoard[1][2].Y_Position=y_bias+28.855;
	ChessBoard[1][2].X_Position=x_bias+22.212;
	                             
	ChessBoard[1][1].Y_Position=y_bias+27.019;
	ChessBoard[1][1].X_Position=x_bias+24.833;
	                             
	ChessBoard[1][0].Y_Position=y_bias+25.184;
	ChessBoard[1][0].X_Position=x_bias+27.454;
	/***********************************/
	/****第三行棋盘坐标***************/
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
		
		/*假设一个黑色棋盘位置**/
		BlacePiece[0][0]=36;
		BlacePiece[0][1]=25;
		
		BlacePiece[1][0]=30;
		BlacePiece[1][1]=20;
		
		WhitePiece[0][0]=25;
		WhitePiece[0][1]=25;
		
		WhitePiece[1][0]=20;
		WhitePiece[1][1]=30;
		

		
}
//下棋函数
//参数：当前棋子颜色，棋子编号
uint8 PlayChess(uint8 State,uint8 Index)
{
	for(uint8 i=0;i<3;i++)
	{
		for(uint8 j=0;j<3;j++)
		{
			if(ChessBoard[i][j].Chess_Index==Index&&ChessBoard[i][j].Chess_State==Empty)//序号对应并且该位置为空位
			{
				ChessBoard[i][j].Chess_State=State;
				return 1;
			}
		
		}
	}
	return 0;

}
//清除全部棋盘
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

//检查棋盘状态，更新位置
uint8 CheckChessPiece()
{
	for(uint8 i=0;i<3;i++)
	{
		for(uint8 j=0;j<3;j++)
		{
				if(ChessBoard[i][j].Chess_State==Empty)//必须棋盘位置是空的才能更新棋子
				{
					ChessBoard[i][j].Chess_State=board[i][j];//更新棋盘类别数据
				}
		}
	}

}

//缓存棋盘状态，用于更新位置
uint8 CheckSavePieceBuff()
{
	for(uint8 i=0;i<3;i++)
	{
		for(uint8 j=0;j<3;j++)
		{
				board[i][j]=ChessBoard[i][j].Chess_State;//缓存棋盘类别数据
		}
	}

}
//检测棋盘前后的状态
uint8 CheckChess()
{


}
//获取两边的棋子，黑棋，白棋的初始位置
uint8 GainSidePiece(uint8 * data,uint16 Datalen)
{
	uint16 BlackCameraBuffer[5][2];//黑色棋子像素坐标缓存
	uint16 WhiteCameraBuffer[5][2];//白色棋子像素坐标缓存
	
	uint16 BlackCount=0;//黑色棋子数量
	uint16 WhiteCount=0;//白色棋子数量
	
	uint16 DataCount=0;
	
	float BlackBuffer[5][2];//黑色棋子坐标缓存
	float WhiteBuffer[5][2];//白色棋子坐标缓存
	//获取棋子像素坐标
	union 
	{
		char byte[2];
		short value;
	}Decode;
	//rt_kprintf("A");
	while(DataCount<=Datalen)
	{
	//	rt_kprintf("%c",data[DataCount]);
		if(data[DataCount]==0x19&&data[DataCount+1]==0x10&&data[DataCount+2]==0x15)//校验帧头，如果符合帧头要求
		{
		//	rt_kprintf("B");
			DataCount=DataCount+3;//索引前进三位
			//开始读取坐标，将坐标缓存到缓存坐标中
			if(data[DataCount]==0x88)//表示该棋子来自外面
			{
			//	rt_kprintf("C");
				DataCount++;
				while(data[DataCount]!=0xFF)//直到读取到帧尾
				{
					//rt_kprintf("D");
					if(DataCount>Datalen)
						break;
					if(data[DataCount]=='B')//黑色
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
	//检查五个棋子是否都获得完毕

	
//	//将棋子坐标系转为全局坐标系
	if(BlackCount>BlackPieceCount||GradeFlag==1)//本次数量大于上次数量才更改
	{
			for(uint8 i=0;i<BlackCount;i++)//转黑棋坐标
		{
		LeftCameraTurnPosition(BlackCameraBuffer[i][0],BlackCameraBuffer[i][1],&BlackBuffer[i][0],&BlackBuffer[i][1]);
		BlacePiece[i][0]=BlackBuffer[i][0];
		BlacePiece[i][1]=BlackBuffer[i][1];
		}
		
			BlackPieceCount=BlackCount;
	}
	if(WhiteCount>WhitePieceCount||GradeFlag==1)
	{
			for(uint8 i=0;i<WhiteCount;i++)//转白棋坐标
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

//根据获得棋子的全局坐标系获取棋子所在棋盘的位置
void GainChessIndex(uint8 board[3][3],float boardpoints[9][2] ,float Point[2],uint8 State)
{
	//遍历棋盘所有位置
	float Dis=0;
	
	for(uint8 i=0;i<3;i++)
	{
		for(uint8 j=0;j<3;j++)
		{

					Dis=(Point[0]-ChessBoard[i][j].X_Position)*(Point[0]-ChessBoard[i][j].X_Position)+(Point[1]-ChessBoard[i][j].Y_Position)*(Point[1]-ChessBoard[i][j].Y_Position);
					Dis=sqrt(Dis);
					//rt_kprintf("Dis=%d\n",(int)Dis);
					//计算棋子是否在棋盘位置的中心范围内
					if(Dis<Chess_width*1.414*0.5)//该棋子在该范围内
					{
						board[i][j]=State;//将该位置置为棋子的颜色
						//保存上一次棋盘的坐标
						boardpoints[i*3+j][0]=Point[0];
						boardpoints[i*3+j][1]=Point[1];
						
					}				
		}
	}
	

}
/***获取棋盘中间的棋子***************/
uint8 GainChessPiece(uint8 * data,uint16 Datalen)
{
	
	uint16 BlackCameraBuffer[200][2];//黑色棋子像素坐标缓存
	uint16 WhiteCameraBuffer[200][2];//白色棋子像素坐标缓存
	
	uint16 BlackCount=0;//黑色棋子数量
	uint16 WhiteCount=0;//白色棋子数量
	
	uint16 DataCount=0;
	
	uint8 board_buff[3][3]={0};//棋盘状态缓存
	float board_buff_points[9][2];
	float BlackBuffer[200][2];//黑色棋子坐标缓存
	float WhiteBuffer[200][2];//白色棋子坐标缓存
	//获取棋子像素坐标
	union 
	{
		char byte[2];
		short value;
	}Decode;
	while(DataCount<=Datalen)
	{
		if(data[DataCount]==0x19&&data[DataCount+1]==0x10&&data[DataCount+2]==0x15)//校验帧头，如果符合帧头要求
		{
			DataCount=DataCount+3;//索引前进三位
			if(data[DataCount]==0x66)//表示该棋子来自外面
			{
				DataCount++;
				while(data[DataCount]!=0xFF)//直到读取到帧尾
				{
					if(DataCount>Datalen)
						break;
					if(data[DataCount]=='B')//黑色
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
	
	
//	//检查五个棋子是否都获得完毕
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
	
//	//将棋子坐标系转为全局坐标系
		for(uint8 i=0;i<BlackCount;i++)//转黑棋坐标
		{
			LeftCameraTurnPosition(BlackCameraBuffer[i][0],BlackCameraBuffer[i][1],&BlackBuffer[i][0],&BlackBuffer[i][1]);
			GainChessIndex(board_buff,board_buff_points,BlackBuffer[i],Black_Chess);
			
		}
			for(uint8 i=0;i<WhiteCount;i++)//转白棋坐标
		{
			LeftCameraTurnPosition(WhiteCameraBuffer[i][0],WhiteCameraBuffer[i][1],&WhiteBuffer[i][0],&WhiteBuffer[i][1]);
			GainChessIndex(board_buff,board_buff_points,WhiteBuffer[i],White_Chess);
		}
		
		//更新棋盘当前缓存状态
		for(uint8 i=0;i<3;i++)
		{
			for(uint8 j=0;j<3;j++)
			{
				//rt_kprintf("board_buff[%d][%d]=%d\n",i,j,(int)board_buff[i][j]);
				board_Buff[i][j]=board_buff[i][j];		
				BoardBuffPoints[i*3+j][0]=board_buff_points[i*3+j][0];//保存获取的棋盘坐标				
				BoardBuffPoints[i*3+j][1]=board_buff_points[i*3+j][1];//保存获取的棋盘坐标		
			}
		}
		
	
}

uint8 Communication_State=1;//通信状态，0空闲，1开始发送信号给openart握手，2握手成功开始获取棋盘两侧棋子坐标，3，更新棋盘中心的坐标
uint8 GradeFlag=1;//更新标志位，当题目返回到闲置状态时可以更新坐标
uint8 count=0;
void GainCartPosition()
{
		if(Communication_State==1)//发送0xBB让openart进行棋盘获取
		{
			count++;
			if(count==1)
			{
				count=0;
				uart_write_byte(UART_4,0xBB);
			}

		}
		fifo_uart4data_count = fifo_used(&uart4_data_fifo);                           // 查看 fifo 是否有数据                       // 查看 fifo 是否有数据
		fifo_read_buffer(&uart4_data_fifo, fifo4_get_data, &fifo_uart4data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲

		if(Communication_State==1)//接收art应答位进入下一个状态
		{
			
			if(fifo_uart4data_count>0)
			{
				
				for(uint8 i=0;i<fifo_uart4data_count;i++)
				{
				//	rt_kprintf("%c",fifo4_get_data[i]);
					
					if(fifo4_get_data[i]==0xee)//如果接收到应答进入获取棋子位置状态
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
			if(fifo_uart4data_count!=0)//表示接收到数据
			{
				if(TopicIndex==1||TopicIndex==2||TopicIndex==3)
				{
						GainSidePiece(fifo4_get_data,fifo_uart4data_count);//获取数据
						//GainChessPiece(fifo4_get_data,fifo_uart4data_count);
				}
				else if(TopicIndex==4)
				{
					if(Topic_4_Flag==0)//题目没开始的时候才获取外面的坐标
					{
						GainSidePiece(fifo4_get_data,fifo_uart4data_count);//获取数据
					}
					else
					{
						if((Topic_4_Chess_Order==0&&Topic_4_PeopleStartFlag==1)||(Topic_4_Chess_Order==1))//轮到人下,获取棋盘数据
						{
						GainChessPiece(fifo4_get_data,fifo_uart4data_count);
						
						}
					}

						
				}
				else if(TopicIndex==5)//第五题
				{
					if(Topic_5_Flag==0)//第五题还没开始的时候获取外部棋子坐标
					{
						GainSidePiece(fifo4_get_data,fifo_uart4data_count);//获取数据
					}
					else//如果题目5已经开始了，则只在人下完确认后获取棋盘中间数据
					{
						if(Topic_5_Chess_Order==0&&Topic_5_PeopleStartFlag==1)//轮到人在下，并且人已经下完了
						{
							GainChessPiece(fifo4_get_data,fifo_uart4data_count);
						}
						
					}
				}

			}
		
		}
		if(fifo_uart4data_count==9)//表示接收到数据
		{
				UartGetPosition(fifo4_get_data,fifo_uart4data_count,&CartKind,&Camera_x,&Camera_y);//解码获取卡片坐标以及类型		
				//计算偏移量，如果卡片在机械臂范围则不需要偏移
			
		}


}

float ADC_TurnAngle(uint16 ADC)
{
	float K=-0.09483;
	float Angle=0;
	Angle=K*(ADC-ADC_Angle_Middle);
	return Angle;

}

//根据选择矩阵值更新棋盘位置坐标
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
