#include "control.h"
#include "motor.h"
#include "math.h"

uint8 TopicIndex=0;//题目索引
uint8 TopicIndexCache=1;//题目索引光标，用于显示在界面上

/***题目1控制****/

/***题目1变量****/
uint8 Topic_1_BlackIndex=1;//题目1黑棋索引
uint8 Topic_1_Flag=0;//题目1标志位
uint8 Topic_1_StartFlag=0;//题目1运行
void Topic_1_Control()
{
	if(Topic_1_Flag==1&&Topic_1_StartFlag==0)//表示开始运行题目1
	{
		Topic_1_StartFlag=1;//表示题目1已经运行
		StartRobot(BlacePiece[Topic_1_BlackIndex-1],5);//设置机械臂参数
		PlayChess(Black_Chess,5);
		RobotArmControlFlag=1;//开启机械臂
	
	}
}

//清除题目1所有标志位
void ClearTopic1Flag()
{
	Topic_1_BlackIndex=1;
	Topic_1_Flag=0;
	Topic_1_StartFlag=0;//题目1运行b
	
}

/*******题目2参数*********/
uint8 Topic_2_PieceTargetSelectIndex[4];//棋子目标索引，前两个黑棋，后两个白棋
uint8 Topic_2_ChessTargetSelectIndex[4];//棋盘目标索引，前两个黑棋，后两个白棋
uint8 Topic_2_PieceSelectIndex=1;//棋子选择索引，黑棋1，2，白棋3，4
uint8 Topic_2_ChessSelectIndex=1;//棋盘选择索引，黑棋1，2，白棋3，4
uint8 Topic_2_Flag=0;//题目2标志位,0选择棋子，1选择棋盘，2准备运行，3开始运行
uint8 Topic_2_StartFlag=0;//题目2运行标志位
uint8 Topic_2_Count=0;//题目2棋子数量
/*****************************/

/***题目2数据更新，将棋子和对应棋盘坐标对应上***/

void Topic_2_DataUpdate()
{
	

}
void Topic_2_Control()
{
	if(Topic_2_Flag==3)//表示题目开始运行
	{
		if(Topic_2_StartFlag==0)
		{
			Topic_2_StartFlag=1;//题目开始运行
			if(Topic_2_Count==0)//放黑棋1
			{
				StartRobot(BlacePiece[clip(Topic_2_PieceTargetSelectIndex[0]-1,0,4)],clip(Topic_2_ChessTargetSelectIndex[0],1,9));//设置机械臂参数
				PlayChess(Black_Chess,clip(Topic_2_ChessTargetSelectIndex[0],1,9));
				RobotArmControlFlag=1;//开启机械臂
				
			}
			else if(Topic_2_Count==1)//放黑棋2
			{
				StartRobot(BlacePiece[clip(Topic_2_PieceTargetSelectIndex[1]-1,0,4)],clip(Topic_2_ChessTargetSelectIndex[1],1,9));//设置机械臂参数
				PlayChess(Black_Chess,clip(Topic_2_ChessTargetSelectIndex[1],1,9));
				RobotArmControlFlag=1;//开启机械臂
				
			}
			else if(Topic_2_Count==2)//放白棋1
			{
				StartRobot(WhitePiece[clip(Topic_2_PieceTargetSelectIndex[2]-1,0,4)],clip(Topic_2_ChessTargetSelectIndex[2],1,9));//设置机械臂参数
				PlayChess(White_Chess,clip(Topic_2_ChessTargetSelectIndex[2],1,9));
				RobotArmControlFlag=1;//开启机械臂
				
			}
			else if(Topic_2_Count==3)//放白棋2
			{
				StartRobot(WhitePiece[clip(Topic_2_PieceTargetSelectIndex[3]-1,0,4)],clip(Topic_2_ChessTargetSelectIndex[3],1,9));//设置机械臂参数
				PlayChess(White_Chess,clip(Topic_2_ChessTargetSelectIndex[3],1,9));
				RobotArmControlFlag=1;//开启机械臂
				
			}
			Topic_2_Count++;
			
		
		}
	}



}
//清除题目2所有标志位
void ClearTopic2Flag()
{
		Topic_2_StartFlag=0;//过程结束
		Topic_2_Count=0;
		Topic_2_Flag=2;//切换回准备运行

		
	
}
/****题目选择*****/

/****题目4变量和函数***********/

uint8 Topic_4_Chess_Order=1;//棋盘顺序，一开始默认机器先下，1为机器下，0为人下
uint8 Topic_4_RobotIndex=1;//机器下棋索引，只有一开始下第一个棋子需要
uint8 Topic_4_RobotStartFlag=0;//机器人开始标志位
uint8 Topic_4_PeopleStartFlag=0;//人开始下棋标志位，由按键设置
uint8 Topic_4_PeopleIndex=1;//人下棋索引
uint8 Topic_4_BlackIndex=0;//黑棋索引
uint8 Topic_4_WhiteIndex=0;//白棋索引
uint8 Topic_4_Flag=0;//题目2标志位,0选择棋子，1选择棋盘，2准备运行，3开始运行
uint8 Topic_4_StartFlag=0;

void Topic_4_Control()
{
	if(Topic_4_Flag==1)//启动题目
	{
		//检测是否有人胜利
		if(checkWin()==Max_Robot)
		{
			Topic_4_Flag=2;//表示机器人胜利
		}
		else if(checkWin()==Min_People)
		{
			Topic_4_Flag=3;//表示人胜利
		}
		else if(isDraw()!=0)
		{
			Topic_4_Flag=4;//表示平局
		}
		
		
		if(Topic_4_StartFlag==1&&RobotArmControlFlag==0)//表示开启题目4运行
		{
			
			if(Topic_4_Chess_Order==1)//机器人控制阶段
			{
				
				if(Topic_4_RobotStartFlag==0)//表示是机器人第一步，由人设置的坐标
				{
					Topic_4_RobotStartFlag=1;//表示第一步已经走了
					Communication_State=0;
					StartRobot(BlacePiece[clip(Topic_4_BlackIndex,0,4)],Topic_4_RobotIndex);//设置机械臂参数
					PlayChess(Max_Robot,Topic_4_RobotIndex);
					CheckSavePieceBuff();//保存当前棋盘
					RobotArmControlFlag=1;//启动机械臂
					Topic_4_BlackIndex++;
				}
				else//表示是机器人自己走的
				{
					
					//悔棋检测，如果没有晦气则执行MiniMax算法，否则恢复棋子
					uint8 Chess_Index=0;
					Communication_State=0;//启动机械臂前设置为空闲模式
					Chess_Index=bestMove();
					StartRobot(BlacePiece[clip(Topic_4_BlackIndex,0,4)],Chess_Index);//设置机械臂参数
					PlayChess(Max_Robot,Chess_Index);
					CheckSavePieceBuff();//保存当前棋盘
	
					RobotArmControlFlag=1;
//					Topic_4_Chess_Order=0;//切换回人走的步骤
//					Topic_4_StartFlag=0;//表示题目4停止运行
					Topic_4_BlackIndex++;					
				}
			
			
			}
			else//人控制阶段
			{
				if(Topic_4_PeopleStartFlag==1)//表示已经可以开始获得人下的棋子
				{
					uint8 flag=0;
					flag=Topic_4_GainWhitePIece();//获取人下的棋子
					//rt_kprintf("flag=%d",flag);
					if(flag!=0)//表示已经找到人下的棋子位置了
					{
						PlayChess(Min_People,Topic_4_PeopleIndex);
						CheckSavePieceBuff();//保存当前棋盘
						Topic_4_Chess_Order=1;//切换回机器走的步骤
						Topic_4_WhiteIndex++;
						Topic_4_PeopleStartFlag=0;
					}
					
					
				}
				
			}
		}
	}


}
//题目4获取白棋
uint8 Topic_4_GainWhitePIece()
{
	for(uint8 i=0;i<3;i++)
	{
		for(uint8 j=0;j<3;j++)
		{
			
				if(board_Buff[i][j]==White_Chess&&board[i][j]==Empty)//如果当前为白色前面为空闲，则该位置为下的位置
				{
					ChessBoard[i][j].Chess_State=White_Chess;
					return ChessBoard[i][j].Chess_Index;
				}
		}
	}
	
	return 0;


}
//题目4作弊检测
uint8 Topic_4_CheatCheck()
{
	//将当前棋盘状态和前一个棋盘状态进行比较
	for(uint8 i=0;i<3;i++)
	{
		for(uint8 j=0;j<3;j++)
		{
			
				if(ChessBoard[i][j].Chess_State==Empty&&board[i][j]==Black_Chess)//如果当前该位置是空闲的，但
				{
					return ChessBoard[i][j].Chess_Index;
				}
		}
	}


}
//清除题目4所有数据
void Topic_4_ClearAllData()
{
	Topic_4_Chess_Order=1;//棋盘顺序，一开始默认机器先下，1为机器下，0为人下
	Topic_4_RobotIndex=1;//机器下棋索引，只有一开始下第一个棋子需要
	Topic_4_RobotStartFlag=0;//机器人开始标志位
	Topic_4_PeopleIndex=1;//人下棋索引
	Topic_4_BlackIndex=0;//黑棋索引
	Topic_4_WhiteIndex=0;//白棋索引
	Topic_4_Flag=0;//题目2标志位,0选择棋子，1选择棋盘，2准备运行，3开始运行
	Topic_4_StartFlag=0;
	Topic_4_PeopleStartFlag=0;
	ClearAllChess();

}

/****题目5变量和函数***********/

uint8 Topic_5_Chess_Order=0;//棋盘顺序，一开始默认人先下，1为机器下，0为人下
uint8 Topic_5_RobotIndex=1;//机器下棋索引，只有一开始下第一个棋子需要
uint8 Topic_5_RobotStartFlag=0;//机器人开始标志位
uint8 Topic_5_PeopleIndex=1;//人下棋索引
uint8 Topic_5_PeopleStartFlag=0;//人下棋开始标志位
uint8 Topic_5_BlackIndex=0;//黑棋索引
uint8 Topic_5_WhiteIndex=0;//白棋索引
uint8 Topic_5_Flag=0;//题目5
uint8 Topic_5_StartFlag=0;

//题目5获取黑色棋子
uint8 Topic_5_GainBlackPiece()
{
	for(uint8 i=0;i<3;i++)
	{
		for(uint8 j=0;j<3;j++)
		{
			
				if(board_Buff[i][j]==Black_Chess&&board[i][j]==Empty)//如果当前为白色前面为空闲，则该位置为下的位置
				{
					ChessBoard[i][j].Chess_State=Black_Chess;
					return ChessBoard[i][j].Chess_Index;
				}
		}
	}
	
	return 0;

}
void Topic_5_Control()
{
	if(Topic_5_Flag==1)//启动题目
	{
		//检测是否有人胜利
		if(checkWin()==Max_Robot)
		{
			Topic_5_Flag=2;//表示机器人胜利
		}
		else if(checkWin()==Min_People)
		{
			Topic_5_Flag=3;//表示人胜利
		}
		else if(isDraw()!=0)
		{
			Topic_5_Flag=4;//表示平局
		}
		
		
		if(Topic_5_StartFlag==1&&RobotArmControlFlag==0)//表示开启题目4运行
		{
			
			if(Topic_5_Chess_Order==1)//机器人控制阶段
			{
				rt_kprintf("A\n");
					uint8 Chess_Index=0;
					Chess_Index=bestMove();
					StartRobot(WhitePiece[clip(Topic_5_WhiteIndex,0,4)],Chess_Index);//设置机械臂参数
					PlayChess(Max_Robot,Chess_Index);
					RobotArmControlFlag=1;
//					Topic_4_Chess_Order=0;//切换回人走的步骤
//					Topic_4_StartFlag=0;//表示题目4停止运行
					Topic_5_WhiteIndex++;					
			
			
			}
			else//人控制阶段
			{
			//	rt_kprintf("B\n");
				if(Topic_5_PeopleStartFlag==1)//人已经下完确认了
				{
				//	rt_kprintf("C\n");
					uint8 flag=0;
					flag=Topic_5_GainBlackPiece();//找到棋子在棋盘的位置
				//	rt_kprintf("flag=%d",flag);
					if(flag!=0)//找到了黑色棋子对应的棋盘位置
					{
						PlayChess(Min_People,flag);
						Topic_5_Chess_Order=1;//切换回机器走的步骤
						Topic_5_BlackIndex++;
						Topic_5_PeopleStartFlag=0;
						CheckSavePieceBuff();//保存当前棋盘
						
					}
				
				}
					
			}
		}
	}


}

//清除题目5所有数据
void Topic_5_ClearAllData()
{
	Topic_5_Chess_Order=0;//棋盘顺序，一开始默认机器先下，1为机器下，0为人下
	Topic_5_RobotIndex=1;//机器下棋索引，只有一开始下第一个棋子需要
	Topic_5_RobotStartFlag=0;//机器人开始标志位
	Topic_5_PeopleIndex=1;//人下棋索引
	Topic_5_BlackIndex=0;//黑棋索引
	Topic_5_WhiteIndex=0;//白棋索引
	Topic_5_Flag=0;//题目2标志位,0选择棋子，1选择棋盘，2准备运行，3开始运行
	Topic_5_StartFlag=0;
	Max_Robot=Black_Chess;
	Min_People=White_Chess;
	ClearAllChess();

}
void TopicSelect()
{
	switch(TopicIndex)
	{
		case 1:Topic_1_Control();break;
		case 2:
		case 3:Topic_2_Control();break;
		case 4:Topic_4_Control();break;
		case 5:Topic_5_Control();break;
	}

}