#include "control.h"
#include "motor.h"
#include "math.h"

uint8 TopicIndex=0;//��Ŀ����
uint8 TopicIndexCache=1;//��Ŀ������꣬������ʾ�ڽ�����

/***��Ŀ1����****/

/***��Ŀ1����****/
uint8 Topic_1_BlackIndex=1;//��Ŀ1��������
uint8 Topic_1_Flag=0;//��Ŀ1��־λ
uint8 Topic_1_StartFlag=0;//��Ŀ1����
void Topic_1_Control()
{
	if(Topic_1_Flag==1&&Topic_1_StartFlag==0)//��ʾ��ʼ������Ŀ1
	{
		Topic_1_StartFlag=1;//��ʾ��Ŀ1�Ѿ�����
		StartRobot(BlacePiece[Topic_1_BlackIndex-1],5);//���û�е�۲���
		PlayChess(Black_Chess,5);
		RobotArmControlFlag=1;//������е��
	
	}
}

//�����Ŀ1���б�־λ
void ClearTopic1Flag()
{
	Topic_1_BlackIndex=1;
	Topic_1_Flag=0;
	Topic_1_StartFlag=0;//��Ŀ1����b
	
}

/*******��Ŀ2����*********/
uint8 Topic_2_PieceTargetSelectIndex[4];//����Ŀ��������ǰ�������壬����������
uint8 Topic_2_ChessTargetSelectIndex[4];//����Ŀ��������ǰ�������壬����������
uint8 Topic_2_PieceSelectIndex=1;//����ѡ������������1��2������3��4
uint8 Topic_2_ChessSelectIndex=1;//����ѡ������������1��2������3��4
uint8 Topic_2_Flag=0;//��Ŀ2��־λ,0ѡ�����ӣ�1ѡ�����̣�2׼�����У�3��ʼ����
uint8 Topic_2_StartFlag=0;//��Ŀ2���б�־λ
uint8 Topic_2_Count=0;//��Ŀ2��������
/*****************************/

/***��Ŀ2���ݸ��£������ӺͶ�Ӧ���������Ӧ��***/

void Topic_2_DataUpdate()
{
	

}
void Topic_2_Control()
{
	if(Topic_2_Flag==3)//��ʾ��Ŀ��ʼ����
	{
		if(Topic_2_StartFlag==0)
		{
			Topic_2_StartFlag=1;//��Ŀ��ʼ����
			if(Topic_2_Count==0)//�ź���1
			{
				StartRobot(BlacePiece[clip(Topic_2_PieceTargetSelectIndex[0]-1,0,4)],clip(Topic_2_ChessTargetSelectIndex[0],1,9));//���û�е�۲���
				PlayChess(Black_Chess,clip(Topic_2_ChessTargetSelectIndex[0],1,9));
				RobotArmControlFlag=1;//������е��
				
			}
			else if(Topic_2_Count==1)//�ź���2
			{
				StartRobot(BlacePiece[clip(Topic_2_PieceTargetSelectIndex[1]-1,0,4)],clip(Topic_2_ChessTargetSelectIndex[1],1,9));//���û�е�۲���
				PlayChess(Black_Chess,clip(Topic_2_ChessTargetSelectIndex[1],1,9));
				RobotArmControlFlag=1;//������е��
				
			}
			else if(Topic_2_Count==2)//�Ű���1
			{
				StartRobot(WhitePiece[clip(Topic_2_PieceTargetSelectIndex[2]-1,0,4)],clip(Topic_2_ChessTargetSelectIndex[2],1,9));//���û�е�۲���
				PlayChess(White_Chess,clip(Topic_2_ChessTargetSelectIndex[2],1,9));
				RobotArmControlFlag=1;//������е��
				
			}
			else if(Topic_2_Count==3)//�Ű���2
			{
				StartRobot(WhitePiece[clip(Topic_2_PieceTargetSelectIndex[3]-1,0,4)],clip(Topic_2_ChessTargetSelectIndex[3],1,9));//���û�е�۲���
				PlayChess(White_Chess,clip(Topic_2_ChessTargetSelectIndex[3],1,9));
				RobotArmControlFlag=1;//������е��
				
			}
			Topic_2_Count++;
			
		
		}
	}



}
//�����Ŀ2���б�־λ
void ClearTopic2Flag()
{
		Topic_2_StartFlag=0;//���̽���
		Topic_2_Count=0;
		Topic_2_Flag=2;//�л���׼������

		
	
}
/****��Ŀѡ��*****/

/****��Ŀ4�����ͺ���***********/

uint8 Topic_4_Chess_Order=1;//����˳��һ��ʼĬ�ϻ������£�1Ϊ�����£�0Ϊ����
uint8 Topic_4_RobotIndex=1;//��������������ֻ��һ��ʼ�µ�һ��������Ҫ
uint8 Topic_4_RobotStartFlag=0;//�����˿�ʼ��־λ
uint8 Topic_4_PeopleStartFlag=0;//�˿�ʼ�����־λ���ɰ�������
uint8 Topic_4_PeopleIndex=1;//����������
uint8 Topic_4_BlackIndex=0;//��������
uint8 Topic_4_WhiteIndex=0;//��������
uint8 Topic_4_Flag=0;//��Ŀ2��־λ,0ѡ�����ӣ�1ѡ�����̣�2׼�����У�3��ʼ����
uint8 Topic_4_StartFlag=0;

void Topic_4_Control()
{
	if(Topic_4_Flag==1)//������Ŀ
	{
		//����Ƿ�����ʤ��
		if(checkWin()==Max_Robot)
		{
			Topic_4_Flag=2;//��ʾ������ʤ��
		}
		else if(checkWin()==Min_People)
		{
			Topic_4_Flag=3;//��ʾ��ʤ��
		}
		else if(isDraw()!=0)
		{
			Topic_4_Flag=4;//��ʾƽ��
		}
		
		
		if(Topic_4_StartFlag==1&&RobotArmControlFlag==0)//��ʾ������Ŀ4����
		{
			
			if(Topic_4_Chess_Order==1)//�����˿��ƽ׶�
			{
				
				if(Topic_4_RobotStartFlag==0)//��ʾ�ǻ����˵�һ�����������õ�����
				{
					Topic_4_RobotStartFlag=1;//��ʾ��һ���Ѿ�����
					Communication_State=0;
					StartRobot(BlacePiece[clip(Topic_4_BlackIndex,0,4)],Topic_4_RobotIndex);//���û�е�۲���
					PlayChess(Max_Robot,Topic_4_RobotIndex);
					CheckSavePieceBuff();//���浱ǰ����
					RobotArmControlFlag=1;//������е��
					Topic_4_BlackIndex++;
				}
				else//��ʾ�ǻ������Լ��ߵ�
				{
					
					//�����⣬���û�л�����ִ��MiniMax�㷨������ָ�����
					uint8 Chess_Index=0;
					Communication_State=0;//������е��ǰ����Ϊ����ģʽ
					Chess_Index=bestMove();
					StartRobot(BlacePiece[clip(Topic_4_BlackIndex,0,4)],Chess_Index);//���û�е�۲���
					PlayChess(Max_Robot,Chess_Index);
					CheckSavePieceBuff();//���浱ǰ����
	
					RobotArmControlFlag=1;
//					Topic_4_Chess_Order=0;//�л������ߵĲ���
//					Topic_4_StartFlag=0;//��ʾ��Ŀ4ֹͣ����
					Topic_4_BlackIndex++;					
				}
			
			
			}
			else//�˿��ƽ׶�
			{
				if(Topic_4_PeopleStartFlag==1)//��ʾ�Ѿ����Կ�ʼ������µ�����
				{
					uint8 flag=0;
					flag=Topic_4_GainWhitePIece();//��ȡ���µ�����
					//rt_kprintf("flag=%d",flag);
					if(flag!=0)//��ʾ�Ѿ��ҵ����µ�����λ����
					{
						PlayChess(Min_People,Topic_4_PeopleIndex);
						CheckSavePieceBuff();//���浱ǰ����
						Topic_4_Chess_Order=1;//�л��ػ����ߵĲ���
						Topic_4_WhiteIndex++;
						Topic_4_PeopleStartFlag=0;
					}
					
					
				}
				
			}
		}
	}


}
//��Ŀ4��ȡ����
uint8 Topic_4_GainWhitePIece()
{
	for(uint8 i=0;i<3;i++)
	{
		for(uint8 j=0;j<3;j++)
		{
			
				if(board_Buff[i][j]==White_Chess&&board[i][j]==Empty)//�����ǰΪ��ɫǰ��Ϊ���У����λ��Ϊ�µ�λ��
				{
					ChessBoard[i][j].Chess_State=White_Chess;
					return ChessBoard[i][j].Chess_Index;
				}
		}
	}
	
	return 0;


}
//��Ŀ4���׼��
uint8 Topic_4_CheatCheck()
{
	//����ǰ����״̬��ǰһ������״̬���бȽ�
	for(uint8 i=0;i<3;i++)
	{
		for(uint8 j=0;j<3;j++)
		{
			
				if(ChessBoard[i][j].Chess_State==Empty&&board[i][j]==Black_Chess)//�����ǰ��λ���ǿ��еģ���
				{
					return ChessBoard[i][j].Chess_Index;
				}
		}
	}


}
//�����Ŀ4��������
void Topic_4_ClearAllData()
{
	Topic_4_Chess_Order=1;//����˳��һ��ʼĬ�ϻ������£�1Ϊ�����£�0Ϊ����
	Topic_4_RobotIndex=1;//��������������ֻ��һ��ʼ�µ�һ��������Ҫ
	Topic_4_RobotStartFlag=0;//�����˿�ʼ��־λ
	Topic_4_PeopleIndex=1;//����������
	Topic_4_BlackIndex=0;//��������
	Topic_4_WhiteIndex=0;//��������
	Topic_4_Flag=0;//��Ŀ2��־λ,0ѡ�����ӣ�1ѡ�����̣�2׼�����У�3��ʼ����
	Topic_4_StartFlag=0;
	Topic_4_PeopleStartFlag=0;
	ClearAllChess();

}

/****��Ŀ5�����ͺ���***********/

uint8 Topic_5_Chess_Order=0;//����˳��һ��ʼĬ�������£�1Ϊ�����£�0Ϊ����
uint8 Topic_5_RobotIndex=1;//��������������ֻ��һ��ʼ�µ�һ��������Ҫ
uint8 Topic_5_RobotStartFlag=0;//�����˿�ʼ��־λ
uint8 Topic_5_PeopleIndex=1;//����������
uint8 Topic_5_PeopleStartFlag=0;//�����忪ʼ��־λ
uint8 Topic_5_BlackIndex=0;//��������
uint8 Topic_5_WhiteIndex=0;//��������
uint8 Topic_5_Flag=0;//��Ŀ5
uint8 Topic_5_StartFlag=0;

//��Ŀ5��ȡ��ɫ����
uint8 Topic_5_GainBlackPiece()
{
	for(uint8 i=0;i<3;i++)
	{
		for(uint8 j=0;j<3;j++)
		{
			
				if(board_Buff[i][j]==Black_Chess&&board[i][j]==Empty)//�����ǰΪ��ɫǰ��Ϊ���У����λ��Ϊ�µ�λ��
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
	if(Topic_5_Flag==1)//������Ŀ
	{
		//����Ƿ�����ʤ��
		if(checkWin()==Max_Robot)
		{
			Topic_5_Flag=2;//��ʾ������ʤ��
		}
		else if(checkWin()==Min_People)
		{
			Topic_5_Flag=3;//��ʾ��ʤ��
		}
		else if(isDraw()!=0)
		{
			Topic_5_Flag=4;//��ʾƽ��
		}
		
		
		if(Topic_5_StartFlag==1&&RobotArmControlFlag==0)//��ʾ������Ŀ4����
		{
			
			if(Topic_5_Chess_Order==1)//�����˿��ƽ׶�
			{
				rt_kprintf("A\n");
					uint8 Chess_Index=0;
					Chess_Index=bestMove();
					StartRobot(WhitePiece[clip(Topic_5_WhiteIndex,0,4)],Chess_Index);//���û�е�۲���
					PlayChess(Max_Robot,Chess_Index);
					RobotArmControlFlag=1;
//					Topic_4_Chess_Order=0;//�л������ߵĲ���
//					Topic_4_StartFlag=0;//��ʾ��Ŀ4ֹͣ����
					Topic_5_WhiteIndex++;					
			
			
			}
			else//�˿��ƽ׶�
			{
			//	rt_kprintf("B\n");
				if(Topic_5_PeopleStartFlag==1)//���Ѿ�����ȷ����
				{
				//	rt_kprintf("C\n");
					uint8 flag=0;
					flag=Topic_5_GainBlackPiece();//�ҵ����������̵�λ��
				//	rt_kprintf("flag=%d",flag);
					if(flag!=0)//�ҵ��˺�ɫ���Ӷ�Ӧ������λ��
					{
						PlayChess(Min_People,flag);
						Topic_5_Chess_Order=1;//�л��ػ����ߵĲ���
						Topic_5_BlackIndex++;
						Topic_5_PeopleStartFlag=0;
						CheckSavePieceBuff();//���浱ǰ����
						
					}
				
				}
					
			}
		}
	}


}

//�����Ŀ5��������
void Topic_5_ClearAllData()
{
	Topic_5_Chess_Order=0;//����˳��һ��ʼĬ�ϻ������£�1Ϊ�����£�0Ϊ����
	Topic_5_RobotIndex=1;//��������������ֻ��һ��ʼ�µ�һ��������Ҫ
	Topic_5_RobotStartFlag=0;//�����˿�ʼ��־λ
	Topic_5_PeopleIndex=1;//����������
	Topic_5_BlackIndex=0;//��������
	Topic_5_WhiteIndex=0;//��������
	Topic_5_Flag=0;//��Ŀ2��־λ,0ѡ�����ӣ�1ѡ�����̣�2׼�����У�3��ʼ����
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