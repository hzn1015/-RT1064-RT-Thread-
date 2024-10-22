#include "robotArm.h"

float RobotAngle1=0;
float RobotAngle2=0;
float RobotAngle3=0;

RobotPosition targetPosition[7];//��е�۸��ؽ�Ŀ��Ƕ�
RobotPosition LastPosition;//��е�۸��ؽڵ�ǰ�Ƕ�
uint16 RobotTime=0;//��е��ʱ���������
uint8 RobotStartFlag=0;//��е�����б�־λ,0����δ���У�1�������У�2���н���
uint8 RobotArmControlFlag=0;//��е��������־λ
uint8 RobotArmTargetPosition=0;//��е��Ŀ��������±�־λ
/************************************************/
uint16 RobotTargetPiecePosition[2];//��е��Ŀ����������
uint16 RobotTargetChess;//��е��Ŀ������λ��
/*********��е�۳�ʼ��*****************

*/

void RobotArmInit()
{
	targetPosition[0].angle1=0;
	targetPosition[0].angle2=-25;
	targetPosition[0].angle3=135;
	targetPosition[0].angle4=265;
	
	targetPosition[1].angle1=0;
	targetPosition[1].angle2=0;
	targetPosition[1].angle3=0;
	targetPosition[1].angle4=265;
	                         
	targetPosition[2].angle1=0;
	targetPosition[2].angle2=60;
	targetPosition[2].angle3=-60;
	targetPosition[2].angle4=260;
	                         
	targetPosition[3].angle1=0;
	targetPosition[3].angle2=60;
	targetPosition[3].angle3=-60;
	targetPosition[3].angle4=260;
	                         
	targetPosition[4].angle1=0;
	targetPosition[4].angle2=60;
	targetPosition[4].angle3=-90;
	targetPosition[4].angle4=260;
	 
	//����Ƭ�ĽǶ�
	targetPosition[5].angle1=-15;
	targetPosition[5].angle2=5;
	targetPosition[5].angle3=-110;
	targetPosition[5].angle4=260;
	                         
	targetPosition[6].angle1=0;
	targetPosition[6].angle2=-40;
	targetPosition[6].angle3=140;
	targetPosition[6].angle4=250;
	                  
	
	LastPosition.angle1=targetPosition[6].angle1;
	LastPosition.angle2=targetPosition[6].angle2;
	LastPosition.angle3=targetPosition[6].angle3;
	LastPosition.angle4=targetPosition[6].angle4;
	RobotContro(targetPosition[6].angle1,targetPosition[6].angle2,targetPosition[6].angle3,targetPosition[6].angle4);
	//RobotContro(targetPosition[5].angle1,targetPosition[5].angle2,targetPosition[5].angle3,targetPosition[5].angle4);
	gpio_init(D4, GPO, 0, GPO_PUSH_PULL);


}
/*********�������תȫ������ϵ****
���룺�������x��y
�����


******/
void LeftCameraTurnPosition(uint16 camera_x,uint16 camera_y,float *out_x,float *out_y)
{
	float f_x=510.64,f_y=493.282;//���غ����ױ�ֵ
	float u_0=169,v_0=125.899;
	float Z_c=380;//����߶ȣ���λmm
	float p_x=0,p_y=0,p_z=0;
	
	float P_X=0,P_Y=0;
	
//	float Delta_x=20,Delta_y=20;//����ͻ�е��ԭ��ƫ��
	//������������ϵ
	p_x=((camera_x-u_0)*Z_c/f_x);
	p_y=((camera_y-v_0)*Z_c/f_y);
	p_z=Z_c;
	//rt_kprintf("p_x=%d,p_y=%d\n",(int)p_x,(int)p_y);
	//�������ϵ�ͻ�е�ۻ�������ϵת��
	
	P_X=245.48 - 0.56065*p_y - 0.056027*p_z - 0.82615*p_x;
	P_Y=0.56194*p_x - 0.82712*p_y - 0.0093442*p_z + 283.09;
	
	//rt_kprintf("P_X=%d,P_Y=%d\n",(int)P_X,(int)P_Y);
	*out_x=P_X/10;
	*out_y=P_Y/10;
	

}

/*********�������תС������****
���룺�������x��y
�����


******/
void RightCameraTurnPosition(uint16 camera_x,uint16 camera_y,int *out_x,int *out_y)
{

	

}


/*********���������˶�ѧ��Ƕ�**********
��������õ����ؽڽǶ�



*/

/*********��е�ۼ�Ƭ****************


*************************************/
uint16 robot_time_count=0;//ʱ�����������Բ�ֵ
uint8 robot_time_delay=0;
uint8 robot_time_delay_flag=0;
uint8 robot_step=50;
void RobotPickUpCard()
{
	RobotPosition NowPosition;//��е�۸��ؽڵ�ǰ�Ƕ�
	if(RobotArmControlFlag==1)//������е��
	{
		if(robot_time_delay_flag==0)//������ʱ��е������������openart���겻��
			robot_time_delay++;
		if(robot_time_delay==10&&robot_time_delay_flag==0)
			{
					robot_time_delay=0;
					robot_time_delay_flag=1;//��ʱ��������е��
			}
		if(robot_time_delay_flag==1)//��ʱ������ʼץȡ
			{
				if(RobotTime==1&&RobotStartFlag==0)//���»�е��Ŀ��Ƕ�
				{	
					RobotStartFlag=1;//��ʾ��е�ۿ�ʼ����
					robot_step=60;
					//ElectromagnetEnable();//�򿪵����
				}
					
			//����������ϽǶ�
			NowPosition.angle1=(targetPosition[RobotTime].angle1-LastPosition.angle1)*robot_time_count/robot_step+LastPosition.angle1;
			NowPosition.angle2=(targetPosition[RobotTime].angle2-LastPosition.angle2)*robot_time_count/robot_step+LastPosition.angle2;
			NowPosition.angle3=(targetPosition[RobotTime].angle3-LastPosition.angle3)*robot_time_count/robot_step+LastPosition.angle3;
			NowPosition.angle4=(targetPosition[RobotTime].angle4-LastPosition.angle4)*robot_time_count/robot_step+LastPosition.angle4;
			if(robot_time_delay==0)
			robot_time_count++;
			RobotContro(NowPosition.angle1,NowPosition.angle2,NowPosition.angle3,NowPosition.angle4);
			if(robot_time_count>=robot_step+1)//���Բ�ֵÿ��λ��֮�����40����
			{

				LastPosition.angle1=targetPosition[RobotTime].angle1;
				LastPosition.angle2=targetPosition[RobotTime].angle2;
				LastPosition.angle3=targetPosition[RobotTime].angle3;
				LastPosition.angle4=targetPosition[RobotTime].angle4;
				RobotTime++;
				robot_time_count=0;
				
			}
			if(RobotTime>=7)//��ʾ��е��һ���жϽ�������е���м����8������
			{
				
				RobotTime=0;
				robot_time_delay_flag=0;
				RobotStartFlag=0;
				RobotArmControlFlag=0;//�˳���е���˶�״̬
				if(TopicIndex==1)//��Ŀ1
				{
					ClearTopic1Flag();
				}
				else if(TopicIndex==2||TopicIndex==3)
				{
					if(Topic_2_Count!=4)
					Topic_2_StartFlag=0;//���̽���
					else if(Topic_2_Count=4)
					{
						ClearTopic2Flag();
						ClearAllChess();
						Communication_State=2;//�л���æµ״̬
					}
					
				}
				else if(TopicIndex==4)
				{
					
					Topic_4_Chess_Order=0;//�л������ߵĲ���
					Topic_4_StartFlag=0;//��ʾ��Ŀ4ֹͣ����
					
					Communication_State=2;//�л���æµ״̬
					
				}
				else if(TopicIndex==5)
				{
					Topic_5_Chess_Order=0;//�л������ߵĲ���
					Topic_5_StartFlag=0;//��ʾ��Ŀ4ֹͣ����
					Communication_State=2;//�л���æµ״̬
					
				}
				
			
			}
			if(RobotTime==2&&robot_time_count==0)
			{
					robot_step=50;
					robot_time_delay++;
					if(robot_time_delay==50)
					{
						robot_time_delay=0;
						ElectromagnetEnable();//�򿪵����
					}
			}
			if(RobotTime==5&&robot_time_count==0)//���¿�Ƭ
			{
					gpio_set_level(D4,0);
					robot_time_delay++;
					if(robot_time_delay==50)
					{
						robot_time_delay=0;
					}
			}
			
			
		}
		
	}

}


void InverseKinematics(float x ,float y ,float z)
{
		float K1=0,K2=0,K3=0,K4=0;
		RobotAngle1=atan2(y,x)*180/3.1415926;
		
		K1=(x*x+y*y+(z-H)*(z-H)-(L1*L1+L2*L2));
		K2=(2*L1*L2);
		float k=0;//��һ��
		k=sqrt(K1*K1+K2*K2);
		K1=K1/k;
		K2=K2/k;
		float value=0;
		value=K1/K2;
		if(value>=1)
			value=1;
		else if(value<=-1)
			value=-1;
		RobotAngle3=acos(value);
		K3=L1+L2*cos(RobotAngle3);
		K4=L2*sin(RobotAngle3);
		k=sqrt(K3*K3+K4*K4);
		K3=K3/k;
		K4=K4/k;
		float betha=(z-H)/k;
		if(betha>=1)
		betha=1;
		else if(betha<=-1)
			betha=-1;
		
		RobotAngle2=atan2(K3,K4)-asin(betha);
		
		RobotAngle2=RobotAngle2*180/3.1415926;
		RobotAngle3=RobotAngle3*180/3.1415926;
		//RobotAngle2=(atan2(K1,K2)+RobotAngle3)/2;
}

/*************��е�ۿ���***************
��������ؽڽǶ�
�ؽ�				��Χ
1					-90--90
2					-90--90
3					-90--90
4					-90--90


*/
void RobotContro(float angle1,float angle2,float angle3,float angle4)
{
	angle1=-angle1+angle1_bias;
	angle2=-angle2+angle2_bias;
	angle3=-angle3+angle3_bias;
	
	//�Ƕ�����
	if(angle1>=90+angle1_bias)
		angle1=90+angle1_bias;
	else if(angle1<-90+angle1_bias)
		angle1=-90+angle1_bias;
//	
//	if(angle2>=90+angle2_bias)
//		angle2=angle2_bias+90;
//	else if(angle2<=-90+angle2_bias)
//		angle2=-90+angle2_bias;
	
	if(angle3>=135+angle3_bias)
		angle3=angle3_bias+135;
	else if(angle3<=-135+angle3_bias)
		angle3=-135+angle3_bias;
	//���ö�����
	Set_Angle(1,AngleTurnPwm1(angle1));
	Set_Angle(2,AngleTurnPwm2(angle2));
	Set_Angle(3,AngleTurnPwm2(angle3));
	Set_Angle(4,AngleTurnPwm2(angle4));
}

//��ȫ������ϵתΪ��е������ϵ
void BaseCoordinates_Turn_RobotCoordinates(float *x,float *y,uint16 *z)
{
	float x0=*x,y0=*y,z0=*z;
	x0=x0+Base_X_Bias;
	y0=y0+Base_Y_Bias;
	*x=(x0)*(cos(Angle_Base_Bias))+y0*sin(Angle_Base_Bias)+Robot_End_Bias/1.414;
	*y=-x0*sin(Angle_Base_Bias)+y0*cos(Angle_Base_Bias)-Robot_End_Bias/1.414;


}

//�����ͨ��
void ElectromagnetEnable()
{
	
	gpio_set_level(D4,1);

} 
//�ص������
void ElectromagnetDisable()
{
	gpio_set_level(D4,0);
}

//������е��
//�������������꣬����λ��
uint8 StartRobot(float Points[2],uint16 ChessPosition)
{
	float x=0,y=0;
	x=Points[0];
	y=Points[1];
	BaseCoordinates_Turn_RobotCoordinates(&x,&y,0);
	
	InverseKinematics(x,y,Chess_H+10);//����е�����õ������Ϸ�10���״�
	
	targetPosition[0].angle1=RobotAngle1;
	targetPosition[0].angle2=RobotAngle2;
	targetPosition[0].angle3=RobotAngle3;
	
	InverseKinematics(x,y,Chess_H+3.5);//����е�����õ������Ϸ�10���״�
	targetPosition[1].angle1=RobotAngle1;
	targetPosition[1].angle2=RobotAngle2;
	targetPosition[1].angle3=RobotAngle3;
	
	InverseKinematics(x,y,Chess_H+10);//����е�����õ������Ϸ�10���״�
	
	targetPosition[2].angle1=RobotAngle1;
	targetPosition[2].angle2=RobotAngle2;
	targetPosition[2].angle3=RobotAngle3;
	
	/*******************************Ѱ����������*****************/
	for(uint8 i=0;i<3;i++)
		for(uint8 j=0;j<3;j++)
		{
			if(ChessBoard[i][j].Chess_Index==ChessPosition)//�ҵ���Ӧ�������
			{
				float Points[2];
				Points[0]=ChessBoard[i][j].X_Position;
				Points[1]=ChessBoard[i][j].Y_Position;
				BaseCoordinates_Turn_RobotCoordinates(&Points[0],&Points[1],0);//ȫ������ϵ�ͻ�е������ϵת��
				
				InverseKinematics(Points[0],Points[1],Chess_H+10);//��е���䵽�����Ϸ�
				targetPosition[3].angle1=RobotAngle1;
				targetPosition[3].angle2=RobotAngle2;
				targetPosition[3].angle3=RobotAngle3;
				
				InverseKinematics(Points[0],Points[1],Chess_H+4.5);//��е���䵽�����Ϸ�
				targetPosition[4].angle1=RobotAngle1;
				targetPosition[4].angle2=RobotAngle2;
				targetPosition[4].angle3=RobotAngle3;
				
				InverseKinematics(Points[0],Points[1],Chess_H+10);//��е���䵽�����Ϸ�
				targetPosition[5].angle1=RobotAngle1;
				targetPosition[5].angle2=RobotAngle2;
				targetPosition[5].angle3=RobotAngle3;
				
				return 1;//��ʾ�ҵ������̲������ú���
			}
		
		
		}
		
	return 0;//��ʾ�Ҳ�������
	
	

}
