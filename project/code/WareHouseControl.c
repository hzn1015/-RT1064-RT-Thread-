#include "WareHouseControl.h"

/*******���ձ���************************/

WareHouseStructure WareHouse[8];//8������
WareHouseLabelPair Label[18];//15���࣬15����ǩ
uint8 WareHouseIndex=0;//��ǰ��������,
uint8 WareHouseState=3;//����״̬��0װ����1ж����2,����ĸ�����û�й��ϣ��ػ���,3����״̬
float WareHouseCurrentAngle =0;//��ǰ���ն�����ڵĽǶ�
/*************���տ���****************



***/
//���սṹ���ʼ��
void WareHouseStructureInit()
{
	//����װ���Ƕ�
	int LeftAngleBias=-90;
	int RightAngleBias=180;//-60;
	int LoadAngleBias=-150;
	
	WareHouse[0].WareHouseLoadAngle=(320  +LoadAngleBias) *0.5;
	WareHouse[1].WareHouseLoadAngle=(370  +LoadAngleBias) *0.5;
	WareHouse[2].WareHouseLoadAngle=(410  +LoadAngleBias) *0.5;
	WareHouse[3].WareHouseLoadAngle=(155 +LoadAngleBias) *0.5;
	WareHouse[4].WareHouseLoadAngle=(190 +LoadAngleBias) *0.5;
	WareHouse[5].WareHouseLoadAngle=(235 +LoadAngleBias) *0.5;
	WareHouse[6].WareHouseLoadAngle=(280 +LoadAngleBias) *0.5;
	WareHouse[7].WareHouseLoadAngle=(325 +LoadAngleBias) *0.5;
	
	//�������ж���Ƕ�
	WareHouse[0].WareHouseLeftUnloadAngle=(LeftAngleBias+225)*0.5; 
	WareHouse[1].WareHouseLeftUnloadAngle=(LeftAngleBias+270)*0.5; 
	WareHouse[2].WareHouseLeftUnloadAngle=(LeftAngleBias+315)*0.5;  
	WareHouse[3].WareHouseLeftUnloadAngle=(LeftAngleBias+0+360 )*0.5;
	WareHouse[4].WareHouseLeftUnloadAngle=(LeftAngleBias+45 +360 )*0.5;
	WareHouse[5].WareHouseLeftUnloadAngle=(LeftAngleBias+90 +360)*0.5;
	WareHouse[6].WareHouseLeftUnloadAngle=(LeftAngleBias+135+360)*0.5;
	WareHouse[7].WareHouseLeftUnloadAngle=(LeftAngleBias+180+360 )*0.5;
	//�����Ҳ�ж���Ƕ�
	WareHouse[0].WareHouseRightUnloadAngle=(90  +RightAngleBias   )           *0.5; 
	WareHouse[1].WareHouseRightUnloadAngle=(135	+RightAngleBias		)					*0.5;
	WareHouse[2].WareHouseRightUnloadAngle=(180	+RightAngleBias	-360	)					*0.5;
	WareHouse[3].WareHouseRightUnloadAngle=(225	+RightAngleBias	-360	)					*0.5;
	WareHouse[4].WareHouseRightUnloadAngle=(270	+RightAngleBias	-360	)					*0.5;
	WareHouse[5].WareHouseRightUnloadAngle=(315	+RightAngleBias	-360	)					*0.5;
	WareHouse[6].WareHouseRightUnloadAngle=(360	+RightAngleBias	-360	)					*0.5;
	WareHouse[7].WareHouseRightUnloadAngle=(405	+RightAngleBias	-360	)					*0.5;
	
	WareHouse[0].WareHouseState=0;
	WareHouse[1].WareHouseState=0;
	WareHouse[2].WareHouseState=0;
	WareHouse[3].WareHouseState=0;
	WareHouse[4].WareHouseState=0;
	WareHouse[5].WareHouseState=0;
	WareHouse[6].WareHouseState=0;
	WareHouse[7].WareHouseState=0;
	
	WareHouse[0].WareHouseLabel=0;
	WareHouse[1].WareHouseLabel=0;
	WareHouse[2].WareHouseLabel=0;
	WareHouse[3].WareHouseLabel=0;
	WareHouse[4].WareHouseLabel=0;
	WareHouse[5].WareHouseLabel=0;
	WareHouse[6].WareHouseLabel=0;
	WareHouse[7].WareHouseLabel=0;
	
	//***********��ǩ��ʼ��****************************/
	Label[0].WareHouseLabel=1;
	Label[0].state=0;
	Label[1].WareHouseLabel=2;
	Label[1].state=0;
	Label[2].WareHouseLabel=3;
	Label[2].state=0;
	
	uint8 j=0;
	for(uint8 i=3;i<18;i++)
	{
		Label[i].WareHouseLabel=18+j;
		Label[i].CartLabel=3+j;
		Label[i].state=0;
		j++;
	}
	
	WareHouseCurrentAngle=WareHouse[0].WareHouseLoadAngle;
	Set_Angle(5,AngleTurnPwm2(WareHouseCurrentAngle));
	
	


}

//���ճ�ʼ��
void WareHouseInit(void)
{
	gpio_init(WareHousePin1, GPO, 0, GPO_PUSH_PULL);
	gpio_init(WareHousePin2, GPO, 0, GPO_PUSH_PULL);
	gpio_init(WareHousePin3, GPO, 0, GPO_PUSH_PULL);
	gpio_init(WareHouseEnablePin, GPO, 0, GPO_PUSH_PULL);

	WareHouseStructureInit();
	
	
	
}
//���ƻ��ն����ת
//���������Ŀ��Ƕȣ��ٶȣ������ǰ�Ƕ�
uint8 WareHouseAngleSet(float angle, float speed)//ר�����ڿ��ƻ��յĶ��,�ٶȽ���ȡֵ1.5~5
{
	float Error = angle - WareHouseCurrentAngle;
	float step = (angle - WareHouseCurrentAngle)/speed;
	int time = abs(Error);

	if(angle != WareHouseCurrentAngle)
	{
		Error = angle - WareHouseCurrentAngle;
		//system_delay_ms((int)(time/speed));
		if(abs(Error) > abs(step))
		{
			WareHouseCurrentAngle +=  step;
			Set_Angle(5,AngleTurnPwm2(WareHouseCurrentAngle));
			return 0;
		}
		else{
			WareHouseCurrentAngle = angle;
			Set_Angle(5,AngleTurnPwm2(WareHouseCurrentAngle));
			return 1;
		}
	}
	else
	return 0;
}
void OpenWarehouse(unsigned char Index)
{
	int Pin1=0,Pin2=0,Pin3=0;
	switch(Index)
	{
		case 1: 
		{

			gpio_set_level(WareHousePin1,0);
			gpio_set_level(WareHousePin2,1);
			gpio_set_level(WareHousePin3,1);
		};break;
		case 2: 
		{
			gpio_set_level(WareHousePin1,1);
			gpio_set_level(WareHousePin2,0);
			gpio_set_level(WareHousePin3,0);
			
		};break;
		case 3: 
		{
			gpio_set_level(WareHousePin1,1);
			gpio_set_level(WareHousePin2,0);
			gpio_set_level(WareHousePin3,1);
			

		};break;
		case 4: 
		{
			gpio_set_level(WareHousePin1,1);
			gpio_set_level(WareHousePin2,1);
			gpio_set_level(WareHousePin3,0);
			
			

		};break;
		case 5: 
		{
			gpio_set_level(WareHousePin1,1);
			gpio_set_level(WareHousePin2,1);
			gpio_set_level(WareHousePin3,1);

		};break;
		case 6: 
		{
			gpio_set_level(WareHousePin1,0);
			gpio_set_level(WareHousePin2,0);
			gpio_set_level(WareHousePin3,0);
			

			
		
		};break;
		case 7: 
		{
			gpio_set_level(WareHousePin1,0);
			gpio_set_level(WareHousePin2,0);
			gpio_set_level(WareHousePin3,1);
			
		};break;
		case 8: 
		{
			gpio_set_level(WareHousePin1,0);
			gpio_set_level(WareHousePin2,1);
			gpio_set_level(WareHousePin3,0);
			

			

		};break;
		
	}
	gpio_set_level(WareHouseEnablePin,1);
	
		

}
//������л��ձ�ǩ״̬
void LabelClose()
{

	for(uint8 i=0;i<15;i++)
	{
		Label[i].state=0;
	}
}
uint8 WareHouseChoice(uint8 Kind)
{
	
	//�������л��������ж��Ƿ�����ͬ�ı�ǩ
	for(uint8 i=0;i<8;i++)
	{
		if(WareHouse[i].WareHouseLabel==Kind)//�������ͬ�ı�����������λ����Ӧ����
		{
			WareHouseIndex=i;//���»�������
			return i;//���ػ�������
		}
	}
	
	WareHouseIndex++;
	WareHouse[WareHouseIndex].WareHouseLabel=Kind;
	WareHouse[WareHouseIndex].WareHouseState=1;
	return WareHouseIndex;
}

//���յ�ǰ�Ƕȸ���
void WareHouseNowAngleUpdate()
{
	
}

//����ƥ�䣬���ڶ���Ƭ��Ӧƥ�����
//����Ϊ��ǩ�����ݱ�ǩƥ���Ӧ����
uint8 WareHouseMatching(uint8 Kind)
{
	for(uint8 i=0;i<15;i++)//�������л��ձ�ǩ
	{
		if(Label[i].WareHouseLabel==Kind)//����ҵ��˶�Ӧ��ǩ��˵���ҵ��˸û��ն�Ӧ�Ŀ�Ƭ��ǩ���ٱ���һ�λ��ձ�ǩ�ҵ��������
		{
			if(Label[i].state==0)//����û��ջ�û�б���
			{
				Label[i].state=1;
				for(uint8 j=0;j<8;j++)
				{
					if(Label[i].CartLabel==WareHouse[j].WareHouseLabel)//�ҵ��û��ձ�ǩ
					{
						return j;//���ػ������к�
					}
				}
					
			}
			else
				{
				return 0;
				}
			}
			
		}
	return 0;
}
void WareHouseControl()
{
	
	while(1)
	{
		//װ��
		if(WareHouseState==0)
		{
			uint8 flag=0;
			flag=WareHouseAngleSet(WareHouse[WareHouseIndex].WareHouseLoadAngle,40);
		}
		else if(WareHouseState==1)//ж��
		{
			
			uint8 flag=0;
			if(CartDirection==0)
			{	
				flag=WareHouseAngleSet(WareHouse[WareHouseIndex].WareHouseLeftUnloadAngle,40);
			}
			else
			{
				flag=WareHouseAngleSet(WareHouse[WareHouseIndex].WareHouseRightUnloadAngle,40);
			}
			if(flag==1)
				{
					//OpenWarehouse(WareHouseIndex+1);
					rt_thread_delay(2000);
					WareHouse[WareHouseIndex].WareHouseLabel=0;//�����ǩ
					WareHouse[WareHouseIndex].WareHouseState=0;//�������״̬
					gpio_set_level(WareHouseEnablePin,0);
				  WareHouseState=2;//����ػ���״̬
				}
		}
		else if(WareHouseState==2)//�ز���
		{ 
			
			uint8 flag=0;
			if(CartDirection==0)
			{		
				flag=WareHouseAngleSet(WareHouse[WareHouseIndex].WareHouseLeftUnloadAngle+45,20);
			}
			else
			{
				flag=WareHouseAngleSet(WareHouse[WareHouseIndex].WareHouseRightUnloadAngle+45,30);
			}
			if(flag==1)
				{
					 WareHouseState=3;//���ش���״̬
					if(MotionStateFlag==1)//ʮ�ֶ���Ƭ
					{

						if(LossCartStartFlag==0)
						{
								CartStateFlag=0;//�޿�Ƭ
								CrossLoseCardSatute=0;
							  P_encoder[0]=0;
							  P_encoder[1]=0;
							  P_encoder[2]=0;
							  P_encoder[3]=0;
						}
//						else
//						{
//							LossCartStartFlag=0;//�������ζ���Ƭ��־
//							CrossLoseCardSatute=0;
//							WareHouseDistance=WareHouseDistance_2-(X_offset_Target-WareHouseDistance_Bias);
//							WareHouseDistance_Bias=0;
//							X_offset_Target=0;
//							
//							//���������ֵ
//							P_encoder[0]=0;
//							P_encoder[1]=0;
//							P_encoder[2]=0;
//							P_encoder[3]=0;
//		
//						}
					
					}
					else if(MotionStateFlag==2)//Բ������Ƭ
					{
						
						if(LossCartStartFlag==0)
						{
								CartStateFlag=0;//�޿�Ƭ
								CircularRingLossCartState=0;
							  P_encoder[0]=0;
							  P_encoder[1]=0;
							  P_encoder[2]=0;
							  P_encoder[3]=0;
						}
					}
					else if(MotionStateFlag==0)//Ѳ�߶���Ƭ
					{
						
						if(LossCartStartFlag==0)
						{
								CartStateFlag=0;//�޿�Ƭ
								ScanLineLossCartState=0;
							  P_encoder[0]=0;
							  P_encoder[1]=0;
							  P_encoder[2]=0;
							  P_encoder[3]=0;
						}
					}
					
				}

			 
	
		}
		else if(WareHouseState==3)
		{
			uint8 flag=0;
			flag=WareHouseAngleSet(WareHouseCurrentAngle,5);
//			
		}
		rt_thread_delay(10);
		
	}
}
