#ifndef _WAREHOUSE_H_
#define _WAREHOUSE_H_
#include "zf_common_headfile.h"

#define WareHouseIndex1 0x00
#define WareHouseIndex2 0x01
#define WareHouseIndex3 0x02
#define WareHouseIndex4 0x03
#define WareHouseIndex5 0x04
#define WareHouseIndex6 0x05
#define WareHouseIndex7 0x06
#define WareHouseIndex8 0x07

#define WareHousePin1 B15
#define WareHousePin2 B21
#define WareHousePin3 B11
#define WareHouseEnablePin D2

#define WareHouseAngleErr 45;//ÿ������֮��Ƕȼ��




typedef struct //���սṹ��
{
	uint16 WareHouseLeftUnloadAngle;//�������ж���Ƕ�
	uint16 WareHouseRightUnloadAngle;//�����Ҳ�ж���Ƕ�
	uint16 WareHouseLoadAngle;//����װ���Ƕ�
	uint8  WareHouseState;//����״̬��0�գ�1װ�л���
	uint8  WareHouseLabel;//���ձ�ǩ

}WareHouseStructure;

typedef struct 
{
	uint8 WareHouseLabel;//���ձ�ǩ
	uint8 CartLabel;//��Ӧ��Ƭ��ǩ
	uint8 state;

}WareHouseLabelPair;
enum CartLabel
{
	Label_1=0,
	Label_2,
	Label_3,
	
	Label_A=18,
	Label_B,
	Label_C,
	Label_D,
	Label_E,
	Label_F,
	Label_G,
	Label_H,
	Label_I,
	Label_J,
	Label_K,
	Label_L,
	Label_M,
	Label_N,
	Label_O
	
};
extern uint8 WareHouseIndex;//��ǰ��������,
extern WareHouseStructure WareHouse[8];//8������
extern uint8 WareHouseIndex;//��ǰ��������,
extern uint8 WareHouseState;//����״̬��0װ����1ж����2,����ĸ�����û�й��ϣ��ػ���
extern float WareHouseCurrentAngle;//��ǰ���ն�����ڵĽǶ�

void WareHouseInit(void);
void OpenWarehouse(unsigned char Index);
uint8 WareHouseAngleSet(float angle, float speed);//ר�����ڿ��ƻ��յĶ��,�ٶȽ���ȡֵ1.5~5
void WareHouseControl();
uint8 WareHouseChoice(uint8 Kind);
uint8 WareHouseMatching(uint8 Kind);
void LabelClose();
#endif