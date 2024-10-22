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

#define WareHouseAngleErr 45;//每个货舱之间角度间隔




typedef struct //货舱结构体
{
	uint16 WareHouseLeftUnloadAngle;//货舱左侧卸货角度
	uint16 WareHouseRightUnloadAngle;//货舱右侧卸货角度
	uint16 WareHouseLoadAngle;//货舱装货角度
	uint8  WareHouseState;//货舱状态，0空，1装有货物
	uint8  WareHouseLabel;//货舱标签

}WareHouseStructure;

typedef struct 
{
	uint8 WareHouseLabel;//货舱标签
	uint8 CartLabel;//对应卡片标签
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
extern uint8 WareHouseIndex;//当前货舱索引,
extern WareHouseStructure WareHouse[8];//8个货舱
extern uint8 WareHouseIndex;//当前货舱索引,
extern uint8 WareHouseState;//货舱状态：0装货，1卸货，2,检查哪个货舱没有关上，关货舱
extern float WareHouseCurrentAngle;//当前货舱舵机所在的角度

void WareHouseInit(void);
void OpenWarehouse(unsigned char Index);
uint8 WareHouseAngleSet(float angle, float speed);//专门用于控制货舱的舵机,速度建议取值1.5~5
void WareHouseControl();
uint8 WareHouseChoice(uint8 Kind);
uint8 WareHouseMatching(uint8 Kind);
void LabelClose();
#endif