#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "zf_common_headfile.h"

extern uint8 TopicIndex;
extern uint8 TopicIndexCache;//��Ŀ������꣬������ʾ�ڽ�����

/***��Ŀ1����*****/
extern uint8 Topic_1_BlackIndex;
extern uint8 Topic_1_Flag;
extern uint8 Topic_1_StartFlag;//��Ŀ1����

/***��Ŀ2����*****/

extern uint8 Topic_2_PieceTargetSelectIndex[4];//����Ŀ��������ǰ�������壬����������
extern uint8 Topic_2_ChessTargetSelectIndex[4];//����Ŀ��������ǰ�������壬����������
extern uint8 Topic_2_PieceSelectIndex;//����ѡ������������1��2������3��4
extern uint8 Topic_2_ChessSelectIndex;//����ѡ������������1��2������3��4
extern uint8 Topic_2_Flag;//��Ŀ2��־λ,0ѡ�����ӣ�1ѡ�����̣�2׼�����У�3��ʼ����
extern uint8 Topic_2_StartFlag;//��Ŀ2���б�־λ
extern uint8 Topic_2_Count;//��Ŀ2��������

/***��Ŀ4����******/

extern uint8 Topic_4_Chess_Order;//����˳��һ��ʼĬ�ϻ������£�1Ϊ�����£�0Ϊ����
extern uint8 Topic_4_RobotIndex;//��������������ֻ��һ��ʼ�µ�һ��������Ҫ
extern uint8 Topic_4_PeopleIndex;//����������
extern uint8 Topic_4_PeopleStartFlag;//�˿�ʼ�����־λ���ɰ�������
extern uint8 Topic_4_Flag;//��Ŀ2��־λ,0ѡ�����ӣ�1ѡ�����̣�2׼�����У�3��ʼ����
extern uint8 Topic_4_StartFlag;

/****��Ŀ5����******/
extern uint8 Topic_5_Chess_Order;//����˳��һ��ʼĬ�������£�1Ϊ�����£�0Ϊ����
extern uint8 Topic_5_RobotIndex;//��������������ֻ��һ��ʼ�µ�һ��������Ҫ
extern uint8 Topic_5_RobotStartFlag;//�����˿�ʼ��־λ
extern uint8 Topic_5_PeopleStartFlag;
extern uint8 Topic_5_PeopleIndex;//����������
extern uint8 Topic_5_BlackIndex;//��������
extern uint8 Topic_5_WhiteIndex;//��������
extern uint8 Topic_5_Flag;//��Ŀ2��־λ,0ѡ�����ӣ�1ѡ�����̣�2׼�����У�3��ʼ����
extern uint8 Topic_5_StartFlag;
void ClearTopic1Flag();
void ClearTopic2Flag();
void Topic_4_ClearAllData();
uint8 Topic_4_GainWhitePIece();
void Topic_5_ClearAllData();
void TopicSelect();
#endif