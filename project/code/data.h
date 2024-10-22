#ifndef _DATA_H
#define _DATA_H


#include "zf_common_headfile.h"

#define Chess_H 7 //���̸߶�
#define Black_Max 5//��ɫ��������

#define ADC_Angle_Middle 2078//ADC��ֵ����2312��ʱ��Ϊ0
#define Chess_width 3//���̿��

extern uint16 Camera_x;//��������
extern uint16 Camera_y;

extern int real_x;//ʵ������
extern int real_y;

//����λ��״̬
typedef enum
{
	Empty,
	Black_Chess,
	White_Chess

};
//����λ������
struct Chess_Data
{
	unsigned char Chess_Index;//����λ�����
	unsigned char Chess_State;//����״̬��0�գ�1���壬2����
	float X_Position;//���̸���λ�õ�x����
	float Y_Position;//���̸���λ�õ�y����
};
extern struct Chess_Data ChessBoard[3][3];//����һ��3*3�����̣�ÿ������λ���ж�Ӧ������Ϣ
extern uint8 board[3][3];//������ʱ���������ڸ�����������
extern uint8 board_Buff[3][3];//�������껺��
extern uint8 ChessPositionIndex;
extern uint8 ChessState;//����״̬��0�����ˣ�1��
extern float BlacePiece[10][2];
extern float WhitePiece[10][2];

extern uint8 WhitePieceCount;
extern uint8 BlackPieceCount;

extern float ChessAngle;

extern uint8 Max_Robot;//����Ĭ�Ϻ�ɫ����
extern uint8 Min_People;

extern uint8 Communication_State;//ͨ��״̬��0���У�1��ʼ�����źŸ�openart���֣�2���ֳɹ���ʼ��ȡ���������������꣬�ڻ�ȡ������������
extern uint8 GradeFlag;//���±�־λ������Ŀ���ص�����״̬ʱ���Ը�������
uint8 PlayChess(uint8 State,uint8 Index);
uint8 ClearAllChess();
void InitChessBoard();
void GainCartPosition();
float ADC_TurnAngle(uint16 ADC);
void GainChessPosition(float Angle);
uint8 CheckSavePieceBuff();
uint8 bestMove();
int clip(int x, int min, int max);
#endif