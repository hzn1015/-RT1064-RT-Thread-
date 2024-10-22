#ifndef _DATA_H
#define _DATA_H


#include "zf_common_headfile.h"

#define Chess_H 7 //棋盘高度
#define Black_Max 5//黑色棋子数量

#define ADC_Angle_Middle 2078//ADC中值数，2312的时候为0
#define Chess_width 3//棋盘宽度

extern uint16 Camera_x;//像素坐标
extern uint16 Camera_y;

extern int real_x;//实际坐标
extern int real_y;

//棋盘位置状态
typedef enum
{
	Empty,
	Black_Chess,
	White_Chess

};
//棋盘位置数据
struct Chess_Data
{
	unsigned char Chess_Index;//棋盘位置序号
	unsigned char Chess_State;//棋盘状态，0空，1黑棋，2白棋
	float X_Position;//棋盘格子位置的x坐标
	float Y_Position;//棋盘格子位置的y坐标
};
extern struct Chess_Data ChessBoard[3][3];//定义一个3*3的棋盘，每个棋盘位置有对应棋盘信息
extern uint8 board[3][3];//棋盘临时变量，用于更新棋盘坐标
extern uint8 board_Buff[3][3];//棋盘坐标缓存
extern uint8 ChessPositionIndex;
extern uint8 ChessState;//下棋状态，0机器人，1人
extern float BlacePiece[10][2];
extern float WhitePiece[10][2];

extern uint8 WhitePieceCount;
extern uint8 BlackPieceCount;

extern float ChessAngle;

extern uint8 Max_Robot;//机器默认黑色先手
extern uint8 Min_People;

extern uint8 Communication_State;//通信状态，0空闲，1开始发送信号给openart握手，2握手成功开始获取棋盘两侧棋子坐标，在获取棋盘中心棋子
extern uint8 GradeFlag;//更新标志位，当题目返回到闲置状态时可以更新坐标
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