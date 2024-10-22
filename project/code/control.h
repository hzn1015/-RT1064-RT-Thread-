#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "zf_common_headfile.h"

extern uint8 TopicIndex;
extern uint8 TopicIndexCache;//题目索引光标，用于显示在界面上

/***题目1变量*****/
extern uint8 Topic_1_BlackIndex;
extern uint8 Topic_1_Flag;
extern uint8 Topic_1_StartFlag;//题目1运行

/***题目2变量*****/

extern uint8 Topic_2_PieceTargetSelectIndex[4];//棋子目标索引，前两个黑棋，后两个白棋
extern uint8 Topic_2_ChessTargetSelectIndex[4];//棋盘目标索引，前两个黑棋，后两个白棋
extern uint8 Topic_2_PieceSelectIndex;//棋子选择索引，黑棋1，2，白棋3，4
extern uint8 Topic_2_ChessSelectIndex;//棋盘选择索引，黑棋1，2，白棋3，4
extern uint8 Topic_2_Flag;//题目2标志位,0选择棋子，1选择棋盘，2准备运行，3开始运行
extern uint8 Topic_2_StartFlag;//题目2运行标志位
extern uint8 Topic_2_Count;//题目2棋子数量

/***题目4变量******/

extern uint8 Topic_4_Chess_Order;//棋盘顺序，一开始默认机器先下，1为机器下，0为人下
extern uint8 Topic_4_RobotIndex;//机器下棋索引，只有一开始下第一个棋子需要
extern uint8 Topic_4_PeopleIndex;//人下棋索引
extern uint8 Topic_4_PeopleStartFlag;//人开始下棋标志位，由按键设置
extern uint8 Topic_4_Flag;//题目2标志位,0选择棋子，1选择棋盘，2准备运行，3开始运行
extern uint8 Topic_4_StartFlag;

/****题目5变量******/
extern uint8 Topic_5_Chess_Order;//棋盘顺序，一开始默认人先下，1为机器下，0为人下
extern uint8 Topic_5_RobotIndex;//机器下棋索引，只有一开始下第一个棋子需要
extern uint8 Topic_5_RobotStartFlag;//机器人开始标志位
extern uint8 Topic_5_PeopleStartFlag;
extern uint8 Topic_5_PeopleIndex;//人下棋索引
extern uint8 Topic_5_BlackIndex;//黑棋索引
extern uint8 Topic_5_WhiteIndex;//白棋索引
extern uint8 Topic_5_Flag;//题目2标志位,0选择棋子，1选择棋盘，2准备运行，3开始运行
extern uint8 Topic_5_StartFlag;
void ClearTopic1Flag();
void ClearTopic2Flag();
void Topic_4_ClearAllData();
uint8 Topic_4_GainWhitePIece();
void Topic_5_ClearAllData();
void TopicSelect();
#endif