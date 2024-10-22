#ifndef _PID_H
#define _PID_H

#include "zf_common_headfile.h"

#define DUTY_MAX 2500 //待补充
#define TurnPidMax 1000//转向环pid限幅


/*********转向环参数***************/
struct TurnPD
{
	float P;
	float D;

};
struct PositionPIDTypDef_t
{
	float KP;
	float KI;
	float KD;
	
};

extern float Velocity_KP[4];
extern float Velocity_KI[4];
extern float Velocity_KD[4];

extern struct PositionPIDTypDef_t PatrolLinePID;
extern struct TurnPD turnPD;//角
extern struct TurnPD anglePD;//角度环pid参数
extern float target_motor[4];//四个轮子目标速度
extern float pid_motor[4];

extern struct PositionPIDTypDef_t DisplaceXPID;
extern struct PositionPIDTypDef_t DisplaceYPID;

extern struct PositionPIDTypDef_t PointsDisplaceXPID;
extern struct PositionPIDTypDef_t PointsDisplaceYPID;


void PID_PraInit();
float TurnPD(float err,float err_d,struct TurnPD *pid);
void Incremental_PI(float target_motor[4],float encoder[4]);
float PatrolLinePositionPID();
float TurnSpeedPD(float err,float target);
float displacePID(float target_motor,float Distance,uint8 flag);
float PointsDisplacePID(float target_motor,float Distance,uint8 flag);
float AnglePID(float target,float angle,float angle_speed);
#endif