#include "pid.h"
#include "control.h"
float Velocity_KP[4]={100,100,100,100};
float Velocity_KI[4]={10,10,10,10};
float Velocity_KD[4]={0,0,0,0};

float MotorCompensate[4]={0,0,0,0};//电机补偿
float target_motor[4];//四个轮子目标速度
float pid_motor[4]={0,0,0,0};
									 
float Position_KP[4]={25,25,25,25};
float Position_KI[4]={2,2,2,2};                                                                    
float Position_KD[4]={70,70,70,70};


struct TurnPD turnPD;//转向环pid参数
struct TurnPD anglePD;//角度环pid参数
struct PositionPIDTypDef_t PatrolLinePID;
struct PositionPIDTypDef_t TurnSpeedPID;

struct PositionPIDTypDef_t DisplaceXPID;
struct PositionPIDTypDef_t DisplaceYPID;

struct PositionPIDTypDef_t PointsDisplaceXPID;
struct PositionPIDTypDef_t PointsDisplaceYPID;
/*************************
pid参数初始化


**************************/
void PID_PraInit()
{
	PatrolLinePID.KP=-0.1;
	PatrolLinePID.KI=0;
	PatrolLinePID.KD=0.5;
	
	TurnSpeedPID.KP=0.05;
	TurnSpeedPID.KI=15;
	TurnSpeedPID.KD=0;
	
	turnPD.P=1;
	turnPD.D=0.5;
	
	DisplaceXPID.KP=0.8;
	DisplaceXPID.KI=0;
	DisplaceXPID.KD=-0.1;
	
	DisplaceYPID.KP=-1;
	DisplaceYPID.KI=0;
	DisplaceYPID.KD=0;
	
	PointsDisplaceXPID.KP=-1.2;//-0.8; 
	PointsDisplaceXPID.KI=0;
	PointsDisplaceXPID.KD=-5;
	
	PointsDisplaceYPID.KP=-1.1;
	PointsDisplaceYPID.KI=0;
	PointsDisplaceYPID.KD=-5;
	
	
	anglePD.P=-5;
	anglePD.D=1.3;
}
/**************************************************************************
函数功能：速度PI
入口参数：target_motor[4]   encoder[4]
返回  值：无
**************************************************************************/
void Incremental_PI(float target_motor[4],float encoder[4])
{ 			
		static float Bias[4],Last_bias[4],Last_last_bias[4],Motor_PID[4];
		
		Bias[0]=target_motor[0]-encoder[0];
		Bias[1]=target_motor[1]-encoder[1]; 
		Bias[2]=target_motor[2]-encoder[2]; 
		Bias[3]=target_motor[3]-encoder[3]; 
		
		Motor_PID[0]+=Velocity_KP[0]*(Bias[0]-Last_bias[0])+Velocity_KI[0]*Bias[0]+Velocity_KD[0]*(Bias[0]-2*Last_bias[0]+Last_last_bias[0]);   //增量式PI控制器
		Motor_PID[1]+=Velocity_KP[1]*(Bias[1]-Last_bias[1])+Velocity_KI[1]*Bias[1]+Velocity_KD[1]*(Bias[1]-2*Last_bias[1]+Last_last_bias[1]);   //增量式PI控制器
		Motor_PID[2]+=Velocity_KP[2]*(Bias[2]-Last_bias[2])+Velocity_KI[2]*Bias[2]+Velocity_KD[2]*(Bias[0]-2*Last_bias[2]+Last_last_bias[2]);   //增量式PI控制器
		Motor_PID[3]+=Velocity_KP[3]*(Bias[3]-Last_bias[3])+Velocity_KI[3]*Bias[3]+Velocity_KD[3]*(Bias[3]-2*Last_bias[3]+Last_last_bias[3]);   //增量式PI控制器
		
		Last_bias[0]=Bias[0];//保存上一次偏差 
		Last_bias[1]=Bias[1];//保存上一次偏差
		Last_bias[2]=Bias[2];//保存上一次偏差
		Last_bias[3]=Bias[3];//保存上一次偏差
	
		Last_last_bias[0]=Last_bias[0];
		Last_last_bias[1]=Last_bias[1];
		Last_last_bias[2]=Last_bias[2];
		Last_last_bias[3]=Last_bias[3];
		
//		pid_motor[0]=Motor_PID[0];
//		pid_motor[1]=Motor_PID[1];
//		pid_motor[2]=Motor_PID[2];
//		pid_motor[3]=Motor_PID[3];
		if(Motor_PID[0]>0)
		pid_motor[0]=Motor_PID[0]+MotorCompensate[0];
		else if(Motor_PID[0]<0)
		pid_motor[0]=Motor_PID[0]-MotorCompensate[0];
		else
			pid_motor[0]=0;
		
		//pid补偿，解决电机静态环境下非线性化
		if(Motor_PID[1]>0)
		pid_motor[1]=Motor_PID[1]+MotorCompensate[1];
		else if(Motor_PID[1]<0)
		pid_motor[1]=Motor_PID[1]-MotorCompensate[1];
		else
			pid_motor[1]=0;
		
		if(Motor_PID[2]>0)
		pid_motor[2]=Motor_PID[2]+MotorCompensate[2];
		else if(Motor_PID[2]<0)
		pid_motor[2]=Motor_PID[2]-MotorCompensate[2];
		else
			pid_motor[2]=0;
		
		if(Motor_PID[3]>0)
		pid_motor[3]=Motor_PID[3]+MotorCompensate[3];
		else if(Motor_PID[3]<0)
		pid_motor[3]=Motor_PID[3]-MotorCompensate[3];
		else
		pid_motor[3]=0;
		
		
}

/*************转向环，使用增量式pid*******************
参数：角度误差，角速度，pid参数，用结构体后面可能使用模糊pid使用不同pid参数


*/
float TurnPD(float err,float err_d,struct TurnPD *pid)
{
	float TurnPIDValue=0;
//	if(abs(err)<=5)
//		err=0;
	TurnPIDValue=pid->P*err+pid->D*err_d;
	if(TurnPIDValue>=200)
		TurnPIDValue=200;
		if(TurnPIDValue<=-200)
		TurnPIDValue=-200;
	
	return TurnPIDValue;
}
/***********转向速度环*******************/
float TurnSpeedPD(float err,float target)
{
	static float TurnSpeedpid=0;
	static float TurnSpeedError=0,LastTurnSpeedError=0;//上次角速度误差 
	static float LastTurnSpeedSum=0;//上次角速度积分
	static float Last_last_TurnSpeedError=0;
	
	TurnSpeedError=err-target;
	
	LastTurnSpeedSum=TurnSpeedError-LastTurnSpeedError;//积分
	
	
		if(LastTurnSpeedSum>=20)
		LastTurnSpeedSum=20;
	if(LastTurnSpeedSum<=-20)
		LastTurnSpeedSum=-20;
	TurnSpeedpid=TurnSpeedpid+TurnSpeedPID.KP*TurnSpeedError+TurnSpeedPID.KI*LastTurnSpeedSum+TurnSpeedPID.KD*(TurnSpeedError-2*LastTurnSpeedError+Last_last_TurnSpeedError);
	
		if(TurnSpeedpid>=300)
		TurnSpeedpid=300;
		if(TurnSpeedpid<=-300)
		TurnSpeedpid=-300;
	Last_last_TurnSpeedError=LastTurnSpeedError;
	LastTurnSpeedError=TurnSpeedError;
	
	return TurnSpeedpid;
}
/********巡线位置环*******************
采用pi控制

*/
float PatrolLinePositionPID()
{
	float PIDValue=0;
	//误差限幅
	if(abs(Y_Error)<=2)
		Y_Error=0;
	
	sumYError=sumYError+Y_Error;//求积分项
	//积分限幅
	if(sumYError>=300)
		sumYError=300;
	if(sumYError<=-300)
		sumYError=-300;
	
	
	PIDValue=PatrolLinePID.KP*Y_Error+PatrolLinePID.KI*sumYError+PatrolLinePID.KD*Y_Error_D;
	
	
	
return PIDValue;
}

/*********位移环***************

输入：目标值，当前值，标志位（0为x轴方向位移，1为y方向位移）
**/


float displacePID(float target_motor,float Distance,uint8 flag)
{
	float x_dis_pid,y_dis_pid;
	float a=0.5,c1=0,c2=0,c3=0;
	static float X_Distance_Bias,Last_X_Distance_Bia,X_Distance_Sum;
	static float Y_Distance_Bias,Last_Y_Distance_Bia,Y_Distance_Sum;
//	static float X_Dis_D=0,Last_X_Dis_D=0;
		if(flag==0)
		{
			//微分先行
			X_Distance_Bias=target_motor-Distance;
//			c1=a*DisplaceXPID.KD+DisplaceXPID.KP;
//			c2=a*DisplaceXPID.KD;
//			c3=DisplaceXPID.KD+DisplaceXPID.KP;
			
		//	X_Dis_D=(c2/c1)*Last_X_Dis_D+(c3/c1)*Distance+(DisplaceXPID.KD/c1)*Last_X_Distance_Bia;
		//	Last_X_Dis_D=X_Dis_D;
			if(abs(X_Distance_Bias)<=5)
			X_Distance_Sum=X_Distance_Sum+X_Distance_Bias;
			else
				X_Distance_Sum=0;
			x_dis_pid=DisplaceXPID.KP*X_Distance_Bias+DisplaceXPID.KI*X_Distance_Sum;//+X_Dis_D;   //增量式PI控制器
			Last_X_Distance_Bia=Distance;
			
			if(abs(X_Distance_Bias)<1)
				x_dis_pid=0;
			if(x_dis_pid>=200)
			x_dis_pid=200;
			if(x_dis_pid<=-200)
			x_dis_pid=-200;
			return x_dis_pid;
		}
		else
		{
			Y_Distance_Bias=target_motor-Distance;
			if(abs(Y_Distance_Bias)<=5)
			Y_Distance_Sum=Y_Distance_Sum+X_Distance_Bias;
			else
				Y_Distance_Sum=0;
			y_dis_pid=DisplaceYPID.KP*Y_Distance_Bias+DisplaceYPID.KI*Y_Distance_Sum+DisplaceYPID.KD*(Y_Distance_Bias-Last_Y_Distance_Bia);   //增量式PI控制器
			
			Last_Y_Distance_Bia=Y_Distance_Bias;
			if(abs(Y_Distance_Bias)<=1)
				y_dis_pid=0;
			if(y_dis_pid>=200)
			y_dis_pid=200;
			if(y_dis_pid<=-200)
			y_dis_pid=-200;
			

			return y_dis_pid;
			
		}
		

}

// 点方向进行平移
float PointsDisplacePID(float target_motor,float Distance,uint8 flag)
{

	float X_Points_dis_PID,Y_Points_dis_PID;
	float X_Points_dis_Bias,Y_Points_dis_Bias;
	static float X_Points_dis_Sum,Y_Points_dis_Sum,Last_X_Points_Bia,Last_Y_Points_Bia;
	
		if(flag==0)
		{
			X_Points_dis_Bias=target_motor-Distance;
			
			
			if(abs(X_Points_dis_Bias)<=5)
			X_Points_dis_Sum=X_Points_dis_Sum+X_Points_dis_Bias;
			else
				X_Points_dis_Sum=0;
			X_Points_dis_PID=PointsDisplaceXPID.KP*X_Points_dis_Bias+PointsDisplaceXPID.KI*X_Points_dis_Sum+PointsDisplaceXPID.KD*(X_Points_dis_Bias-Last_X_Points_Bia);   //增量式PI控制器
			Last_X_Points_Bia=X_Points_dis_Bias;
			
			if(abs(X_Points_dis_Bias)<1)
				X_Points_dis_PID=0;
			if(X_Points_dis_PID>=200)
			X_Points_dis_PID=200;
			if(X_Points_dis_PID<=-200)
			X_Points_dis_PID=-200;
			return X_Points_dis_PID;
		}
		else
		{
			Y_Points_dis_Bias=target_motor-Distance;
			
			Y_Points_dis_PID=PointsDisplaceYPID.KP*Y_Points_dis_Bias+PointsDisplaceYPID.KI*Y_Points_dis_Sum+PointsDisplaceYPID.KD*(Y_Points_dis_Bias-Last_Y_Points_Bia);   //增量式PI控制器
			
			Last_Y_Points_Bia=Y_Points_dis_Bias;
			if(abs(Y_Points_dis_Bias)<=1)
				Y_Points_dis_PID=0;
			if(Y_Points_dis_PID>=200)
			Y_Points_dis_PID=200;
			if(Y_Points_dis_PID<=-200)
			Y_Points_dis_PID=-200;
			

			return Y_Points_dis_PID;
			
		}
		

}
//角度环
float AnglePID(float target,float angle,float angle_speed)
{
	float pid=0;
	float err=0;
	err=target-angle;
	
	if(abs(err)<=2)
		err=0;
	pid=anglePD.P*err+anglePD.D*angle_speed;
	
	return pid;

}
