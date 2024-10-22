#ifndef _CAR_MOVE_H
#define _CAR_MOVE_H

#include "zf_common_headfile.h"

void updata_mileage(void);
void Car_move(void);
void reset_mileage(void);
void car_speedplan(void);
void tar_speed_plan(void);
void get_location(void);
void car_recmode(void);
void carmove_mileage(float x,float y);
void car_out(void);
void car_return(void);

typedef struct location_goal {
    //已到达的目标点
    int8_t Position_Pointer;
    //当前速度
    double Speed_X;
    double Speed_Y;
    double Speed_Z;
    //当前位置
    float x;
    float y;
    //目标位置
    float x1;
    float y1;
    //当前姿态
    float Angel;
    //目标姿态
    double Angel_Target;
    //目标距离
    float DistanceX;
    float DistanceY;
    //距上次转向之后前进的距离(里程）
    float MileageX;
    float MileageY;
}location_goal;

extern location_goal Car;
extern float coordin_flag;
extern float Tar_det;//目标距离
extern float Car_dis;
extern float speed_tar;
extern bool finsh_correct;
extern bool action;

#endif