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
    //�ѵ����Ŀ���
    int8_t Position_Pointer;
    //��ǰ�ٶ�
    double Speed_X;
    double Speed_Y;
    double Speed_Z;
    //��ǰλ��
    float x;
    float y;
    //Ŀ��λ��
    float x1;
    float y1;
    //��ǰ��̬
    float Angel;
    //Ŀ����̬
    double Angel_Target;
    //Ŀ�����
    float DistanceX;
    float DistanceY;
    //���ϴ�ת��֮��ǰ���ľ���(��̣�
    float MileageX;
    float MileageY;
}location_goal;

extern location_goal Car;
extern float coordin_flag;
extern float Tar_det;//Ŀ�����
extern float Car_dis;
extern float speed_tar;
extern bool finsh_correct;
extern bool action;

#endif