#include "car_move.h"

location_goal Car;
float speed_tar = 0;//目标速度
float coordin_flag = 0;//坐标更新标志位
float det_x,det_y;//用来记录目标点和当前位置的横纵轴的偏差值
float Tar_det = 0;//目标距离
int calculate_flag=0;
float Car_dis=0;//小车的实时距离
int Speed_y[5] = {37,42,45,48,50};
bool finsh_correct = false;
bool oncar = false;
bool action = true;
bool back = true;

void updata_mileage(void)
{
		float x,y;
		x = (encoder[1] + encoder[0] + encoder[3] + encoder[2]) * 0.002070f;
		y = (encoder[1] - encoder[3] + encoder[2] - encoder[0]) * 0.002070f;
	  Car.MileageX += x*fastcos(Angle_Z) + y*fastsin(Angle_Z);
		Car.MileageY += x*fastsin(Angle_Z) + y*fastcos(Angle_Z);
		Car_dis=sqrt(Car.MileageX*Car.MileageX+Car.MileageY*Car.MileageY);
}

//获取坐标点函数
void get_location(void)
{
	coordin_flag=1;
	det_x = points[0].x-now_coor[0];
	det_y = points[0].y-now_coor[1];
	Car.Angel_Target=atan2((points[0].x-now_coor[0]),(points[0].y-now_coor[1]))*180/PI;
  Car.DistanceX=20*(points[0].x-now_coor[0]);
  Car.DistanceY=20*(points[0].y-now_coor[1]);
	Tar_det=sqrt((det_x/5)*(det_x/5)+(det_y/5)*(det_y/5));
}


void carmove_mileage(float x,float y)
{
		Car.DistanceX=x;
    Car.DistanceY=y;
    while(abs(Car.MileageX)<abs(Car.DistanceX)) {
        if (x > 0) {
						Vx = speed_tar;
        } else if (x == 0) {
            Vx = 0;
        } else if (x < 0) {
            Vx = -speed_tar;
        }
    }
    Vx=0;
    car_stop();
		system_delay_ms(400);
    while(abs(Car.MileageY)< abs(Car.DistanceY)){
        if (y > 0) {
            Vy = speed_tar;
        } else if (y == 0) {
            Vy = 0;
        } else if (y < 0) {
            Vy = -speed_tar;
        }
    }
		Vy=0;
    car_stop();
		system_delay_ms(400);
    reset_mileage();
}

//根据图片类型搬运到不同的区域
void car_carry()
{
	reset_mileage();
	switch(pictureBigType)//选择图片补偿坐标和距离
	{
		case above :     //上方区域
			Car.DistanceY=20*(26.5-now_coor[1]);
			now_coor[1]=26.5;
			break;
		case left :     //左方区域
			Car.DistanceX=20*(1.2+now_coor[0]);
			now_coor[0]=-1.2;
			break;
		case right :    //右方区域
			Car.DistanceX=20*(now_coor[0]-36.5);
			now_coor[0]=36.5;
			break;
		case below :    //下方区域
			Car.DistanceY=20*(-(now_coor[1]+0.5));
			now_coor[1]=-0.5;
			break;
		case vehicle :
			oncar = true;
			break;
		default:
			break;
	}
	carmove_mileage(Car.DistanceX,Car.DistanceY);
	car_stop();
	reset_mileage();
}
//出库函数
void car_out(void)
{
	action = false;
	speed_tar=120;
  system_delay_ms(200);
	Car.DistanceY = 40;
	Car.DistanceX = 0;
	carmove_mileage(Car.DistanceX,Car.DistanceY);
	now_coor[0] = 0.8;
	now_coor[1] = 2.5;
}

//回库函数
void car_return(void)
{
	if(back){
		back = false;
		action = false;
		openartMode = get_yellow;
		if(now_coor[1]>25)
		{
			Car.DistanceY = -(20*10);
			Car.DistanceX = 0;
			carmove_mileage(Car.DistanceX,Car.DistanceY);
			now_coor[1] = 16.5;
			car_stop();
		}
		else if(now_coor[1]<0)
		{
			Car.DistanceY = 20*5;
			Car.DistanceX = 0;
			carmove_mileage(Car.DistanceX,Car.DistanceY);
			now_coor[1] = 4.5;
			car_stop();
		}
		if(now_coor[0]<0){
			Car.DistanceY = 0;
			Car.DistanceX = 20*(now_coor[0]-0.5);
			carmove_mileage(Car.DistanceX,Car.DistanceY);
			car_stop();
		}
		else{
			Car.DistanceY = 0;
			Car.DistanceX = 20*(now_coor[0]);
			carmove_mileage(Car.DistanceX,Car.DistanceY);
			car_stop();
		}
		while(1)
		{
			uart_write_byte(UART_4,'4');
			system_delay_ms(200);
			if(getyellow)
			{
				Vx = -picture_xerror_pid(yellow[0],45); 
				Vy = 0;
			}
			if(yellow[0]-45<8)
			{
				Vx = 0;
				Vy = 0;
				break;
			}
		}
		Car.DistanceY = 20*(-(now_coor[1]+0.5));;
		Car.DistanceX = 0;
		carmove_mileage(Car.DistanceX,Car.DistanceY);
		car_stop();
		now_coor[0] = 1.4;
		now_coor[1] = -0.5;
		reset_mileage();
	}
}

//到达目标点之后进行矫正，等待识别后抓取图片搬运
void car_recmode(void)
{
	//矫正未写
	int n = 0;
   int tmp=6;
    while(1){
			 uart_write_byte(UART_4,'3');
			system_delay_ms(200);
       if(havepicture) {
				 gpio_set_level(B11,0);
            Vx = -picture_xerror_pid(mod_correct[0], 168);
            Vy =  picture_yerror_pid(mod_correct[1], 141);
            n=0;
        }else{
					gpio_set_level(B11,1);
            //没有图片，搜索附近,没有则直接退出
            reset_mileage();
            n++;
						if (n <= tmp) {
								Vx = 0;
                Vy = -35;
            }else if (n <= tmp * 2) {
								Vy = 0;
								Vx = -35;
            }else if (n <= tmp * 4) {
								Vx = 0;
							Vy = 35;
            }else if (n <= tmp * 6) {
                //Vy = -20;
							Vy = 0;
								Vx = 35;
            }else if (n <= tmp * 8) {
                Vy= -30;
							Vx = 0;
            }else if (n <= tmp * 9) {
                Vx = -35;
							Vy = 0; 
            }else if (n <= tmp * 10) {
                Vx = 0;
							Vy = 35; 
            }else if(n<=tmp*10){
							buzzer();
							Vx = 0;
							Vy = 0;
                reset_mileage();
                break;
					
					
					
//            if (n <= tmp * 10) {
//                Vy = -20;
//            }else if (n <= tmp * 40) {
//                Vy = 0;
//								Vx = -25;
//            }else if (n <= tmp * 50) {
//               Vx = 20;
//							Vy = 0;
//            }else if (n <= tmp * 120) {
//                Vy = -20;
//								Vx = 0;
//            }else if (n <= tmp * 150) {
//                Vx= 10;
//            }else if (n <= tmp * 180) {
//                Vx = -10;
//            }else if(n<=tmp*200){
//							buzzer();
//                reset_mileage();
//                break;
            }
					}
//				system_delay_ms(2000);
//					Vx = 0;
//					Vy = 0;
//					reset_mileage();
//					system_delay_ms(200);
//					break;	
					if(abs(mod_correct[0]-168)<6&&abs(mod_correct[1]-141)<6)
					{
						Vx = 0;
						Vy = 0;
						reset_mileage();
						system_delay_ms(200);
						havepicture = false;
						finsh_correct = true;
						mod_correct[0] = 0;
						mod_correct[1] = 0;
						break;
					}
        }	
			uart_write_byte(UART_4,'2');
			system_delay_ms(1500);
	//识别未写
	Carry(0);
	car_carry();
	if(oncar){
		Carry(2);
		oncar = false;
	}else{
		Carry(1);		
	}
	reset_mileage();
}

void reset_mileage(void)
{
		Vx=0;
    Vy=0;
    Vz=0;
		
    Car.MileageX=0;
    Car.MileageY=0;
    Car.DistanceX=0;
    Car.DistanceY=0;
}

void car_speedplan(void)
{
	get_location();
	bool x_flag=false,y_flag=false;
	if(abs(Car.MileageY) < abs(Car.DistanceY))
	{
		Vy = (speed_tar * cos(Car.Angel_Target/180 *PI));
		y_flag=false;
	}
	else {
		Vy = 0;
		y_flag=true;
	}
	if(abs(Car.MileageX) < abs(Car.DistanceX))
	{
		Vx = -(speed_tar * sin(Car.Angel_Target/180 *PI));
		x_flag=false;
	}
	else {
		Vx = 0;
		x_flag=true;
	}
	 if(x_flag && y_flag){
		 car_stop();
		 updatecoordinates();
		 //Angle_Z=90;
		 reset_mileage();
		 car_recmode();
		 x_flag=false;
		 y_flag=false;	
		 Car.MileageX=0;
     Car.MileageY=0;
		 Car.Position_Pointer++;
    }
}

void Car_move(void)
{
	get_location();
	//speed_tar=50;
	//tar_speed_plan();
	car_speedplan();
	Vz = Turn_PD(90);
	tran_speed(Vx,Vy,Vz);
	Incremental_PI(target_motor,encoder);
	motor_control(pid_motor);
}

