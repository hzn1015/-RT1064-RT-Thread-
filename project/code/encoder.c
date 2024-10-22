#include "encoder.h"

int32 encoder_fr = 0,encoder_fl = 0,encoder_br = 0,encoder_bl = 0;


void encoder_init(void)
{
	encoder_dir_init(ENCODER_FR_DIR,ENCODER_FR_DIR_PULSE,ENCODER_FR_DIR_DIR);
	encoder_dir_init(ENCODER_FL_DIR,ENCODER_FL_DIR_PULSE,ENCODER_FL_DIR_DIR);
	encoder_dir_init(ENCODER_BR_DIR,ENCODER_BR_DIR_PULSE,ENCODER_BR_DIR_DIR);
	encoder_dir_init(ENCODER_BL_DIR,ENCODER_BL_DIR_PULSE,ENCODER_BL_DIR_DIR);
	
//	pit_ms_init(PIT_CH,100);//初始化定时中断为100ms
//	interrupt_set_priority(PIT_PRIORITY,0);//设定中断优先级
}

void encoder_get(void)
{
	encoder_fr = -encoder_get_count(ENCODER_FR_DIR);
	encoder_fl =  encoder_get_count(ENCODER_FL_DIR);
	encoder_br = -encoder_get_count(ENCODER_BR_DIR);
	encoder_bl =  encoder_get_count(ENCODER_BL_DIR);
}

////每100ms获取一次编码器值
////void pit_handler()
////{
////	encoder1 = -encoder_get_count(ENCODER1_DIR_DIR);
////	encoder2 = encoder_get_count(ENCODER2_DIR_DIR);
////	encoder3 = -encoder_get_count(ENCODER3_DIR_DIR);
////	encoder4 = encoder_get_count(ENCODER4_DIR_DIR);
////	
////	encoder_clear_count(ENCODER1_DIR_DIR);
////	encoder_clear_count(ENCODER2_DIR_DIR);
////	encoder_clear_count(ENCODER3_DIR_DIR);
////	encoder_clear_count(ENCODER4_DIR_DIR);
////}

