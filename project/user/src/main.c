#include "zf_common_headfile.h"

/*

中断号在MIMXRTRT1064.h
中断函数在isr.h
中断组初始化会在clock_init内部调用
自定义中断函数需要在MIMXRTRT1064.h声明

*/



/**************************信号量**************************/
/***************************线程****************************/
//LED线程，用于显示单片机是否正常工作

//main线程优先级为4，最小优先级8
//main线程作为提取赛道元素进程

int main(void)
{
	
	/****************************************/

    while(1)
    {
			 rt_thread_delay(20);//加入延时函数让main线程一直挂起
			 
		                          

    }
		return 0;

}




 



