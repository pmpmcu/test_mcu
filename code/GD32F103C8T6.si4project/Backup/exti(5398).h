#ifndef __EXTI_H
#define __EXIT_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//外部中断 驱动代码			   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/12/01  
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved	  
////////////////////////////////////////////////////////////////////////////////// 	  
#define PP_CONT_START 	1
#define PP_CONT_STOP	0
						    
#define INT0_timer_cnt_val 10	
#define INT1_timer_cnt_val 10


void EXTIX_Init(void);//IO初始化
extern char EXTIX_Init_flag;
extern u8 INT0_FLAG;
extern u8 INT1_FLAG;
extern u8 INT0_printf_FLAG;
extern u8 INT1_printf_FLAG;

extern	u16 INT0_timer_cnt;
extern	u16 INT1_timer_cnt;

#endif

