#ifndef __EXTI_H
#define __EXIT_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//�ⲿ�ж� ��������			   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/12/01  
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved	  
////////////////////////////////////////////////////////////////////////////////// 	  
#define PP_CONT_START 	1
#define PP_CONT_STOP	0
						    
#define INT0_timer_cnt_val 10	
#define INT1_timer_cnt_val 10


void EXTIX_Init(void);//IO��ʼ��
extern char EXTIX_Init_flag;
extern u8 INT0_FLAG;
extern u8 INT1_FLAG;
extern u8 INT0_printf_FLAG;
extern u8 INT1_printf_FLAG;

extern	u16 INT0_timer_cnt;
extern	u16 INT1_timer_cnt;

#endif

