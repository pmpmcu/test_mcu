#include "exti.h"
#include "led.h"
#include "key.h"
#include "delay.h"
#include "usart.h"
#include "lcd.h"

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
u8 INT0_FLAG=PP_CONT_STOP;
u8 INT1_FLAG=PP_CONT_STOP;
u8 INT0_printf_FLAG=0;
u8 INT1_printf_FLAG=0;

u16 INT0_timer_cnt=0;
u16 INT1_timer_cnt=0;

 char EXTIX_Init_flag=0;
//�ⲿ�жϳ�ʼ������
void EXTIX_Init(void)
{
 
 	  EXTI_InitTypeDef EXTI_InitStructure;
 	  NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//�ⲿ�жϣ���Ҫʹ��AFIOʱ��

	  KEY_Init();//��ʼ��������Ӧioģʽ

    //GPIOC.0 �ж����Լ��жϳ�ʼ������
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//���½��ش���
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

    //GPIOC.1	  �ж����Լ��жϳ�ʼ������
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line1;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	  	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

    //GPIOA.0	  �ж����Լ��жϳ�ʼ������
  	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource2);

   	//EXTI_InitStructure.EXTI_Line=EXTI_Line2;
  	//EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	////EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	//EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	//EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���


 
  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//�����ȼ�1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//�����ȼ�1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure); 
 
 
   	//NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
  	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
  	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//�����ȼ�1
  	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	//NVIC_Init(&NVIC_InitStructure); 
 
}

 
void EXTI0_IRQHandler(void)
{
	//LCD_ShowString(0,115,128,128,12,"EXTI0_IRQHandler");
  //delay_ms(10);    //����
	//if(WK_UP==1)
	{	  
		//LED0=!LED0;
		//LED1=!LED1;	
	}
   //EXTIX_Init_flag=1;
   INT0_timer_cnt=INT0_timer_cnt_val;
   if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0)
   	{
   		
   		if(INT0_FLAG==PP_CONT_STOP) // �����ֹͣ���½���Ϊ��ʼ��
		{
			INT0_FLAG=PP_CONT_START;// ��Ϊ��ʼ��
			INT0_printf_FLAG=1;//��һ�ο�ʼ��ӡ��
		}
   	}
  
	EXTI_ClearITPendingBit(EXTI_Line0);  //���EXTI0��·����λ
}
 void EXTI1_IRQHandler(void)
{			
	//delay_ms(10);   //����			 
	//if(KEY0==0)	{
		//LED0=!LED0;
	//}
	//EXTIX_Init_flag=2;
	INT1_timer_cnt=INT1_timer_cnt_val;
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)==0)
   	{
   			
   		if(INT1_FLAG==PP_CONT_STOP) // �����ֹͣ���½���Ϊ��ʼ��
		{
			INT1_FLAG=PP_CONT_START;// ��Ϊ��ʼ��
			INT1_printf_FLAG=1;//��һ�ο�ʼ��ӡ��
		}
   	}
  
 	 EXTI_ClearITPendingBit(EXTI_Line1);    //���LINE5�ϵ��жϱ�־λ  
}


void EXTI2_IRQHandler(void)
{
  //delay_ms(10);    //����			 
  //if(KEY1==0)	{
		//LED1=!LED1;
	//}
	//LCD_ShowString(0,115,128,128,12,"EXTI2_IRQHandler");
	EXTIX_Init_flag=3;
	EXTI_ClearITPendingBit(EXTI_Line2);  //���LINE15��·����λ
}
