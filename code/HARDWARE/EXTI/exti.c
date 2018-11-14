#include "exti.h"
#include "led.h"
#include "key.h"
#include "delay.h"
#include "usart.h"
#include "lcd.h"

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
u8 INT0_FLAG=PP_CONT_STOP;
u8 INT1_FLAG=PP_CONT_STOP;
u8 INT0_printf_FLAG=0;
u8 INT1_printf_FLAG=0;

u16 INT0_timer_cnt=0;
u16 INT1_timer_cnt=0;

 char EXTIX_Init_flag=0;
//外部中断初始化函数
void EXTIX_Init(void)
{
 
 	  EXTI_InitTypeDef EXTI_InitStructure;
 	  NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟

	  KEY_Init();//初始化按键对应io模式

    //GPIOC.0 中断线以及中断初始化配置
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource12|GPIO_PinSource13);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line12;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//上下降沿触发
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

	EXTI_InitStructure.EXTI_Line=EXTI_Line13;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//上下降沿触发
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
    //GPIOC.1	  中断线以及中断初始化配置
  	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);

  	//EXTI_InitStructure.EXTI_Line=EXTI_Line1;
  	//EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	//EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	//EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	//EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

    //GPIOA.0	  中断线以及中断初始化配置
  	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource2);

   	//EXTI_InitStructure.EXTI_Line=EXTI_Line2;
  	//EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	////EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	//EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	//EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器


 
  	//NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//使能按键所在的外部中断通道
  	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
  	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级1
  	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	//NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
		
	//NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			//使能按键所在的外部中断通道
  	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级1
  	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	//NVIC_Init(&NVIC_InitStructure); 
 
 
   	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//使能按键所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//子优先级1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure); 
 
}

void EXTIX_UART0_can_Init(void) 
{ 
EXTI_InitTypeDef EXTI_InitStructure; 
NVIC_InitTypeDef NVIC_InitStructure; 
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

//KEY_Init();//初始化按键对应io模式

GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource10); 
EXTI_InitStructure.EXTI_Line=EXTI_Line10;
EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
EXTI_InitStructure.EXTI_LineCmd = ENABLE; 
EXTI_Init(&EXTI_InitStructure);

//GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource11); 
//EXTI_InitStructure.EXTI_Line=EXTI_Line11;
//EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
//EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
//EXTI_InitStructure.EXTI_LineCmd = ENABLE; 
//EXTI_Init(&EXTI_InitStructure);

NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; 
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; 
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; 
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
NVIC_Init(&NVIC_InitStructure); 
}  

void EXTIX_UART0_can_DISABLE(void) 
{ 
EXTI_InitTypeDef EXTI_InitStructure; 
NVIC_InitTypeDef NVIC_InitStructure; 
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); 
GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource10); 
EXTI_InitStructure.EXTI_Line=EXTI_Line10;
EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
EXTI_InitStructure.EXTI_LineCmd = DISABLE; 
EXTI_Init(&EXTI_InitStructure); 

//GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource11); 
//EXTI_InitStructure.EXTI_Line=EXTI_Line11;
//EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
//EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
//EXTI_InitStructure.EXTI_LineCmd = DISABLE; 
//EXTI_Init(&EXTI_InitStructure);

NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; 
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; 
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; 
NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE; 
NVIC_Init(&NVIC_InitStructure); 
}  


void EXTI0_IRQHandler(void)
{
	//LCD_ShowString(0,115,128,128,12,"EXTI0_IRQHandler");
  //delay_ms(10);    //消抖
	//if(WK_UP==1)
	{	  
		//LED0=!LED0;
		//LED1=!LED1;	
	}
   //EXTIX_Init_flag=1;
   INT0_timer_cnt=INT0_timer_cnt_val;
   if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0)
   	{
   		
   		if(INT0_FLAG==PP_CONT_STOP) // 如果在停止，下降延为开始。
		{
			INT0_FLAG=PP_CONT_START;// 改为开始。
			INT0_printf_FLAG=1;//第一次开始打印。
		}
   	}
  
	EXTI_ClearITPendingBit(EXTI_Line0);  //清除EXTI0线路挂起位
}
 void EXTI1_IRQHandler(void)
{			
	//delay_ms(10);   //消抖			 
	//if(KEY0==0)	{
		//LED0=!LED0;
	//}
	//EXTIX_Init_flag=2;
	INT1_timer_cnt=INT1_timer_cnt_val;
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)==0)
   	{
   			
   		if(INT1_FLAG==PP_CONT_STOP) // 如果在停止，下降延为开始。
		{
			INT1_FLAG=PP_CONT_START;// 改为开始。
			INT1_printf_FLAG=1;//第一次开始打印。
		}
   	}
  
 	 EXTI_ClearITPendingBit(EXTI_Line1);    //清除LINE5上的中断标志位  
}


void EXTI2_IRQHandler(void)
{
  //delay_ms(10);    //消抖			 
  //if(KEY1==0)	{
		//LED1=!LED1;
	//}
	//LCD_ShowString(0,115,128,128,12,"EXTI2_IRQHandler");
	EXTIX_Init_flag=3;
	EXTI_ClearITPendingBit(EXTI_Line2);  //清除LINE15线路挂起位

}

void EXTI15_10_IRQHandler(void) 
{ 
	EXTI_ClearITPendingBit(EXTI_Line10); 
	EXTI_ClearITPendingBit(EXTI_Line12); 
	EXTI_ClearITPendingBit(EXTI_Line13); 
	EXTIX_Init_flag=15; //一定不要在中断加打印。

} 

