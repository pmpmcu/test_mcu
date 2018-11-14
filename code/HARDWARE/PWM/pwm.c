#include "pwm.h"
#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK Mini STM32开发板
//PWM  驱动代码			   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/12/03
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  
u8 PPM_CHANNEL_FLAG=0,PPM_CHANNEL=0;
u16 PPM_CHANNEL_DATA[11]={1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,1990};


u16 TIM1_IRQHandler_count=0;
u8 TIM1_flag=1;

#define TIM1_PWM
//#define TIM1_PWM_capture


//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM1_PWM_Init(u16 arr,u16 psc)
{  
	 GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef         NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);// 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);  //使能GPIO外设时钟使能
	                                                                     	

   //设置该引脚为复用输出功能,输出TIM1 CH1的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //TIM_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	#ifdef TIM1_PWM   //PWN
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;   //每次都更新事件。
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim   TIM_CKD_DIV4
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 1000; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	


  	TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE 主输出使能	

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1预装载使能	 
	
	TIM_ARRPreloadConfig(TIM1, ENABLE); //使能TIMx在ARR上的预装载寄存器 
	
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE); //使能TIM1 中断  TIM_IT_CC1

	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  //TIM1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	#endif
	
	#ifdef TIM1_PWM_capture    //比较功能实现TIM1_CH1管脚输出指定频率的脉冲
	
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;   //每次都更新事件。
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim   TIM_CKD_DIV4
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE 主输出使能	

	//TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1预装载使能	 
	
	//TIM_ARRPreloadConfig(TIM1, ENABLE); //使能TIMx在ARR上的预装载寄存器 
	TIM_ITConfig(TIM1,TIM_IT_CC1,ENABLE); //使能TIM1 中断  TIM_IT_CC1
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;

	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	#endif
	
	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
 
   
}

void TIM1_UP_IRQHandler(void)   
{
 if ( TIM_GetITStatus(TIM1 , TIM_IT_Update) != RESET ) 
 { 
  TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);    //清除中断标志
   //if(TIM1_IRQHandler_count>0)
   	//TIM1_IRQHandler_count--;

   GPIOA->ODR ^= (1<<11);
   #if 1
   if(TIM1_flag)
   	{
   	GPIO_SetBits(GPIOA,GPIO_Pin_11);
	TIM1_flag=0;
   	}
   else
   	{
   	GPIO_ResetBits(GPIOA,GPIO_Pin_11);
	TIM1_flag=1;
   	}
   #endif
 }    
}

TIM1_CC_IRQHandler(void)

{
//   u16 capture;

    if(TIM_GetITStatus(TIM1,TIM_IT_CC1) == SET)

    	{

       TIM_ClearITPendingBit(TIM1, TIM_IT_CC1 );
	   //PPM_CHANNEL_FLAG=1;
	   GPIOA->ODR ^= (1<<11);
	   TIM_SetCounter(TIM1,0x0000); //清零脉冲计数器
	   if(PPM_CHANNEL<10)
	  	{
	  	//capture = TIM_GetCapture1(TIM1);
			//TIM_SetCompare1(TIM1, capture+PPM_CHANNEL_DATA[PPM_CHANNEL]);
			if(TIM1_flag)
				{
	 			TIM_SetCompare1(TIM1,PPM_CHANNEL_DATA[PPM_CHANNEL]);
				TIM1_flag=0;
				}
			else
				{
				TIM_SetCompare1(TIM1,(2000-(PPM_CHANNEL_DATA[PPM_CHANNEL])));
				TIM1_flag=1;
				PPM_CHANNEL++;
				}
  		}

	if(PPM_CHANNEL==10)
		PPM_CHANNEL=0;


	#if 0
 		if((PPM_CHANNEL>9)&(PPM_CHANNEL<17))   //关闭 4ms
			{
			TIM_CtrlPWMOutputs(TIM1,DISABLE);	//MOE 主输出使能
			TIM_SetCompare1(TIM1,1000);
			PPM_CHANNEL++;
			}
	if(PPM_CHANNEL==17)  //关闭 4ms 后打开
			{
			TIM1_flag=1;
			PPM_CHANNEL=0;
			TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE 主输出使能	
			
			if(TIM1_flag)
				{
	 			TIM_SetCompare1(TIM1,PPM_CHANNEL_DATA[PPM_CHANNEL]);
				TIM1_flag=0;
				}
			else
				{
				TIM_SetCompare1(TIM1,(2000-(PPM_CHANNEL_DATA[PPM_CHANNEL])));
				TIM1_flag=1;
				PPM_CHANNEL++;
				}
			}
	
	#endif
	
		//if((PPM_CHANNEL==10))   //关闭 4ms
			{
			//TIM_SetCompare1(TIM1,1900);
			//TIM_CtrlPWMOutputs(TIM1,DISABLE);	//MOE 主输出使能
			//PPM_CHANNEL++;
			}
		
		
	
    	}
}


