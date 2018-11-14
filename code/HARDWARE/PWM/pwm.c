#include "pwm.h"
#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK Mini STM32������
//PWM  ��������			   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/12/03
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  
u8 PPM_CHANNEL_FLAG=0,PPM_CHANNEL=0;
u16 PPM_CHANNEL_DATA[11]={1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,1990};


u16 TIM1_IRQHandler_count=0;
u8 TIM1_flag=1;

#define TIM1_PWM
//#define TIM1_PWM_capture


//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM1_PWM_Init(u16 arr,u16 psc)
{  
	 GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef         NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);// 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);  //ʹ��GPIO����ʱ��ʹ��
	                                                                     	

   //���ø�����Ϊ�����������,���TIM1 CH1��PWM���岨��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //TIM_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	#ifdef TIM1_PWM   //PWN
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;   //ÿ�ζ������¼���
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim   TIM_CKD_DIV4
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 1000; //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	


  	TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE �����ʹ��	

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1Ԥװ��ʹ��	 
	
	TIM_ARRPreloadConfig(TIM1, ENABLE); //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ��� 
	
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE); //ʹ��TIM1 �ж�  TIM_IT_CC1

	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  //TIM1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	#endif
	
	#ifdef TIM1_PWM_capture    //�ȽϹ���ʵ��TIM1_CH1�ܽ����ָ��Ƶ�ʵ�����
	
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;   //ÿ�ζ������¼���
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim   TIM_CKD_DIV4
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0; //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE �����ʹ��	

	//TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1Ԥװ��ʹ��	 
	
	//TIM_ARRPreloadConfig(TIM1, ENABLE); //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ��� 
	TIM_ITConfig(TIM1,TIM_IT_CC1,ENABLE); //ʹ��TIM1 �ж�  TIM_IT_CC1
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;

	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	#endif
	
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1
 
   
}

void TIM1_UP_IRQHandler(void)   
{
 if ( TIM_GetITStatus(TIM1 , TIM_IT_Update) != RESET ) 
 { 
  TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);    //����жϱ�־
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
	   TIM_SetCounter(TIM1,0x0000); //�������������
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
 		if((PPM_CHANNEL>9)&(PPM_CHANNEL<17))   //�ر� 4ms
			{
			TIM_CtrlPWMOutputs(TIM1,DISABLE);	//MOE �����ʹ��
			TIM_SetCompare1(TIM1,1000);
			PPM_CHANNEL++;
			}
	if(PPM_CHANNEL==17)  //�ر� 4ms ���
			{
			TIM1_flag=1;
			PPM_CHANNEL=0;
			TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE �����ʹ��	
			
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
	
		//if((PPM_CHANNEL==10))   //�ر� 4ms
			{
			//TIM_SetCompare1(TIM1,1900);
			//TIM_CtrlPWMOutputs(TIM1,DISABLE);	//MOE �����ʹ��
			//PPM_CHANNEL++;
			}
		
		
	
    	}
}


