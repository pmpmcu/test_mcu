#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "Adc.h" 
#include "lcd.h"
#include "exti.h"
#include "timer.h"
#include "usmart.h"	
#include "pwm.h" 
#include "spi.h"
#include "stm32f10x_flash.h"
#include "iwdg.h"
/************************************************
 ALIENTEK战舰STM32开发板实验4
 串口实验 
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/
#define delay_ms_value 20000  // 20000*1ms=20s
#define timer_out_cnt 2000 //2000*100ms=200s

#if 0

 /*******************************************************************************
 * Function Name  : RCC_Configuration 
 * Description	  :  RCC配置(使用外部8MHz晶振)
 * Input			: 无
 * Output		  : 无
 * Return		  : 无
 *******************************************************************************/
 void RCC_Configuration(void)
 {
   ErrorStatus HSEStartUpStatus;
   /*将外设RCC寄存器重设为缺省值*/
   RCC_DeInit();
  
   /*设置外部高速晶振（HSE）*/
   RCC_HSEConfig(RCC_HSE_ON);	//RCC_HSE_ON——HSE晶振打开(ON)
  
   /*等待HSE起振*/
   HSEStartUpStatus = RCC_WaitForHSEStartUp();
  
   if(HSEStartUpStatus == SUCCESS)		  //SUCCESS：HSE晶振稳定且就绪
   {
	 /*设置AHB时钟（HCLK）*/ 
	 RCC_HCLKConfig(RCC_SYSCLK_Div1);  //RCC_SYSCLK_Div1——AHB时钟= 系统时钟
  
	 /* 设置高速AHB时钟（PCLK2）*/ 
	 RCC_PCLK2Config(RCC_HCLK_Div1);   //RCC_HCLK_Div1——APB2时钟= HCLK
  
	 /*设置低速AHB时钟（PCLK1）*/	 
 RCC_PCLK1Config(RCC_HCLK_Div2);   //RCC_HCLK_Div2——APB1时钟= HCLK / 2
  
	 /*设置FLASH存储器延时时钟周期数*/
	 FLASH_SetLatency(FLASH_Latency_2);    //FLASH_Latency_2  2延时周期
	
  /*选择FLASH预取指缓存的模式*/  
	 FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);		 // 预取指缓存使能
  
	 /*设置PLL时钟源及倍频系数*/ 
	 RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);	  
 // PLL的输入时钟= HSE时钟频率；RCC_PLLMul_9——PLL输入时钟x 9
	
   /*使能PLL */
	 RCC_PLLCmd(ENABLE); 
  
	 /*检查指定的RCC标志位(PLL准备好标志)设置与否*/   
	 while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) 	 
		{
		}
  
	 /*设置系统时钟（SYSCLK）*/ 
	 RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); 
 //RCC_SYSCLKSource_PLLCLK——选择PLL作为系统时钟
  
	 /* PLL返回用作系统时钟的时钟源*/
	 while(RCC_GetSYSCLKSource() != 0x08)		 //0x08：PLL作为系统时钟
		{ 
		}
	  }
  
  /*使能或者失能APB2外设时钟*/	  
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
 RCC_APB2Periph_GPIOC , ENABLE); 
 //RCC_APB2Periph_GPIOA    GPIOA时钟
 //RCC_APB2Periph_GPIOB    GPIOB时钟
 //RCC_APB2Periph_GPIOC    GPIOC时钟
 //RCC_APB2Periph_GPIOD    GPIOD时钟
 }


#endif
 u8 PA0_FLAG_BAK=1,PA1_FLAG_BAK=1,IO_CHANGE_FLAG=0;

 void Send_Byte(u8 byte)
 {
	 USART_SendData(USART1, byte);
	 while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
 }

 void  level_dect(void)
 {
	
	 
	// if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 0)
	 {
		//delay_ms(10);
	 
		 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 0)//??????
		 {
			
			 GPIO_ResetBits(GPIOA,GPIO_Pin_4);	
		  }  
		 else
		 {
			 GPIO_SetBits(GPIOA,GPIO_Pin_4);
		 }	 
	 }	 
 		
	// if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == 0)
	 {
		
		//delay_ms(10);
		 
		 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == 0)//??????
		 {
			
			 GPIO_ResetBits(GPIOA,GPIO_Pin_5);	
		 }	 
		 else
		 {
			 GPIO_SetBits(GPIOA,GPIO_Pin_5); 

		 }	 
   } 
	 
	 //PAout(4)=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);
	// PAout(5)=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1);
	#if 0
	 if((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)!=PA0_FLAG_BAK))
	 {
		 IO_CHANGE_FLAG=1;
	 }
	 
	if((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)!=PA1_FLAG_BAK))
	 {
	 	IO_CHANGE_FLAG=1;
	 }
	 #endif
	 if(PA0_FLAG_BAK==1)
	 	{
	 	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==0)
			IO_CHANGE_FLAG=1;
	 	}
	 if(PA1_FLAG_BAK==1)
	 	{
	 	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)==0)
			IO_CHANGE_FLAG=1;
	 	}
	 
	 PA0_FLAG_BAK=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0); 
	 PA1_FLAG_BAK=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1);
	 
	 if(IO_CHANGE_FLAG==1)//IO 有变化。
		 {
		 	IO_CHANGE_FLAG=0;
		  //Send_Byte(0x10+PA0_FLAG_BAK);// 11 10
		  	Send_Byte(0x10+(~PA0_FLAG_BAK)&0x01);// 11 10
		  //Send_Byte(0x20+PA1_FLAG_BAK);//21 20
			Send_Byte(0x20+(~PA1_FLAG_BAK)&0x01);//21 20
		 }
	 
 }

void TIM3_DELAY_MS_(u16 times)
{
	TIM3_DELAY_MS=times;
	while(TIM3_DELAY_MS);
		
}

 int main(void)
 {		
 	u8 t,len;  
	//u16 PWM_VALUE=0;	
	//u8 dir=1;
	//float adcx=0;
	//u16 PPM_VALUE=0;
	//u8 KEY_VALUE=0;
	u16 timer_count=timer_out_cnt;
	//u8 DATA_CMD[]={0xA6,0xA8,0x55,0x66};
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD
		|RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOF|RCC_APB2Periph_GPIOG|RCC_APB2Periph_AFIO, ENABLE);	 //使能PA,PB,PC,PD,PE,PF,PG端口时钟
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);   	//取消JATG功能 当普通IO 用。
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	//uart_init(115200);	 //串口初始化为115200
	uart1_init(115200);
	printf("uart1_init_ok\r\n");
 	LED_Init();			     //LED端口初始化
	//KEY_Init();          //初始化与按键连接的硬件接口
	//Adc_Init();		  		//ADC3初始化
	//LCD_Init();		//LCD 初始化
	EXTIX_Init();		//外部中断初始化
	
	//TIM1_PWM_Init(1999,71);//不分频。PWM频率=72000/(899+1)=80Khz 10K   2ms -1
	//TIM1_PWM_Init(1999,71);//不分频。PWM频率=72000/(899+1)=80Khz 10K   2ms
	TIM3_Int_Init(9,7199);//10Khz的计数频率，计数到5000为500ms -1
	//usmart_dev.init(SystemCoreClock/1000000);	//初始化USMART	
	//LCD_ShowString(0,115,128,128,12,"LCD_TEST_lcd_test...1234567890abcdefghijlmnopqrstuvwxyz");
	//LCD_TEST(0,0,1,GREEN,BLACK);
	
	//LCD_Color_Fill(50,50,100,100,GREEN);
	//LCD_DrawLine(0,0,100,110,RED);
	//LCD_DrawLine(0,127,127,0,RED);
	//LCD_DrawRectangle(10,10,100,100,RED);
	//LCD_Draw_Circle(63,63,20,RED); //
	//LCD_Draw_Circle(63,63,21,RED); //
	//LCD_Draw_Circle(63,63,22,RED); //
	//LCD_Draw_Circle(63,63,21,RED);
	//LCD_Draw_Circle(63,63,22,RED);
	delay_ms(100);
	PA0_FLAG_BAK=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);
 	PA1_FLAG_BAK=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1);
	delay_ms(100);
	 while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	//Send_Byte(0x10+PA0_FLAG_BAK);// 11 10
	//Send_Byte(0x20+PA1_FLAG_BAK);//21 20
 	while(1)
	{
		//KEY_VALUE=KEY_Scan(0);
		#if 0
		if(KEY_VALUE)
			{
			 printf("KEY_VALUE = %d\n",KEY_VALUE);
			 //t=&KEY_VALUE;
			//LCD_PrintChar8X16_LINE(10,50,t,GREEN,BLACK); 
			//LCD_ShowNum(10,50,KEY_VALUE,2,16,YELLOW,BLACK);
			 if(KEY_VALUE==KEY11_PRES)
			 	PCout(13)=0;
			}
		#endif
		if(INT0_printf_FLAG) //开始
		{
			INT0_printf_FLAG=0;
			Send_Byte(0x20);
			GPIO_ResetBits(GPIOA,GPIO_Pin_4);
		}
		
		if(INT1_printf_FLAG)//开始
		{
			INT0_printf_FLAG=0;
			Send_Byte(0x08);
			GPIO_ResetBits(GPIOA,GPIO_Pin_5);
		}

		if((INT0_FLAG==PP_CONT_START)&(INT0_timer_cnt==0))//结束
		{
			INT0_FLAG=PP_CONT_STOP;
			Send_Byte(0x10);
			GPIO_SetBits(GPIOA,GPIO_Pin_4);
		}
		if((INT1_FLAG==PP_CONT_START)&(INT1_timer_cnt==0))//结束
		{
			INT1_FLAG=PP_CONT_STOP;
			Send_Byte(0x04);
			GPIO_SetBits(GPIOA,GPIO_Pin_5);
		}
		
		if(TIM3_IRQHandler_count==0)
			{
			TIM3_IRQHandler_count=100;
			IWDG_Feed();//看门狗。
			//level_dect();
			if(timer_count>0)
				timer_count--;
			if(timer_count==0)
			{
				//timer_count =0;
				/*????????*/
				GPIO_ResetBits(GPIOA,GPIO_Pin_2);
				 TIM3_DELAY_MS_(delay_ms_value);//ms
				/*??20S*/
				//delay_ms(5000);
				GPIO_SetBits(GPIOA,GPIO_Pin_2);
				timer_count =timer_out_cnt;
			//printf("TIM3_IRQHandler_count\r\n");
			//adcx=Get_Adc_Average(ADC_Channel_8,10);
			//adcx=(adcx*100)/4096*326/100*2;   //转为电压
			//adcx=adcx/4096*3.28*2;
			//printf("adc8= %f",adcx);
			//LCD_ShowNum(10,70,adcx*100,3,16,BLUE,BLACK);
			//adcx=Get_Adc_Average(ADC_Channel_0,10);
			//PPM_VALUE=adcx;
			//adcx=(adcx*100)/4096*326/100*2;   //转为电压
			//adcx=adcx/4096*3.28;
			//printf("PPM_VALUE= %u\n",PPM_VALUE);
			//PPM_VALUE=(PPM_VALUE-500)/3+1000;
			//if(PPM_VALUE<1000)
				//PPM_VALUE=1000;
			//if(PPM_VALUE>1900)
				//PPM_VALUE=1900;
			//TIM_SetCompare1(TIM1,PPM_VALUE);
			
			//LCD_ShowNum(10,90,adcx*100,3,24,GREEN,BLACK);

			//if((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8))==0)//读取按键0
			       //printf("charge ON \n");
	 		}
		}
		
		#if 1
		//if(USART_RX_STA&0x8000)
		if((USART_RX_flag)&(TIM3_IRQHandler_UART_CNT==0))
		{					   
			#if 0
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
			//printf("您发送的消息为:\r\n");
			for(t=0;t<len;t++)
			{
				USART_SendData(USART1, USART_RX_BUF[t]);//向串口1发送数据
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
			}
			printf("\r\n");//插入换行
			#endif
			//if(0 == strncmp("A6A85566",USART_RX_BUF,strlen("A6A85566")))
			if((USART_RX_BUF[0]==0xA6)&(USART_RX_BUF[1]==0xA8)&(USART_RX_BUF[2]==0x55)
				&(USART_RX_BUF[3]==0x66))
				{
				timer_count=timer_out_cnt;
				//printf("CMD_OK\r\n");
				}
			USART_RX_STA=0;
			USART_RX_flag=0;
		}
		#endif
		
		 
	}	 
 }

