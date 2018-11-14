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
#include "can.h"
/************************************************
 ALIENTEKս��STM32������ʵ��4
 ����ʵ�� 
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/
#define delay_ms_value 20000  // 20000*1ms=20s
#define timer_out_cnt 2000 //2000*100ms=200s


	u8 canbuf[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x7,0x08};
	u8 canbuf_rx[8];
 	u8 can_pid_table[4]={Engine_load,Engine_RPM,Velicele_speed,Throttle_position};
	 u8 cnt=0,res=0;
 void can_tx(void)
 {
	 u8 i=0;
	 for(i=0;i<8;i++)
	 {
		 canbuf[i]=i+cnt++;//???????
		 canbuf_rx[i]=0; //clean canbuf_rx
	 }
	 res=Can_Send_Msg(canbuf,8);//??8??? 
 
 }
 
 void can_rx(void)
 {
	 u8 key=0,i=0;
	 key=Can_Receive_Msg(canbuf_rx);
	 if(key)//??????
		 {			 
			 
			 for(i=0;i<key;i++)
			 {
				 printf("can_data:%d\r\n",canbuf_rx[i]);
				 //if(i<4)LCD_ShowxNum(60+i*32,270,canbuf[i],3,16,0X80); //????
				 //else LCD_ShowxNum(60+(i-4)*32,290,canbuf[i],3,16,0X80);	 //????
			 }
		 }
 }
 
 


#if 0

 /*******************************************************************************
 * Function Name  : RCC_Configuration 
 * Description	  :  RCC����(ʹ���ⲿ8MHz����)
 * Input			: ��
 * Output		  : ��
 * Return		  : ��
 *******************************************************************************/
 void RCC_Configuration(void)
 {
   ErrorStatus HSEStartUpStatus;
   /*������RCC�Ĵ�������Ϊȱʡֵ*/
   RCC_DeInit();
  
   /*�����ⲿ���پ���HSE��*/
   RCC_HSEConfig(RCC_HSE_ON);	//RCC_HSE_ON����HSE�����(ON)
  
   /*�ȴ�HSE����*/
   HSEStartUpStatus = RCC_WaitForHSEStartUp();
  
   if(HSEStartUpStatus == SUCCESS)		  //SUCCESS��HSE�����ȶ��Ҿ���
   {
	 /*����AHBʱ�ӣ�HCLK��*/ 
	 RCC_HCLKConfig(RCC_SYSCLK_Div1);  //RCC_SYSCLK_Div1����AHBʱ��= ϵͳʱ��
  
	 /* ���ø���AHBʱ�ӣ�PCLK2��*/ 
	 RCC_PCLK2Config(RCC_HCLK_Div1);   //RCC_HCLK_Div1����APB2ʱ��= HCLK
  
	 /*���õ���AHBʱ�ӣ�PCLK1��*/	 
 RCC_PCLK1Config(RCC_HCLK_Div2);   //RCC_HCLK_Div2����APB1ʱ��= HCLK / 2
  
	 /*����FLASH�洢����ʱʱ��������*/
	 FLASH_SetLatency(FLASH_Latency_2);    //FLASH_Latency_2  2��ʱ����
	
  /*ѡ��FLASHԤȡָ�����ģʽ*/  
	 FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);		 // Ԥȡָ����ʹ��
  
	 /*����PLLʱ��Դ����Ƶϵ��*/ 
	 RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);	  
 // PLL������ʱ��= HSEʱ��Ƶ�ʣ�RCC_PLLMul_9����PLL����ʱ��x 9
	
   /*ʹ��PLL */
	 RCC_PLLCmd(ENABLE); 
  
	 /*���ָ����RCC��־λ(PLL׼���ñ�־)�������*/   
	 while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) 	 
		{
		}
  
	 /*����ϵͳʱ�ӣ�SYSCLK��*/ 
	 RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); 
 //RCC_SYSCLKSource_PLLCLK����ѡ��PLL��Ϊϵͳʱ��
  
	 /* PLL��������ϵͳʱ�ӵ�ʱ��Դ*/
	 while(RCC_GetSYSCLKSource() != 0x08)		 //0x08��PLL��Ϊϵͳʱ��
		{ 
		}
	  }
  
  /*ʹ�ܻ���ʧ��APB2����ʱ��*/	  
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
 RCC_APB2Periph_GPIOC , ENABLE); 
 //RCC_APB2Periph_GPIOA    GPIOAʱ��
 //RCC_APB2Periph_GPIOB    GPIOBʱ��
 //RCC_APB2Periph_GPIOC    GPIOCʱ��
 //RCC_APB2Periph_GPIOD    GPIODʱ��
 }


#endif
 u8 PA0_FLAG_BAK=1,PA1_FLAG_BAK=1,IO_CHANGE_FLAG=0;

 void Send_Byte(u8 byte)
 {
	 USART_SendData(USART1, byte);
	 while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
 }


 

void TIM3_DELAY_MS_(u16 times)
{
	TIM3_DELAY_MS=times;
	while(TIM3_DELAY_MS);
		
}

 int main(void)
 {		
 	u8 t,len;
	u8 can_pid_cnt=0;
	//u16 PWM_VALUE=0;	
	//u8 dir=1;
	//float adcx=0;
	//u16 PPM_VALUE=0;
	//u8 KEY_VALUE=0;
//	u16 timer_count=timer_out_cnt;
	//u8 DATA_CMD[]={0xA6,0xA8,0x55,0x66};
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD
		|RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOF|RCC_APB2Periph_GPIOG|RCC_APB2Periph_AFIO, ENABLE);	 //ʹ��PA,PB,PC,PD,PE,PF,PG�˿�ʱ��
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);   	//ȡ��JATG���� ����ͨIO �á�
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	//uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
	uart1_init(115200);
	printf("uart1_init_ok\r\n");
 	LED_Init();			     //LED�˿ڳ�ʼ��
	//KEY_Init();          //��ʼ���밴�����ӵ�Ӳ���ӿ�
	//Adc_Init();		  		//ADC3��ʼ��
	//LCD_Init();		//LCD ��ʼ��
	//EXTIX_Init();		//�ⲿ�жϳ�ʼ��
	
	//TIM1_PWM_Init(1999,71);//����Ƶ��PWMƵ��=72000/(899+1)=80Khz 10K   2ms -1
	//TIM1_PWM_Init(1999,71);//����Ƶ��PWMƵ��=72000/(899+1)=80Khz 10K   2ms
	TIM3_Int_Init(9,7199);//10Khz�ļ���Ƶ�ʣ�������5000Ϊ500ms -1
	
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);//CAN???????,???500Kbps  
	//usmart_dev.init(SystemCoreClock/1000000);	//��ʼ��USMART	
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
	delay_ms(1000);
	//PA0_FLAG_BAK=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);
 	//PA1_FLAG_BAK=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1);
	//delay_ms(100);
	// while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
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
		
		#if 0
		if(INT0_printf_FLAG) //��ʼ
		{
			INT0_printf_FLAG=0;
			Send_Byte(0x20);
			GPIO_SetBits(GPIOA,GPIO_Pin_4);
		}
		
		if(INT1_printf_FLAG)//��ʼ
		{
			INT1_printf_FLAG=0;
			Send_Byte(0x08);
			GPIO_SetBits(GPIOA,GPIO_Pin_5);
		}
		

		if((INT0_FLAG==PP_CONT_START)&(INT0_timer_cnt==0))//����
		{
			INT0_FLAG=PP_CONT_STOP;
			Send_Byte(0x10);
			GPIO_ResetBits(GPIOA,GPIO_Pin_4);
		}
		if((INT1_FLAG==PP_CONT_START)&(INT1_timer_cnt==0))//����
		{
			INT1_FLAG=PP_CONT_STOP;
			Send_Byte(0x04);
			GPIO_ResetBits(GPIOA,GPIO_Pin_5);
		}
		#endif
		if(TIM3_IRQHandler_count==0)
			{
			TIM3_IRQHandler_count=500;
			if(can_pid_cnt>=4)
				can_pid_cnt=0;
			Can_ISO15765_Send(can_pid_table[can_pid_cnt++],Can_iso15765_Send_id,8);
			//printf("TIM3_IRQHandler_count = %d\n",TIM3_IRQHandler_count);
			//IWDG_Feed();//���Ź���
			//level_dect();
			//can_tx();
			//Can_Send_Msg(canbuf,8);//??8??? 
			
		}
		
		#if 1
		//if(USART_RX_STA&0x8000)
		if((USART_RX_flag)&(TIM3_IRQHandler_UART_CNT==0))
		{					   
			#if 0
			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
			//printf("�����͵���ϢΪ:\r\n");
			for(t=0;t<len;t++)
			{
				USART_SendData(USART1, USART_RX_BUF[t]);//�򴮿�1��������
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			}
			//printf("\r\n");//���뻻��
			#endif
			#if 0
			//if(0 == strncmp("A6A85566",USART_RX_BUF,strlen("A6A85566")))
			if((USART_RX_BUF[0]==0xA6)&(USART_RX_BUF[1]==0xA8)&(USART_RX_BUF[2]==0x55)
				&(USART_RX_BUF[3]==0x66))
				{
				timer_count=timer_out_cnt;
				//printf("CMD_OK\r\n");
				}
			#endif
			
			//Can_Send_Msg(USART_RX_BUF,8);//??8???
			Can_ISO15765_Send(USART_RX_BUF[0],Can_iso15765_Send_id,8);
			
			USART_RX_STA=0;
			USART_RX_flag=0;
		}
		#endif
		
		 
	}	 
 }

