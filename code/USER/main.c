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
//#include "can.h"
#include "back_mirror.h"
#include "stmflash.h"

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
#define FLASH_SAVE_ADDR  0X0800E000		//0X08000000


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
	//u8 can_pid_cnt=0;
	//u16 PWM_VALUE=0;	
	//u8 dir=1;
	//float adcx=0;
	//u16 PPM_VALUE=0;
	u8 KEY_VALUE=0;
	u8 FLASH_write_flag=88;
	u16 FLASH_write_BUF[buffer_len]={88,3001,3002,10003,car_model_rui_hu_5,back_mirror_on};// 0:ID  1:����ʱ��3s��2������ʱ��3s��
							//  3���Զ����ؼ���ʱ��10s   	 	    4:���� 5,�Ƿ�Ҫ������
	u16 FLASH_read_BUF[buffer_len];
	//u16 down_timer=63580; // mm
	//u16 up_timer=63168; // mm
//	u16 timer_count=timer_out_cnt;
	//u8 DATA_CMD[]={0xA6,0xA8,0x55,0x66};
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD
		|RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOF|RCC_APB2Periph_GPIOG|RCC_APB2Periph_AFIO, ENABLE);	 //ʹ��PA,PB,PC,PD,PE,PF,PG�˿�ʱ��
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);   	//ȡ��JATG���� ����ͨIO �á�
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	//uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
	uart1_init(115200);
	
	printf("uart1_init_115200\r\n");
	printf("uart1_init_115200\r\n");
	printf("uart1_init_115200\r\n");
	
	
	uart2_init(115200);
	printf("uart2_init_115200\r\n");
	printf("uart2_init_115200\r\n");
	printf("uart2_init_115200\r\n");
	printf("uart2_init_115200\r\n");
	printf("uart2_init_115200\r\n");
	
	USART_SendData(USART2, 99);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	
 	LED_Init();			     //LED�˿ڳ�ʼ��
	KEY_Init();          //��ʼ���밴�����ӵ�Ӳ���ӿ�
	back_mirror_io_init();
	//Adc_Init();		  		//ADC3��ʼ��
	//LCD_Init();		//LCD ��ʼ��
	EXTIX_Init();		//�ⲿ�жϳ�ʼ��
	
	//TIM1_PWM_Init(1999,71);//����Ƶ��PWMƵ��=72000/(899+1)=80Khz 10K   2ms -1
	//TIM1_PWM_Init(1999,71);//����Ƶ��PWMƵ��=72000/(899+1)=80Khz 10K   2ms
	TIM3_Int_Init(9,7199);//10Khz�ļ���Ƶ�ʣ�������5000Ϊ500ms -1
	
	//CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);//��������CANbps=APB1����Ƶ��36000000/4/(1+8+9))=500k bps   
	//  4  500K    8 250K
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
	//CAN_STB=0; // ��can �շ���
	BT_POWER_EN=1;// ������ģ��.
	delay_ms(10);
	
	//FLASH_write_BUF[0]=FLASH_write_flag; //flash ��д���ԡ�
	//FLASH_write_BUF[1]=down_timer;
	//FLASH_write_BUF[2]=up_timer;
	//FLASH_write_BUF[3]=1234;
	//STMFLASH_Write(FLASH_SAVE_ADDR,FLASH_write_BUF,buffer_len);//д��һ���ֽڡ�
	//STMFLASH_Write(FLASH_SAVE_ADDR,FLASH_write_BUF,1);//д��һ���ֽڡ�
	//FLASH_write_BUF[0]=down_timer;
	//STMFLASH_Write(FLASH_SAVE_ADDR+2,FLASH_write_BUF,1);//д��һ���ֽڡ�
	//FLASH_write_BUF[0]=up_timer;
	//STMFLASH_Write(FLASH_SAVE_ADDR+4,FLASH_write_BUF,1);//д��һ���ֽڡ�
	//FLASH_write_BUF[0]=64321;
	//STMFLASH_Write(FLASH_SAVE_ADDR+6,FLASH_write_BUF,1);//д��һ���ֽڡ�
	
	STMFLASH_Read(FLASH_SAVE_ADDR+0,FLASH_read_BUF, buffer_len);
	if(FLASH_read_BUF[0]==FLASH_write_flag) //�����ٶ�1
		{
		//STMFLASH_Read(FLASH_SAVE_ADDR+2,FLASH_read_BUF, 2);
		//down_timer=FLASH_read_BUF[1];
		//up_timer=FLASH_read_BUF[2];
		motor_down_timer=FLASH_read_BUF[1];
		printf("down_timer:%u\r\n",motor_down_timer);
		motor_up_timer=FLASH_read_BUF[2];
		printf("up_timer:%u\r\n",motor_up_timer);
		car_back_timer=FLASH_read_BUF[3];
		printf("car_back_timer:%u\r\n",car_back_timer);
		car_model=FLASH_read_BUF[4];
		printf("car_model:%u\r\n",car_model);
		}
	
	//STMFLASH_Read(FLASH_SAVE_ADDR+0,FLASH_read_BUF, 2);
	//if(FLASH_read_BUF[0]==FLASH_write_flag)//�����ٶ�2
		{
		//STMFLASH_Read(FLASH_SAVE_ADDR+4,FLASH_read_BUF, 2);
		//up_timer=FLASH_read_BUF[0];
		//printf("perimeter2:%u\r\n",up_timer);
		}


 	while(1)
	{
				
		if(TIM3_IRQHandler_count==0)
			{
			TIM3_IRQHandler_count=200;
			
			LED0=~LED0;
			//USART_SendData(USART2, 0x99);
			//while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
			
		#if 1
			KEY_VALUE=KEY_Scan(0);
			if(KEY_VALUE)
				{
				 printf("KEY_VALUE = %d\r\n",KEY_VALUE);
				 work_over_time=work_over_time_val;
				//REALY_EN=1;
				//back_mirror_L_UP();
				//back_mirror_R_UP(); 
				}
			if(back_mirror_on_off_flag==back_mirror_on) //�򿪹رա�
				{
				back_mirror_ctrl(KEY_VALUE);
				}
		#endif
			
			//Can_ISO15765_send_data_app(Can_iso15765_Receive_id,test_buf);
			
			#ifdef MCU_SLEEP_MODE	//�Զ����߹���
				if(work_over_time>0)
					work_over_time--;
				if((work_over_time==0)) //��ʱû�н��յ����ݽ������,
					{
					//Sys_Enter_Standby();
					//CAN_STB=1; // �ر�can �շ���
					LED0=1;
					BT_POWER_EN=0;// �ر�����ģ��.
					EXTIX_UART0_can_Init(); //���߳�ʼ��
					printf("PWR_EnterSTOPMode\r\n");
					PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);
					SystemInit();
					EXTIX_UART0_can_DISABLE();
					//CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);//��������CANbps=APB1����Ƶ��36000000/4/(1+8+9))=500k bps	 
					//uart1_init(921600);
					work_over_time=work_over_time_val;
					//CAN_STB=0; // ��can �շ���
					BT_POWER_EN=1;// ������ģ��.
					printf("wake_up\r\n");
					}
			#endif

			
		}
		
		#if 1
		//if(USART_RX_STA&0x8000)
		if((USART_RX_flag)&(TIM3_IRQHandler_UART_CNT==0))
		{					   
			#if 1
			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
			//printf("�����͵���ϢΪ:\r\n");
			for(t=0;t<len;t++)
			{
				//while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
				USART_SendData(USART1, USART_RX_BUF[t]);//�򴮿�1��������
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			}
			printf("\r\n");//���뻻��
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
			#if 0
			if(USART_RX_BUF[0]==0x66)
				{
				REALY_EN=1;
				//delay_ms(1);
				if(USART_RX_BUF[1]==0x01)
					{
					back_mirror_L_UP();
					delay_ms(1000); //���1864 MS
					//delay_ms(1000); //���1864 MS
					//delay_ms(1000); //���1864 MS
					back_mirror_L_stop();
					}
				else if(USART_RX_BUF[1]==0x02)
					{
					back_mirror_L_DOWM();
					delay_ms(1000); //���1864 MS
					//delay_ms(1000); //���1864 MS
					//delay_ms(1000); //���1864 MS
					back_mirror_L_stop();
					}
				else if(USART_RX_BUF[1]==0x00)
					{
					back_mirror_L_stop();
					}
				else if(USART_RX_BUF[1]==0x03)
					{
					back_mirror_L_LEFT();
					delay_ms(1000);
					back_mirror_L_stop();
					}
				else if(USART_RX_BUF[1]==0x04)
					{
					back_mirror_L_RIGHT();
					delay_ms(1000);
					back_mirror_L_stop();
					}
				else if(USART_RX_BUF[1]==0x05)
					{
					back_mirror_R_UP();
					delay_ms(1000); //���1864 MS
					//delay_ms(1000); //���1864 MS
					//delay_ms(1000); //���1864 MS
					back_mirror_R_stop();
					}
				else if(USART_RX_BUF[1]==0x06)
					{
					back_mirror_R_DOWM();
					delay_ms(1000); //���1864 MS
					//delay_ms(1000); //���1864 MS
					//delay_ms(1000); //���1864 MS
					back_mirror_R_stop();
					}
				back_mirror_LR_stop_ALL();
				}
			#endif

			#if 0
			if((USART_RX_BUF[0]==0xff)&(USART_RX_BUF[1]==0x66)) //���ͷ
				{
				if(data_check_sum(USART_RX_BUF)==1) //����ok.
					{
					//printf("data_check_sum_ok\r\n");
					if(USART_RX_BUF[2]==0x01)
						{
	 					motor_down_timer=(USART_RX_BUF[3]<<8)|USART_RX_BUF[4];
						FLASH_write_BUF[1]=motor_down_timer;//�������ʱ��
						printf("motor_down_timer:%u\r\n",motor_down_timer);
						}
					else if(USART_RX_BUF[2]==0x02)
						{
	 					motor_up_timer=(USART_RX_BUF[3]<<8)|USART_RX_BUF[4];
						FLASH_write_BUF[2]=motor_up_timer;//�������ʱ��						
						printf("motor_up_timer:%u\r\n",motor_up_timer);
						}
					else if(USART_RX_BUF[2]==0x03)
						{
						car_back_timer=(USART_RX_BUF[3]<<8)|USART_RX_BUF[4];
						FLASH_write_BUF[3]=car_back_timer;//������ǰʱ����						
						printf("car_back_timer:%u\r\n",car_back_timer);
						}
					else if(USART_RX_BUF[2]==0x04)
						{
						car_model=USART_RX_BUF[3];
						FLASH_write_BUF[4]=car_model;//����						
						printf("car_model:%u\r\n",car_model);
						}
					else if(USART_RX_BUF[2]==0x05)
						{
						back_mirror_on_off_flag=USART_RX_BUF[3];
						FLASH_write_BUF[5]=back_mirror_on_off_flag;//back work on_off
						printf("back_mirror_on_off_flag:%u\r\n",back_mirror_on_off_flag);
						}
					STMFLASH_Write(FLASH_SAVE_ADDR,FLASH_write_BUF,buffer_len);//д��4�ֽڡ���������
					}
				}
			#endif
			
			USART_RX_STA=0;
			USART_RX_flag=0;
		}


		if((USART2_RX_flag)&(TIM3_IRQHandler_UART2_CNT==0))
		{					   
			#if 1
			len=USART2_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
			//printf("�����͵���ϢΪ:\r\n");
			for(t=0;t<len;t++)
			{
				//while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
				USART_SendData(USART2, USART2_RX_BUF[t]);//�򴮿�1��������
				while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			}
			//USART_SendData(USART2, 0x0d);//�򴮿�1��������
			//while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			//USART_SendData(USART2, 0x0a);//�򴮿�1��������
			//while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			#endif

			if((USART2_RX_BUF[0]==0xff)&(USART2_RX_BUF[1]==0x66)) //���ͷ
				{
				if(data_check_sum(USART2_RX_BUF)==1) //����ok.
					{
					//printf("data_check_sum_ok2\r\n");
					if(USART2_RX_BUF[2]==0x01)
						{
	 					motor_down_timer=(USART2_RX_BUF[3]<<8)|USART2_RX_BUF[4];
						FLASH_write_BUF[1]=motor_down_timer;//�������ʱ��
						printf("motor_down_timer:%u\r\n",motor_down_timer);
						}
					else if(USART2_RX_BUF[2]==0x02)
						{
	 					motor_up_timer=(USART2_RX_BUF[3]<<8)|USART2_RX_BUF[4];
						FLASH_write_BUF[2]=motor_up_timer;//�������ʱ��
						printf("motor_up_timer:%u\r\n",motor_up_timer);
						}
					else if(USART2_RX_BUF[2]==0x03)
						{
						car_back_timer=(USART2_RX_BUF[3]<<8)|USART2_RX_BUF[4];
						FLASH_write_BUF[3]=car_back_timer;//������ǰʱ����
						printf("car_back_timer:%u\r\n",car_back_timer);
						}
					else if(USART2_RX_BUF[2]==0x04)
						{
						car_model=USART2_RX_BUF[3];
						FLASH_write_BUF[4]=car_model;//����
						printf("car_model:%u\r\n",car_model);
						}
					else if(USART2_RX_BUF[2]==0x05)
						{
						back_mirror_on_off_flag=USART2_RX_BUF[3];
						FLASH_write_BUF[5]=back_mirror_on_off_flag;//back work on_off
						printf("back_mirror_on_off_flag:%u\r\n",back_mirror_on_off_flag);
						}
					STMFLASH_Write(FLASH_SAVE_ADDR,FLASH_write_BUF,buffer_len);//д��4�ֽڡ���������
					}
				}
			
			USART2_RX_STA=0;
			USART2_RX_flag=0;
			
		}
		#endif
		
		 
	}	 
 }

