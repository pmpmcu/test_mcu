#include "back_mirror.h"
#include "stm32f10x.h"
#include "key.h"
#include "sys.h" 
#include "delay.h"

u16 car_back_timer=10000;
u16 motor_down_timer=3000;
u16 motor_up_timer=3000;
u8 car_model=car_model_rui_hu_5;
u16 work_over_time=work_over_time_val;
u8 back_mirror_on_off_flag=back_mirror_on;

u16 TIM3_IRQHandler_car_back_timer; //3：自动调回间离时间10s
u16 TIM3_IRQHandler_motor_down_timer;// 0:ID  1:向下时间3s，
u16 TIM3_IRQHandler_motor_up_timer;//2：向上时间3s，
#define motor_l_h_time_us_val 10
void L_UD_SET(void)
{
	L_UD_L=0;
	delay_us(motor_l_h_time_us_val);
	L_UD_H=1;
}
void L_UD_CLEAN(void)
{
	L_UD_H=0;
	delay_us(motor_l_h_time_us_val);
	L_UD_L=1;
}
void L_LR_SET(void)
{
	L_LR_L=0;
	delay_us(motor_l_h_time_us_val);	
	L_LR_H=1;	
}
void L_LR_CLEAN(void)
{
	L_LR_H=0;
	delay_us(motor_l_h_time_us_val);	
	L_LR_L=1;
}

void L_COM_SET(void)
{
	L_COM_L=0;
	delay_us(motor_l_h_time_us_val);
	L_COM_H=1;
}
void L_COM_CLEAN(void)
{
	L_COM_H=0;
	delay_us(motor_l_h_time_us_val);
	L_COM_L=1;
}

void R_UD_SET(void)
{
	R_UD_L=0;
	delay_us(10);
	R_UD_H=1;
}
void R_UD_CLEAN(void)
{
	R_UD_H=0;
	delay_us(motor_l_h_time_us_val);
	R_UD_L=1;
}
void R_LR_SET(void)
{
	R_LR_L=0;
	delay_us(motor_l_h_time_us_val);	
	R_LR_H=1;
}
void R_LR_CLEAN(void)
{
	R_LR_H=0;
	delay_us(motor_l_h_time_us_val);	
	R_LR_L=1;
}

void R_COM_SET(void)
{
	R_COM_L=0;
	delay_us(motor_l_h_time_us_val);
	R_COM_H=1;
}
void R_COM_CLEAN(void)
{
	R_COM_H=0;
	delay_us(motor_l_h_time_us_val);
	R_COM_L=1;
}



void back_mirror_io_init(void)
{
 
 	GPIO_InitTypeDef  GPIO_InitStructure;
 	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);	 //使能PB,PE,PA端口时钟
 	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

 	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;				 //LED0-->PB.5 端口配置
 	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //输入
 	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 	//GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7
								|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_14|GPIO_Pin_15;	//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 //输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure); 				 //根据设定参数初始化GPIOB.5

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_15;	//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 //输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure); 	


	L_UD_L=0;
	R_LR_H=0;
	R_LR_L=0;
	L_LR_H=0;
	L_LR_L=0;
	REALY_EN=0;
	L_COM_L=0;
	L_COM_H=0;
	L_UD_H=0;
	BT_POWER_EN=0;
	R_COM_L=0;
	R_COM_H=0;
	R_UD_L=0;
	R_UD_H=0;
}

void back_mirror_L_UP(void) //左边上
{
	if(car_model==car_model_rui_hu_5)
		{
		//L_UD_L=0;
		//L_COM_H=0;
		//delay_us(10);
		//L_UD_H=1;
		//L_COM_L=1;
		L_UD_SET();
		L_COM_CLEAN();
		}

	
}
void back_mirror_L_DOWM(void)//左边下
{
	if(car_model==car_model_rui_hu_5)
		{
		//L_UD_H=0;
		//L_COM_L=0;
		//delay_us(10);
		//L_UD_L=1;
		//L_COM_H=1;
		L_UD_CLEAN();
		L_COM_SET();
		}
}

void back_mirror_L_LEFT(void)//左边左
{
	if(car_model==car_model_rui_hu_5)
		{
		//L_LR_L=0;
		//L_COM_H=0;
		//delay_us(10);
		//L_LR_H=1;
		//L_COM_L=1;
		L_LR_SET();
		L_COM_CLEAN();
		}
}

void back_mirror_L_RIGHT(void)//左边右
{
	if(car_model==car_model_rui_hu_5)
		{
//		L_LR_H=0;
//		L_COM_L=0;
//		delay_us(10);
//		L_LR_L=1;
//		L_COM_H=1;
		L_LR_CLEAN();
		L_COM_SET();
		}
}


void back_mirror_R_UP(void)//右边上，  共同一根用左边。
{
	if(car_model==car_model_rui_hu_5)
		{
//		R_UD_L=0;
//		L_COM_H=0;
//		delay_us(10);
//		R_UD_H=1;
//		L_COM_L=1;
		R_UD_SET();
		L_COM_CLEAN();
		}
}

void back_mirror_R_DOWM(void)//右边下,共同一根用左边。
{
	if(car_model==car_model_rui_hu_5)
		{
//		R_UD_H=0;
//		L_COM_L=0;
//		delay_us(10);
//		R_UD_L=1;
//		L_COM_H=1;
		R_UD_CLEAN();
		L_COM_SET();
		}
}

void back_mirror_R_LEFT(void)//右边左,共同一根用左边。
{
	if(car_model==car_model_rui_hu_5)
		{
//		R_LR_L=0;
//		L_COM_H=0;
//		delay_us(10);
//		R_LR_H=1;
//		L_COM_L=1;
		R_LR_SET();
		L_COM_CLEAN();
		}
}

void back_mirror_R_RIGHT(void)//右边右,共同一根用左边。
{
	if(car_model==car_model_rui_hu_5)
		{
//		R_LR_H=0;
//		L_COM_L=0;
//		delay_us(10);
//		R_LR_L=1;
//		L_COM_H=1;
		R_LR_CLEAN();
		L_COM_SET();
		}
}

void back_mirror_L_stop(void)//左边停
{
	if(car_model==car_model_rui_hu_5)
		{
		L_COM_L=0;
		L_COM_H=0;
		L_LR_H=0;
		L_LR_L=0;
		L_UD_H=0;
		L_UD_L=0;
		}
}
void back_mirror_R_stop(void)//右边停
{
	if(car_model==car_model_rui_hu_5)
		{
		L_COM_L=0;
		L_COM_H=0;
		R_LR_H=0;
		R_LR_L=0;
		R_UD_H=0;
		R_UD_L=0;
		}
}

void back_mirror_LR_stop_ALL(void)//所有停
{

	L_COM_L=0;
	L_COM_H=0;
	
	L_LR_H=0;
	L_LR_L=0;
	
	R_COM_L=0;
	R_COM_H=0;
	
	R_LR_H=0;
	R_LR_L=0;
	
	L_UD_L=0;
	L_UD_H=0;
	
	R_UD_L=0;
	R_UD_H=0;
	delay_ms(1);
	//REALY_EN=0;
}

void back_mirror_ctrl(u8 mode) // MODE  1 BACK
{
	static u8 work_mode=Normal_exercise;
	if(mode)
		{
		//printf("mode\r\n");
		if(work_mode==Normal_exercise)
			{
			printf("Normal_exercise\r\n");
			REALY_EN=1;
			delay_ms(10);
			back_mirror_L_DOWM();
			back_mirror_R_DOWM();
			//delay_ms(1500);
			//delay_ms(1500);
			TIM3_IRQHandler_motor_down_timer=motor_down_timer;
			while(TIM3_IRQHandler_motor_down_timer);
			back_mirror_L_stop();
			back_mirror_R_stop();
			work_mode=back_car;
			work_over_time=work_over_time_val;
			//delay_ms(10);
			}
		TIM3_IRQHandler_car_back_timer=car_back_timer; //10秒
		}
	if((mode==0)&(TIM3_IRQHandler_car_back_timer==0))
		{
 		if(work_mode==back_car)
			{
			printf("back_car\r\n");
			back_mirror_L_UP();
			back_mirror_R_UP();
			//delay_ms(1500);
			//delay_ms(1500);
			TIM3_IRQHandler_motor_up_timer=motor_up_timer;
			while(TIM3_IRQHandler_motor_up_timer);
			back_mirror_L_stop();
			back_mirror_R_stop();
			work_mode=Normal_exercise;
			delay_ms(10);
			REALY_EN=0;
			delay_ms(100);
			work_over_time=work_over_time_val;
			}
		}
}


u8 data_check_sum(u8 *uart_buf_dat)
{
	u8 key_i=0;
	u16 temp_check_data=0;
	u8 check_sum_data=0;

	
		for(key_i=0;key_i<data_crc_len;key_i++) //5字节，加头2 尾1，8byte
			{
			temp_check_data=check_sum_data+uart_buf_dat[2+key_i]; //第 3节是数据开始。
			check_sum_data=(u8)temp_check_data;
			if(temp_check_data>0xff) //大于255 低位加 1
				check_sum_data =check_sum_data+1;
			}
		
		check_sum_data=~check_sum_data+1; //取反+1;
		//printf("check_sum_data:%u\r\n",check_sum_data);
		if(check_sum_data==uart_buf_dat[data_crc_len+2]) // 最后一个字节是检验和 
			{
			return 1;
			}
		else
			{
			return 0;
			}
	}


u8 data_generate_sum(u8 *uart_buf_dat)
{
	u8 key_i=0;
	u16 temp_check_data=0;
	u8 check_sum_data=0;

	
		for(key_i=0;key_i<data_crc_len;key_i++) //6字节，加头2 尾1，8byte
			{
			temp_check_data=check_sum_data+uart_buf_dat[(2+key_i)]; //第 3节是数据开始。
			//printf("temp_check_data:%u\r\n",temp_check_data);
			check_sum_data=(u8)temp_check_data;
			if(temp_check_data>0xff) //大于255 低位加 1
				check_sum_data =check_sum_data+1;
			}
		//printf("check_sum_data:%u\r\n",check_sum_data);
		check_sum_data=~check_sum_data+1; //取反+1;
		
	return check_sum_data;
}

