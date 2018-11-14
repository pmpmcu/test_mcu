#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//CAN驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/7
//版本：V1.1 
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved	
//********************************************************************************
//V1.1修改说明 20150528
//修正了CAN初始化函数的相关注释，更正了波特率计算公式
////////////////////////////////////////////////////////////////////////////////// 	 
 
//CAN接收RX0中断使能
#define CAN_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    
										 							 				    
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
 
u8 Can_Send_Msg(u8* msg,u8 len);						//发送数据

u8 Can_Receive_Msg(u8 *buf);							//接收数据
#endif
u8 Can_ISO15765_Send(u8 PID,u32 can_id,u8 len);
void Can_ISO15765_Decode(uint32_t can_id,u8 *buf);
void Can_ISO15765_send_data_app(uint32_t can_id,u8 *buf);

void door_close_test(void);
void door_open_test(void);
void window_close_decode(uint32_t can_id,u8 *buf);
u8 Can_Send_Msg_id(u32 id,const u8* msg,u8 len);
u8 data_generate_sum(u8 *uart_buf_dat);
u8 data_check_sum(u8 *uart_buf_dat);



extern u8 acc_on_flag;
extern u16 acc_off_flag;
extern u8 key_lock;
extern u16 timer_cnt_can;
extern u8 door_lock;
extern u8 Power_on_first;


#define Engine_load 0x04
#define Engine_RPM 0x0C
#define Velicele_speed 0x0D
#define Throttle_position 0x11
#define Battery_Voltage 0x42
#define Can_iso15765_Send_id 0x07df
#define Can_iso15765_Send_id_mode1 0x07e0

#define Can_iso15765_Receive_id 0x07e8


#define can_CAN_Filter_id_0 Can_iso15765_Receive_id 
#define can_CAN_Filter_id_1 0x07e9  
#define can_CAN_Filter_id_2 0x0001 
#define can_CAN_Filter_id_3 0x0002 
#define can_CAN_Filter_id_4 0x0631 
#define can_CAN_Filter_id_5 0x0631 

#define can_CAN_Filter_id_6 0x391
#define can_CAN_Filter_id_7 0x500
#define can_CAN_Filter_id_8 0x580
#define can_CAN_Filter_id_9 0x0fa
#define can_CAN_Filter_id_10 0x101
#define can_CAN_Filter_id_11 0x2c1


//#define  Can_iso15765_EN 
//#define  Can_ID_MODE0 
#define  Can_ID_MODE1 
#define MCU_SLEEP_MODE
//#define Can_iso15765_app
//#define Can_iso15765_debug











