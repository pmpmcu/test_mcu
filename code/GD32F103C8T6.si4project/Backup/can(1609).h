#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//CAN���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/7
//�汾��V1.1 
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved	
//********************************************************************************
//V1.1�޸�˵�� 20150528
//������CAN��ʼ�����������ע�ͣ������˲����ʼ��㹫ʽ
////////////////////////////////////////////////////////////////////////////////// 	 
 
//CAN����RX0�ж�ʹ��
#define CAN_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.								    
										 							 				    
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
 
u8 Can_Send_Msg(u8* msg,u8 len);						//��������

u8 Can_Receive_Msg(u8 *buf);							//��������
#endif
u8 Can_ISO15765_Send(u8 PID,u32 can_id,u8 len);
void Can_ISO15765_Decode(uint32_t can_id,u8 *buf);


#define can_CAN_Filter_id_0 0x0442 
#define can_CAN_Filter_id_1 0x044D  
#define can_CAN_Filter_id_2 0x0012 
#define can_CAN_Filter_id_3 0x0088 
#define can_CAN_Filter_id_4 0x0631 
#define can_CAN_Filter_id_5 0x0631 

#define Engine_load 0x04
#define Engine_RPM 0x0C
#define Velicele_speed 0x0D
#define Throttle_position 0x11
#define Battery_Voltage 0x42
#define Can_iso15765_Send_id 0x07df
#define Can_iso15765_Receive_id 0x07e8















