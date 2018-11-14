#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "stm32f10x_can.h"

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
 
//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024;  tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M,�������CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
//������Ϊ:36M/((8+9+1)*4)=500Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��; 

u8 acc_on_flag=0;
u8 Power_on_first=0;
u16 acc_off_flag=0;
u8 key_lock=0;
u8 door_lock=0;
u16 timer_cnt_can=10;
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{ 
	GPIO_InitTypeDef 		GPIO_InitStructure; 
	CAN_InitTypeDef        	CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure;
#if CAN_RX0_INT_ENABLE 
	NVIC_InitTypeDef  		NVIC_InitStructure;
#endif

	 /* ���ù��ܺ�GPIOB�˿�ʱ��ʹ��*/
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
 
	/* CAN1 ģ��ʱ��ʹ�� */
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//�������� GPIO_Mode_IPU
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO

	/* CAN register init */
 	CAN_DeInit(CAN1); //������CAN��ȫ���Ĵ�������Ϊȱʡֵ
 	CAN_StructInit(&CAN_InitStructure);//��CAN_InitStruct�е�ÿһ��������ȱʡֵ����

	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE;			//��ʱ�䴥��ͨ��ģʽ  
	CAN_InitStructure.CAN_ABOM=DISABLE;			//����Զ����߹���	 
	CAN_InitStructure.CAN_AWUM=DISABLE;			//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=ENABLE;			//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;		 	//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=DISABLE;			//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= mode;	        //ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; 
	//���ò�����
	CAN_InitStructure.CAN_SJW=tsjw;				//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;        //��Ƶϵ��(Fdiv)Ϊbrp+1	
	CAN_Init(CAN1, &CAN_InitStructure);        	//��ʼ��CAN1 

	CAN_FilterInitStructure.CAN_FilterNumber=13;	//������0--13
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList; 	//	list  ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit; 	//16λ�� 
	CAN_FilterInitStructure.CAN_FilterIdHigh=(can_CAN_Filter_id_1|CAN_ID_STD)<<5;	//32λID  ��׼�� һ�� 27λ������5λ�����λ���롣��չ45λ ���� 3λ��
	CAN_FilterInitStructure.CAN_FilterIdLow=(Can_iso15765_Receive_id|CAN_ID_STD)<<5;//32λID  ��׼�� һ�� 27λ������5λ�����λ���롣��չ45λ ���� 3λ��
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x00;//MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x00; // MASK
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0
	CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��
	
	
	CAN_FilterInitStructure.CAN_FilterNumber=0;	//������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList; 	//list ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit; 	//16 λ�� 
	CAN_FilterInitStructure.CAN_FilterIdHigh=(can_CAN_Filter_id_6|CAN_ID_STD)<<5;	//16λID
	CAN_FilterInitStructure.CAN_FilterIdLow=(can_CAN_Filter_id_7|CAN_ID_STD)<<5; //16λID
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;//MASK
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0
	CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��

	CAN_FilterInitStructure.CAN_FilterNumber=1;	//������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList; 	//list ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit; 	//16 λ�� 
	CAN_FilterInitStructure.CAN_FilterIdHigh=(can_CAN_Filter_id_8|CAN_ID_STD)<<5;	//16λID
	CAN_FilterInitStructure.CAN_FilterIdLow=(can_CAN_Filter_id_9|CAN_ID_STD)<<5; //16λID
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;//MASK
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0
	CAN_FilterInit(&CAN_FilterInitStructure);

	CAN_FilterInitStructure.CAN_FilterNumber=2;	//������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList; 	//list ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit; 	//16 λ�� 
	CAN_FilterInitStructure.CAN_FilterIdHigh=(can_CAN_Filter_id_10|CAN_ID_STD)<<5;	//16λID
	CAN_FilterInitStructure.CAN_FilterIdLow=(can_CAN_Filter_id_11|CAN_ID_STD)<<5; //16λID
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;//MASK
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0
	CAN_FilterInit(&CAN_FilterInitStructure);
/*
	CAN_FilterInitStructure.CAN_FilterNumber=2;	//������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList; 	//list ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 	//32λ�� 
	CAN_FilterInitStructure.CAN_FilterIdHigh=can_CAN_Filter_id_2<<5;	//32λID 
	CAN_FilterInitStructure.CAN_FilterIdLow=0|CAN_ID_STD;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0
	CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��
	*/
	
#if CAN_RX0_INT_ENABLE 
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0��Ϣ�Һ��ж�����.		    

	NVIC_InitStructure.NVIC_IRQChannel =USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
#if CAN_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	uint32_t CAN_ID_VALUE=0;
  	CanRxMsg RxMessage;
	//int i=0;
	timer_cnt_can=15;
	//printf("USB_LP_CAN1_RX0_IRQHandler\r\n");
    CAN_Receive(CAN1, 0, &RxMessage);
	if (RxMessage.IDE == CAN_Id_Standard)
		CAN_ID_VALUE=RxMessage.StdId;
	else
		CAN_ID_VALUE=RxMessage.ExtId;
	
	//printf("%3lx ",CAN_ID_VALUE);
	//for(i=0;i<RxMessage.DLC;i++)
		//printf(" %2x",RxMessage.Data[i]);
	//printf("\r\n");
	
	#ifdef Can_iso15765_debug
	Can_ISO15765_Decode(CAN_ID_VALUE,RxMessage.Data);  //���롣
	#endif

	#ifdef Can_iso15765_app
	Can_ISO15765_send_data_app(CAN_ID_VALUE,RxMessage.Data);  //app��
	#endif
	window_close_decode(CAN_ID_VALUE,RxMessage.Data); //����
	
}
#endif

//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
u8 Can_Send_Msg(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x07df;			// ��׼��ʶ�� 
	TxMessage.ExtId=0x18db33f1;			// ������չ��ʾ�� 
	TxMessage.IDE=CAN_Id_Standard; 	// ��׼֡
	TxMessage.RTR=CAN_RTR_Data;		// ����֡
	TxMessage.DLC=len;				// Ҫ���͵����ݳ���
	for(i=0;i<len;i++)
	TxMessage.Data[i]=msg[i];			          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	if(i>=0XFFF)return 1;
	return 0;	 
}

u8 Can_Send_Msg_id(u32 id,const u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=id;			// ��׼��ʶ�� 
	TxMessage.ExtId=id;			// ������չ��ʾ�� 
	TxMessage.IDE=CAN_Id_Standard; 	// ��׼֡
	TxMessage.RTR=CAN_RTR_Data;		// ����֡
	TxMessage.DLC=len;				// Ҫ���͵����ݳ���
	for(i=0;i<len;i++)
	TxMessage.Data[i]=msg[i];			          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	if(i>=0XFFF)return 1;
	return 0;	 
}

//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
u8 Can_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
    for(i=0;i<8;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}


u8 Can_ISO15765_Send(u8 PID,u32 can_id,u8 len)
{
	CanTxMsg TxMessage;
	u8 mbox;
	u16 i=0;
	u8 ISO15765_Send_buf[8]={0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
	ISO15765_Send_buf[2]=PID;
	TxMessage.StdId=can_id;			// ��׼��ʶ�� 
	TxMessage.ExtId=can_id;			// ������չ��ʾ�� 
	TxMessage.IDE=CAN_Id_Standard; 	// ��׼֡
	TxMessage.RTR=CAN_RTR_Data;		// ����֡
	TxMessage.DLC=len;				// Ҫ���͵����ݳ���
	for(i=0;i<len;i++)
	TxMessage.Data[i]=ISO15765_Send_buf[i];			          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	if(i>=0XFFF)return 1;
	return 0;
}


void Can_ISO15765_Decode(uint32_t can_id,u8 *buf)
{
	float temp=0;
	if(can_id==Can_iso15765_Receive_id)
		{
		if(buf[2]==Engine_RPM)
			{
			temp=(buf[3]<<8|buf[4]);
			temp=temp/4;
			//printf("Engine_RPM:%f\r\n",temp);
			printf("�����ת��:%f\r\n",temp);
			}
		else if(buf[2]==Velicele_speed)
			{
			//printf("Velicele_speed:%d\r\n",buf[3]);
			printf("����:%d����\r\n",buf[3]);
			}
		else if(buf[2]==Throttle_position)
			{
			temp=buf[3];
			temp=temp*100/255;
			//printf("Throttle_position:%f\r\n",temp);
			printf("����:%f%%\r\n",temp);
			}
		else if(buf[2]==Engine_load)
			{
			temp=buf[3];
			temp=temp*100/255;
			//printf("Throttle_position:%f%%\r\n",temp);
			printf("���������:%f%%\r\n",temp);
			}
		else if(buf[2]==Battery_Voltage)
			{
			temp=(buf[3]<<8)|buf[4];
			temp=temp/1000;
			//printf("Throttle_position:%f%%\r\n",temp);
			printf("��ص�ѹ:%fV\r\n",temp);
			}
		//printf("--------------------------------------\r\n");
		}
	
}



void door_close_test(void)
{
	
	//740,02 10 03 00 00 00 00 00
	//740,05 2F 61 20 03 01 00 00
	//740,05 2F 61 21 03 01 00 00
	//740,05 2F 61 22 03 01 00 00
	//740,05 2F 61 23 03 01 00 00
	const u8 data_buf1[8]={0x02, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
	const u8 data_buf2[8]={0x05, 0x2f, 0x61, 0x20, 0x03, 0x01, 0x00, 0x00};
	const u8 data_buf3[8]={0x05, 0x2f, 0x61, 0x21, 0x03, 0x01, 0x00, 0x00};
	const u8 data_buf4[8]={0x05, 0x2f, 0x61, 0x22, 0x03, 0x01, 0x00, 0x00};
	const u8 data_buf5[8]={0x05, 0x2f, 0x61, 0x23, 0x03, 0x01, 0x00, 0x00};
	Can_Send_Msg_id(0x740,data_buf1,8);
	delay_ms(60);
	Can_Send_Msg_id(0x740,data_buf2,8);
	delay_ms(60);
	Can_Send_Msg_id(0x740,data_buf3,8);
	delay_ms(60);
	Can_Send_Msg_id(0x740,data_buf4,8);
	delay_ms(60);
	Can_Send_Msg_id(0x740,data_buf5,8);
	delay_ms(60);
}

void door_open_test(void)
{
	
	//740,02 10 03 00 00 00 00 00
	//740,04 2F 61 20 00 00 00 00 
	//740,04 2F 61 21 00 00 00 00
	//740,04 2F 61 22 00 00 00 00
	//740,04 2F 61 23 00 00 00 00
	const u8 data_buf1[8]={0x02, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
	const u8 data_buf2[8]={0x04, 0x2f, 0x61, 0x20, 0x00, 0x00, 0x00, 0x00};
	const u8 data_buf3[8]={0x04, 0x2f, 0x61, 0x21, 0x00, 0x00, 0x00, 0x00};
	const u8 data_buf4[8]={0x04, 0x2f, 0x61, 0x22, 0x00, 0x00, 0x00, 0x00};
	const u8 data_buf5[8]={0x04, 0x2f, 0x61, 0x23, 0x00, 0x00, 0x00, 0x00};
	Can_Send_Msg_id(0x740,data_buf1,8);
	delay_ms(60);
	Can_Send_Msg_id(0x740,data_buf2,8);
	delay_ms(60);
	Can_Send_Msg_id(0x740,data_buf3,8);
	delay_ms(60);
	Can_Send_Msg_id(0x740,data_buf4,8);
	delay_ms(60);
	Can_Send_Msg_id(0x740,data_buf5,8);
	delay_ms(60);
}


void window_close_decode(uint32_t can_id,u8 *buf)
{
	if(can_id==can_CAN_Filter_id_6) // 0x391
		{
		//if(key_lock==1) //����Ѿ�Ϣ��
			{
			if((buf[0]==0x00)&(buf[1]==0x00)&(buf[2]==0x00)&(buf[3]==0x00)
				&(buf[4]==0x00)&(buf[5]==0x00)&(buf[6]==0x00)&(buf[7]==0x22))  // 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x01  ����
																													// 0x22
				{
				door_close_test(); //����
				acc_on_flag=0;
				door_lock=1;
				printf("door_close\r\n");
				}
			if((buf[0]==0x00)&(buf[1]==0x00)&(buf[2]==0x80)&(buf[3]==0x00)
				&(buf[4]==0x00)&(buf[5]==0x00)&(buf[6]==0x80)&(buf[7]==0xA2))//391    00 00 80 00 00 00 80 81 ����//
																									//  80 A2
				{
				door_open_test();
				//door_close_test(); //����
				printf("door_open\r\n");
				}
			}
		if((buf[0]==0x20)&(buf[1]==0x00)&(buf[2]==0x80)&(buf[3]==0x00)
			&(buf[4]==0x00)&(buf[5]==0x00)&(buf[6]==0x00)&(buf[7]==0x00))  // 0x20 0x00 0x80 0x00 0x00 0x00 0x00 0x00  ACC ON
			{
			acc_on_flag=1;
			door_lock=0;
			Power_on_first=1;
			}
		
		}
	if(can_id==can_CAN_Filter_id_9) // 0x0fa  û�н��մ�ID  ˵���Ѿ��ػ�
		{
		acc_off_flag=0;
		}
	
}

u8 data_generate_sum(u8 *uart_buf_dat)
{
	u8 key_i=0;
	u16 temp_check_data=0;
	u8 check_sum_data=0;

	
		for(key_i=0;key_i<13;key_i++) //13�ֽڣ���ͷ2 β1��16byte
			{
			temp_check_data=check_sum_data+uart_buf_dat[(2+key_i)]; //�� 3�������ݿ�ʼ��
			//printf("temp_check_data:%u\r\n",temp_check_data);
			check_sum_data=(u8)temp_check_data;
			if(temp_check_data>0xff) //����255 ��λ�� 1
				check_sum_data =check_sum_data+1;
			}
		//printf("check_sum_data:%u\r\n",check_sum_data);
		check_sum_data=(~check_sum_data)+1; //ȡ��+1;
		
	return check_sum_data;
}

u8 data_check_sum(u8 *uart_buf_dat)
{
	u8 key_i=0;
	u16 temp_check_data=0;
	u8 check_sum_data=0;

	
		for(key_i=0;key_i<13;key_i++) //13�ֽڣ���ͷ2 β1��16byte
			{
			temp_check_data=check_sum_data+uart_buf_dat[2+key_i]; //�� 3�������ݿ�ʼ��
			check_sum_data=(u8)temp_check_data;
			if(temp_check_data>0xff) //����255 ��λ�� 1
				check_sum_data =check_sum_data+1;
			}
		
		check_sum_data=~check_sum_data+1; //ȡ��+1;
		if(check_sum_data==uart_buf_dat[15]) // ���һ���ֽ��Ǽ���� 
			{
			return 1;
			}
		else
			{
			return 0;
			}
	}

void Can_ISO15765_send_data_app(uint32_t can_id,u8 *buf)
{
	u8 i=0;
	u8 t=0;
	u8 app_data_buf[16]={0xff,0x66,0x00,0x00,0x00,0x00,0x00,0x00,
						 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	if(can_id==Can_iso15765_Receive_id)
		{
		for(i=0;i<8;i++)
			{
			app_data_buf[2+i]=buf[i]; //���ݸ��Ƶ�app_data_buf
			}
		app_data_buf[15]=data_generate_sum(app_data_buf);//���һλ��У׼�ϡ�
		for(t=0;t<16;t++)
			{
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
				USART_SendData(USART1, app_data_buf[t]);//�򴮿�1��������
				//while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			}
		}
	
}

