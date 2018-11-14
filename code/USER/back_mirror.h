#ifndef __BACK_MIRROR_H
#define __BACK_MIRROR_H
#include "sys.h"
#include "usart.h"

#define L_UD_L PBout(2)// PB2
#define R_LR_H PBout(3)// PB3
#define R_LR_L PBout(4)// PB4
#define L_LR_H PBout(5)// PB5
#define L_LR_L PBout(6)// PB6
#define REALY_EN PBout(7)// PB7
#define L_COM_L PBout(8)// PB8
#define L_COM_H PBout(9)// PB9
#define L_UD_H PBout(10)// PB10
#define BT_POWER_EN PBout(11)// PB11
#define R_COM_H PBout(14)// PB9
#define R_COM_L PBout(15)// PB9
#define R_UD_L PAout(8)// PA8
#define R_UD_H PAout(15)// PA8


#define Normal_exercise 1
#define back_car 2
#define car_model_rui_hu_5 1
#define back_mirror_on 1
#define buffer_len 6
#define work_over_time_val 6000
#define data_crc_len (8-3)  // 减去 2个字节头 1个尾
#define MCU_SLEEP_MODE 1
void back_mirror_io_init(void);

void back_mirror_L_UP(void);
void back_mirror_L_DOWM(void);
void back_mirror_L_LEFT(void);
void back_mirror_L_RIGHT(void);

void back_mirror_R_UP(void);
void back_mirror_R_DOWM(void);
void back_mirror_R_LEFT(void);
void back_mirror_R_RIGHT(void);

void back_mirror_L_stop(void);
void back_mirror_R_stop(void);
void back_mirror_LR_stop_ALL(void);

void back_mirror_ctrl(u8 mode);
u8 data_check_sum(u8 *uart_buf_dat);
u8 data_generate_sum(u8 *uart_buf_dat);


extern u16 TIM3_IRQHandler_car_back_timer;
extern u16 TIM3_IRQHandler_motor_down_timer;
extern u16 TIM3_IRQHandler_motor_up_timer;
extern u16 car_back_timer;
extern u16 motor_down_timer;
extern u16 motor_up_timer;
extern u8 car_model;
extern u16 work_over_time;
extern u8 back_mirror_on_off_flag;

#endif

