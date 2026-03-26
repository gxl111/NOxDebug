/********************************************************************
filename : thl_data_struct_define.h
discript : 仪器数据结构体定义
editor   : Icy
time     : 2018.02.05
contact  : edreamtek@163.com
********************************************************************/


#ifndef _etk_USART_H__
#define _etk_USART_H__

#include <stdio.h>
#include "stm32f10x.h"


#define BAUD					9600		

#define RxBufferSize   500


extern uint8_t RxBuffer[RxBufferSize];
extern __IO uint16_t RxCounter; 
extern uint16_t NbrOfDataToRead;
extern uint8_t Rx_flag;



void etk_usart_init(void);

void etk_NVIC_config(void);

void  etk_usart_rx_enable(void);



#endif

