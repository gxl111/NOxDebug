/********************************************************************
filename : etk_delay.h
editor   : Icy
time     : 2018.02.05
contact  : edreamtek@163.com
********************************************************************/

#ifndef _ETK_DELAY_H__
#define _ETK_DELAY_H__


#include "stm32f10x.h"



void delay_10ms(__IO uint32_t nTime);

void delay_s(uint16_t nCount);

/*-------------------------------------------------------*/
//private function
void TimingDelay_Decrement(void);


#endif




