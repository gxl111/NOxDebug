/********************************************************************
filename : etk_delay.c
discript : This file offer some delay function use systerm tick.
editor   : Icy
time     : 2017.12.14
contact  : edreamtek@163.com
********************************************************************/


#include "etk_delay.h"


static __IO uint32_t TimingDelay_basic;



void delay_s(uint16_t nCount)
{
	for(; nCount>0; nCount--)
		delay_10ms(100);
}


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void delay_10ms(__IO uint32_t nTime)
{
  TimingDelay_basic = nTime;

  while(TimingDelay_basic != 0);
}


/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay_basic != 0x00)
  { 
    TimingDelay_basic --;
  }
}

