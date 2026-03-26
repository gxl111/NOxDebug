/********************************************************************
filename : etk_led.c
discript : LED驱动
editor   : Icy
time     : 2018.06.9
contact  : edreamtek@163.com
********************************************************************/


#include "etk_led.h"


GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT, LED3_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN, LED3_PIN};
const uint32_t GPIO_CLK[LEDn] = {LED1_GPIO_CLK, LED2_GPIO_CLK, LED3_GPIO_CLK};

uint8_t	led1_timeout,led2_timeout,led3_timeout;


/********************************************************************
function: board_led_init()
discript: LED功能初始化
return  : none
other   : none
********************************************************************/
void board_led_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	uint8_t i;
  
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(GPIO_CLK[0], ENABLE);			//Enable GPIO clock

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//GPIO工作在输出推挽模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	for(i=0; i< LEDn; i++)
	{
		GPIO_InitStructure.GPIO_Pin = GPIO_PIN[i];
		GPIO_Init(GPIO_PORT[i], &GPIO_InitStructure);		//Config GPIO
	}
}

/********************************************************************
function: board_led_ctrl()
discript: LED控制输出
return  : none
other   : none
********************************************************************/
void board_led_ctrl(uint8_t led,uint8_t value)
{
	if(value)		//Value is TRUE,light up LED.
	{
		  GPIO_PORT[led]->BRR = GPIO_PIN[led];		//低电平
	}
	else				//Value is FALSE,light down LED.
	{
		  GPIO_PORT[led]->BSRR = GPIO_PIN[led];		//高电平
	}
}

/********************************************************************
function: board_led_toggle()
discript: LED状态反转
return  : none
other   : none
********************************************************************/
void board_led_toggle(uint8_t led)
{
	  GPIO_PORT[led]->ODR ^= GPIO_PIN[led];
}



