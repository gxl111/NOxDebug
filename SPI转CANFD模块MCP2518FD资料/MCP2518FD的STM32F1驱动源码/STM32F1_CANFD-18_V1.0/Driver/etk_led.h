/********************************************************************
filename : etk_led.h
discript : LEDÇý¶¯
editor   : Icy
time     : 2018.06.9
contact  : edreamtek@163.com
********************************************************************/

#ifndef _ETK_LED_H__
#define _ETK_LED_H__

#include "stm32f10x.h"  
#include "etk_typedefine.h"


/*On blard LED pinmap*/

#define LEDn				3

#define	LED1				0
#define LED1_PIN                         GPIO_Pin_3
#define LED1_GPIO_PORT                   GPIOA
#define LED1_GPIO_CLK                    RCC_APB2Periph_GPIOA

#define	LED2				1
#define LED2_PIN                         GPIO_Pin_4
#define LED2_GPIO_PORT                   GPIOA
#define LED2_GPIO_CLK                    RCC_APB2Periph_GPIOA

#define	LED3				2
#define LED3_PIN                         GPIO_Pin_5
#define LED3_GPIO_PORT                   GPIOA
#define LED3_GPIO_CLK                    RCC_APB2Periph_GPIOA


extern uint8_t	led1_timeout,led2_timeout,led3_timeout;



//LED pin function init.
void board_led_init(void);

//single LED control function.
void board_led_ctrl(uint8_t led,uint8_t value);

//single LEDs toggle function.
void board_led_toggle(uint8_t led);


#endif

