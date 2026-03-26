/********************************************************************
filename : etk_gpio.h
editor   : Icy
time     : 2018.02.05
contact  : edreamtek@163.com
********************************************************************/

#ifndef _ETK_GPIO_H__
#define _ETK_GPIO_H__

#include "stm32f10x.h"  


// ‰»ÎIO∂®“Â
#define PORT_IN1       			GPIO_Pin_10 
#define PORT_IN2            GPIO_Pin_2
#define PORT_IN3            GPIO_Pin_1
#define PORT_IN4            GPIO_Pin_0
#define PORT_IN5            GPIO_Pin_7
#define PORT_IN6            GPIO_Pin_6
#define PORT_IN7            GPIO_Pin_5
#define PORT_IN8            GPIO_Pin_4


#define PORT_L_OUT1         GPIO_Pin_6
#define PORT_L_OUT2         GPIO_Pin_5
#define PORT_L_OUT3         GPIO_Pin_4
#define PORT_L_OUT4         GPIO_Pin_3

#define PORT_W_OUT1         GPIO_Pin_15
#define PORT_W_OUT2         GPIO_Pin_14
#define PORT_W_OUT3         GPIO_Pin_13
#define PORT_W_OUT4         GPIO_Pin_12

#define USER_KEY            GPIO_Pin_2

#define USER_LED            GPIO_Pin_7


#define P_IN1								GPIO_ReadInputDataBit(GPIOB, PORT_IN1)
#define P_IN2								GPIO_ReadInputDataBit(GPIOB, PORT_IN2)
#define P_IN3								GPIO_ReadInputDataBit(GPIOB, PORT_IN3)
#define P_IN4								GPIO_ReadInputDataBit(GPIOB, PORT_IN4)
#define P_IN5 							GPIO_ReadInputDataBit(GPIOA, PORT_IN5)
#define P_IN6								GPIO_ReadInputDataBit(GPIOA, PORT_IN6)
#define P_IN7								GPIO_ReadInputDataBit(GPIOA, PORT_IN7)
#define P_IN8								GPIO_ReadInputDataBit(GPIOA, PORT_IN8)


#define I_USER_KEY					GPIO_ReadInputDataBit(GPIOA, USER_KEY)


#define P_L_OUT1_ON					GPIO_ResetBits(GPIOB,PORT_L_OUT1)
#define P_L_OUT1_OFF				GPIO_SetBits(GPIOB,PORT_L_OUT1)

#define P_L_OUT2_ON					GPIO_ResetBits(GPIOB,PORT_L_OUT2)
#define P_L_OUT2_OFF				GPIO_SetBits(GPIOB,PORT_L_OUT2)

#define P_L_OUT3_ON					GPIO_ResetBits(GPIOB,PORT_L_OUT3)
#define P_L_OUT3_OFF				GPIO_SetBits(GPIOB,PORT_L_OUT3)

#define P_L_OUT4_ON					GPIO_ResetBits(GPIOB,PORT_L_OUT4)
#define P_L_OUT4_OFF				GPIO_SetBits(GPIOB,PORT_L_OUT4)

#define P_W_OUT1_ON					GPIO_ResetBits(GPIOB,PORT_W_OUT1)
#define P_W_OUT1_OFF				GPIO_SetBits(GPIOB,PORT_W_OUT1)

#define P_W_OUT2_ON					GPIO_ResetBits(GPIOB,PORT_W_OUT2)
#define P_W_OUT2_OFF				GPIO_SetBits(GPIOB,PORT_W_OUT2)

#define P_W_OUT3_ON					GPIO_ResetBits(GPIOB,PORT_W_OUT3)
#define P_W_OUT3_OFF				GPIO_SetBits(GPIOB,PORT_W_OUT3)

#define P_W_OUT4_ON					GPIO_ResetBits(GPIOB,PORT_W_OUT4)
#define P_W_OUT4_OFF				GPIO_SetBits(GPIOB,PORT_W_OUT4)

#define O_USER_LED_ON				GPIO_ResetBits(GPIOB,USER_LED)
#define O_USER_LED_OFF			GPIO_SetBits(GPIOB,USER_LED)



//GPIO read into flag
extern uint8_t i_in1,i_in2,i_in3,i_in4,i_in5,i_in6,i_in7,i_in8;

extern uint8_t i_user_key;

//GPIO logic out flag
extern uint8_t o_l_out1,o_l_out2,o_l_out3,o_l_out4;
extern uint8_t o_w_out1,o_w_out2,o_w_out3,o_w_out4;

extern uint8_t o_user_led;



void etk_gpio_config(void);


void etk_gpio_read_in(void);


void etk_gpio_write_out(void);


#endif 
