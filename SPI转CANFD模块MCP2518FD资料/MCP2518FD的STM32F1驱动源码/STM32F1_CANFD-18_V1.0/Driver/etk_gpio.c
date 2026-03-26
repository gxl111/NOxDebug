/********************************************************************
filename : etk_gpio.c
discript : 开关量输入输出引脚初始化
editor   : Icy
time     : 2018.02.14
contact  : talon56@163.com
********************************************************************/

#include "etk_gpio.h"


//GPIO read into flag
uint8_t i_in1,i_in2,i_in3,i_in4,i_in5,i_in6,i_in7,i_in8;

uint8_t i_user_key;

//GPIO logic out flag
uint8_t o_l_out1,o_l_out2,o_l_out3,o_l_out4;
uint8_t o_w_out1,o_w_out2,o_w_out3,o_w_out4;

uint8_t o_user_led;


/********************************************************************
function: etk_gpio_config(void)
discript: 输入输出端口初始化
return  : none
other   : none
********************************************************************/
void etk_gpio_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;     												//定义一个结构体变量GPIO_InitStructure，用于初始化GPIO操作

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 					//使能GPIO时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 					//使能GPIO时钟
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB , ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);

	/* 配置输入IO管脚模式，输入模式，上拉*/
  GPIO_InitStructure.GPIO_Pin = PORT_IN1 | PORT_IN2 | PORT_IN3 | PORT_IN4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//GPIO工作在输入上拉
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  						//相关的GPIO口初始化

  GPIO_InitStructure.GPIO_Pin = PORT_IN5 | PORT_IN6 | PORT_IN7 | PORT_IN8 | USER_KEY;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  						//相关的GPIO口初始化

	
	/* 配置输出IO管脚模式*/
  GPIO_InitStructure.GPIO_Pin = PORT_W_OUT1 | PORT_W_OUT2 | PORT_W_OUT3 | PORT_W_OUT4 |
																PORT_L_OUT1 | PORT_L_OUT2 | PORT_L_OUT3 | PORT_L_OUT4 | USER_LED;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//GPIO工作在输出推挽模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);  						//相关的GPIO口初始化

	GPIO_SetBits(GPIOB,PORT_W_OUT1 | PORT_W_OUT2 | PORT_W_OUT3 | PORT_W_OUT4);    		//拉高对应GPIO口，端口输出低
	GPIO_SetBits(GPIOB,PORT_L_OUT1 | PORT_L_OUT2 | PORT_L_OUT3 | PORT_L_OUT4);    		//拉高对应GPIO口，端口输出低
}



/********************************************************************
function: etk_gpio_read_in(void)
discript: 读取全部输入数据
return  : none
other   : none
********************************************************************/
void etk_gpio_read_in(void)
{
	i_in1 = !P_IN1;													//1 on,		0 off
	i_in2 = !P_IN2;													//1 on,		0 off
	i_in3 = !P_IN3;													//1 on,		0 off
	i_in4 = !P_IN4;													//1 on,		0 off
	i_in5 = !P_IN5;													//1 on,		0 off
	i_in6 = !P_IN6;													//1 on,		0 off
	i_in7 = !P_IN7;													//1 on,		0 off
	i_in8 = !P_IN8;													//1 on,		0 off
	
	i_user_key = !I_USER_KEY;
}

/********************************************************************
function: etk_gpio_write_out(void)
discript: 更新输出端口
return  : none
other   : none
********************************************************************/
void etk_gpio_write_out(void)
{
	o_l_out1 ? P_L_OUT1_ON : P_L_OUT1_OFF;
	o_l_out2 ? P_L_OUT2_ON : P_L_OUT2_OFF;
	o_l_out3 ? P_L_OUT3_ON : P_L_OUT3_OFF;
	o_l_out4 ? P_L_OUT4_ON : P_L_OUT4_OFF;

	o_w_out1 ? P_W_OUT1_ON : P_W_OUT1_OFF;
	o_w_out2 ? P_W_OUT2_ON : P_W_OUT2_OFF;
	o_w_out3 ? P_W_OUT3_ON : P_W_OUT3_OFF;
	o_w_out4 ? P_W_OUT4_ON : P_W_OUT4_OFF;

	o_user_led ? O_USER_LED_ON : O_USER_LED_OFF;
}


