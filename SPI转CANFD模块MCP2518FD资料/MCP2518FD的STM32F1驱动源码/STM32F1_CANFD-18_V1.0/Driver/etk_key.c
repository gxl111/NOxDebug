/********************************************************************
filename : etk_key.c
discript : 按键驱动
editor   : Icy
time     : 2018.06.9
contact  : edreamtek@163.com
********************************************************************/

#include "etk_key.h"

/* key driver module public variable */
uint8_t __IO key_pushed;						//if user key is pushed down and after key up this flag will be TRUE
uint8_t __IO key_down[BUTTONn];			//the pushed down key's flag will be TRUE
uint8_t __IO key_up[BUTTONn];				//the pushed up key's flag will be TRUE
uint8_t __IO key_value[BUTTONn];    //store the current key's value


/* key driver module private variable */
uint8_t __IO key_old_value[BUTTONn];    //store the old key's value befor read new


GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {USER_BUTTON1_GPIO_PORT,USER_BUTTON2_GPIO_PORT,USER_BUTTON3_GPIO_PORT}; 

const uint16_t BUTTON_PIN[BUTTONn] = {USER_BUTTON1_PIN,USER_BUTTON2_PIN,USER_BUTTON3_PIN}; 

const uint32_t BUTTON_CLK[BUTTONn] = {USER_BUTTON1_GPIO_CLK,USER_BUTTON2_GPIO_CLK,USER_BUTTON3_GPIO_CLK};



/* key driver module private function */
void board_key_gpio_init(Button_TypeDef Button);

void board_key_get_gpio_state(void);




/********************************************************************
function: board_key_init(void)
discript: 按键功能初始化
return  : none
other   : none
********************************************************************/
void board_key_init(void)
{
	uint32_t i;
	key_pushed = 0x0;
	
	for(i=0;i<BUTTONn;i++)
	{
		key_down[i] = 0;			//the pushed down key's flag will be TRUE
		key_up[i] = 0;				//the pushed up key's flag will be TRUE
		key_value[i] = 1;    	//store the current key's value
	}
	/* init GPIO pin connected to key */
	board_key_gpio_init(BUTTON_USER1);
	board_key_gpio_init(BUTTON_USER2);
	board_key_gpio_init(BUTTON_USER3);
	
}


/********************************************************************
function: board_key_get_value()
discript: 读取独立按键键值
return  : none
other   : none
********************************************************************/
void board_key_get_value(void)
{
	uint32_t i,temp;
	Button_TypeDef Button;
	
	for(i=0;i<BUTTONn;i++)
	{
		Button = (Button_TypeDef)i;
		key_old_value[Button] = key_value[Button];
	}
	board_key_get_gpio_state(); 

	for(i=0;i<BUTTONn;i++)
	{
		Button = (Button_TypeDef)i;
		temp = key_old_value[Button] ^ key_value[Button];
		if(temp)
		{
			key_pushed = TRUE;
			if(key_value[Button])
			{
				key_up[Button] = TRUE;
			}
			else
			{
				key_down[Button] = TRUE;
			}
		}
	}
}

/********************************************************************
function: board_key_gpio_init()
discript: 独立按键端口初始化
return  : none
other   : none
********************************************************************/
void board_key_gpio_init(Button_TypeDef Button)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable the BUTTON Clock */
  RCC_APB2PeriphClockCmd(BUTTON_CLK[Button], ENABLE);

  /* Configure Button pin as input */
  GPIO_InitStructure.GPIO_Pin = BUTTON_PIN[Button];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//GPIO工作在输入上拉
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStructure);
}



/********************************************************************
function: board_key_get_gpio_state()
discript: 读取独立按键及矩阵按键端口状态
return  : none
other   : none
********************************************************************/
void board_key_get_gpio_state(void)
{
	uint8_t i;
	
	for(i=0;i<BUTTONn;i++)
	{
	  key_value[i] = GPIO_ReadInputDataBit(BUTTON_PORT[i], BUTTON_PIN[i]);
	}
}



