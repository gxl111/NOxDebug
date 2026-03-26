/********************************************************************
filename : etk_key.h
discript : °´¼üÇý¶¯
editor   : Icy
time     : 2018.06.9
contact  : edreamtek@163.com
********************************************************************/

#ifndef __ETK_KEY_H__
#define __ETK_KEY_H__


#include "stm32f10x.h"
#include "etk_typedefine.h"



typedef enum 
{  
  BUTTON_USER1 = 0,
  BUTTON_USER2 = 1,
  BUTTON_USER3 = 2,
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;




#define BUTTONn                          3 /*!< Joystick pins are connected to 
                                                an IO Expander (accessible through 
                                                I2C1 interface) */

/**
 * @brief user push-button
 */
#define USER_BUTTON1_PIN                GPIO_Pin_0
#define USER_BUTTON1_GPIO_PORT          GPIOA
#define USER_BUTTON1_GPIO_CLK           RCC_APB2Periph_GPIOA

#define USER_BUTTON2_PIN                GPIO_Pin_1
#define USER_BUTTON2_GPIO_PORT          GPIOA
#define USER_BUTTON2_GPIO_CLK           RCC_APB2Periph_GPIOA

#define USER_BUTTON3_PIN                GPIO_Pin_2
#define USER_BUTTON3_GPIO_PORT          GPIOA
#define USER_BUTTON3_GPIO_CLK           RCC_APB2Periph_GPIOA




/* key driver module public variable */
/* after key_pushed,key_down,key_up is signed,
   they must cleared by user manually */
extern __IO uint8_t key_pushed;							//if user key is pushed down and after key up this flag will be TRUE
extern __IO uint8_t key_down[BUTTONn];			//the pushed down key's flag will be TRUE
extern __IO uint8_t key_up[BUTTONn];				//the pushed up key's flag will be TRUE
extern __IO uint8_t key_value[BUTTONn];     //store the current key's value




/* key driver module public function */
void board_key_init(void);				//before use key,we should init related function

void board_key_start_scan(void);	//start timer and check key value

void board_key_stop_scan(void);		//stop timer and stop check key value

/* protected function,normally should not be called by user */
void board_key_get_value(void);		//check key value

void board_key_interrupt(void);

void board_key_to_table(void);


/* Read KEY ROW1 pin */
#define KEY_ROW1()       			GPIO_ReadInputDataBit(USER_KEY_ROW1_GPIO_PORT, USER_KEY_ROW1_PIN)

/* Read KEY ROW2 pin */
#define KEY_ROW2()       			GPIO_ReadInputDataBit(USER_KEY_ROW2_GPIO_PORT, USER_KEY_ROW2_PIN)

/* Read KEY ROW3 pin */
#define KEY_ROW3()       			GPIO_ReadInputDataBit(USER_KEY_ROW3_GPIO_PORT, USER_KEY_ROW3_PIN)

/* Read KEY ROW1 pin */
#define KEY_ROW4()       			GPIO_ReadInputDataBit(USER_KEY_ROW4_GPIO_PORT, USER_KEY_ROW4_PIN)


/* KEY COL1 pin low  */
#define KEY_COL1_LOW()       		GPIO_ResetBits(USER_KEY_COL1_GPIO_PORT, USER_KEY_COL1_PIN)
/* KEY COL1 pin high  */
#define KEY_COL1_HIGH()      		GPIO_SetBits(USER_KEY_COL1_GPIO_PORT, USER_KEY_COL1_PIN)

/* KEY COL2 pin low  */
#define KEY_COL2_LOW()       		GPIO_ResetBits(USER_KEY_COL2_GPIO_PORT, USER_KEY_COL2_PIN)
/* KEY COL2 pin high  */
#define KEY_COL2_HIGH()      		GPIO_SetBits(USER_KEY_COL2_GPIO_PORT, USER_KEY_COL2_PIN)

/* KEY COL3 pin low  */
#define KEY_COL3_LOW()       		GPIO_ResetBits(USER_KEY_COL3_GPIO_PORT, USER_KEY_COL3_PIN)
/* KEY COL3 pin high  */
#define KEY_COL3_HIGH()      		GPIO_SetBits(USER_KEY_COL3_GPIO_PORT, USER_KEY_COL3_PIN)

/* KEY COL4 pin low  */
#define KEY_COL4_LOW()       		GPIO_ResetBits(USER_KEY_COL4_GPIO_PORT, USER_KEY_COL4_PIN)
/* KEY COL4 pin high  */
#define KEY_COL4_HIGH()      		GPIO_SetBits(USER_KEY_COL4_GPIO_PORT, USER_KEY_COL4_PIN)

/* KEY COL5 pin low  */
#define KEY_COL5_LOW()       		GPIO_ResetBits(USER_KEY_COL5_GPIO_PORT, USER_KEY_COL5_PIN)
/* KEY COL5 pin high  */
#define KEY_COL5_HIGH()      		GPIO_SetBits(USER_KEY_COL5_GPIO_PORT, USER_KEY_COL5_PIN)

/* KEY COL6 pin low  */
#define KEY_COL6_LOW()       		GPIO_ResetBits(USER_KEY_COL6_GPIO_PORT, USER_KEY_COL6_PIN)
/* KEY COL6 pin high  */
#define KEY_COL6_HIGH()      		GPIO_SetBits(USER_KEY_COL6_GPIO_PORT, USER_KEY_COL6_PIN)




#endif

