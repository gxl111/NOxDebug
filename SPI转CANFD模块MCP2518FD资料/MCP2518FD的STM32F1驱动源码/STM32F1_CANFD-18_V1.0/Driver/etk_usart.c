/********************************************************************
filename : etk_usart.c
discript : 串口驱动，可用于调试信息输出
editor   : Icy
time     : 2018.02.14
contact  : edreamtek@163.com
********************************************************************/

#include "etk_usart.h"





/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
uint8_t RxBuffer[RxBufferSize];
__IO uint16_t RxCounter = 0x00; 
uint16_t NbrOfDataToRead = RxBufferSize;
uint8_t Rx_flag;




/********************************************************************
function: etk_usart_init(void)
discript: Init usart1.
return  : none
other   : none
********************************************************************/
void etk_usart_init(void)
{
	/*以下是配置USART1,包括对应的IO口时钟配置*/
	GPIO_InitTypeDef GPIO_InitStructure;			//声明GPIO初始化结构体
  USART_InitTypeDef USART_InitStructure; 		//声明USART初始化结构体

	/*GPIOA Periph clock enable*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  /*USART1 Periph clock enable*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Configure USART1 Tx (PA.9) as alternate function open-drain */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;					//配置TX所在的PA9管脚
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Configure USART1 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	USART_InitStructure.USART_BaudRate = BAUD;				//波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure USART1 */
  USART_Init(USART1, &USART_InitStructure);
	
	etk_NVIC_config();

	/* Enable USART1 Receive and Transmit interrupts */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);				//使能接收中断
		
	USART_ClearFlag(USART1, USART_FLAG_TC);
  
  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);														//使能USART1	
}

/********************************************************************
function: etk_NVIC_config(void)
discript: 串口中断配置
return  : none
other   : none
********************************************************************/
void etk_NVIC_config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  /* Enable the USART2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

	etk_usart_rx_enable();
}

/********************************************************************
function: etk_usart_init(void)
discript: Init usart1.
return  : none
other   : none
********************************************************************/
void etk_usart_rx_enable(void)
{
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	Rx_flag = 0;
	RxCounter = 0;
}

/********************************************************************
function: USART1_IRQHandler(void)
discript: 接收数据终端
return  : none
other   : none
********************************************************************/
void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    RxBuffer[RxCounter++] = USART_ReceiveData(USART1);
		Rx_flag = 0x1;
    if(RxCounter == NbrOfDataToRead)
    {
			RxCounter = 0;		//清空接收重新接收
    }
  }
}



/*Retargets the C library printf function to the USART.*/
int fputc(int ch, FILE *f)
{
  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

	/* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  return ch;
}


