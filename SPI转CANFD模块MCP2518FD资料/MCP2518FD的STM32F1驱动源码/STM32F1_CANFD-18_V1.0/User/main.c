/********************************************************************
filename : main.c
editor   : etk
discript : STM32F1 CANFD EVM driver
time     : 2020.10.14
contact  : edreamtek@163.com
********************************************************************/

#include "main.h"


void etk_watch_dog_init(void);
void etk_can_msg_transmit(void);
int8_t etk_canfd_rcv_poll(void);


extern uint8_t tick;

uint16_t i=0,j;

CANFD_RX_MSG can_rx_msg;
CANFD_TX_MSG can_tx_msg;


int main(void)
{
  RCC_ClocksTypeDef RCC_Clocks;

	/* Extern clock frequency is 8Mhz */
	/* Systerm clock is 72MHz */
	/* HCLK(AHB) is 72MHz */
	/* PCLK1(APB1) is 36MHz */
	/* PCLK2(APB2) is 72MHz */
	/* ADCCLK is 36MHz */
	
  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

	board_led_init();
	board_key_init();
	etk_usart_init();		//调试串口初始化，波特率9600
	
	etk_can_init(CAN_500K_5M);		//CANFD初始化
																//其他波特率参考“CAN_BITTIME_SETUP”参数，如果该参数中没有则需要查阅MCP2518FD数据手册自行配置

	board_led_ctrl(LED1,ON);
	board_led_ctrl(LED2,ON);
	board_led_ctrl(LED3,ON);
	
	printf("STM32F1 CANFD-18 EVM board init OK.\r\n");

	delay_10ms(50);
	board_led_ctrl(LED1,OFF);
	board_led_ctrl(LED2,OFF);
	board_led_ctrl(LED3,OFF);
	
		
	
	can_tx_msg.head.word[0] = 0;
	can_tx_msg.head.word[1] = 0;
	if(1) //扩展帧初始化帧头
	{
			uint32_t id = (0x12345678)>>18 & 0x7FF;
			id |= (0x12345678 & 0x3FFFF)<<11;
			can_tx_msg.head.word[0] = id;
	}
	else	//标准帧初始化帧头
	{
			can_tx_msg.head.bF.id.SID = 0x123;
	}
	can_tx_msg.head.bF.ctrl.DLC = CAN_DLC_64;	//长度参考“CAN_DLC”定义，不可以填定义里没有的参数
	can_tx_msg.head.bF.ctrl.IDE = 1;	// Extended CAN ID false
	can_tx_msg.head.bF.ctrl.RTR = 0;	// Remote frame
	can_tx_msg.head.bF.ctrl.BRS = TRUE;	//切换速率，也就是数据域启动高速率发送/接收
	can_tx_msg.head.bF.ctrl.FDF = TRUE;	// CAN FD
	for(j=0;j<64;j++)
	{
		can_tx_msg.dat[j] = j;
	}

	etk_can_msg_transmit();

	etk_watch_dog_init();			//看门狗初始化
	
	while(1)
	{
		/* Update IWDG counter */
    IWDG_ReloadCounter();
		
		if(key_pushed)		//如果有按键按下
		{
			key_pushed = 0;	//先清除标志
			if(key_down[BUTTON_USER1])
			{
				key_down[BUTTON_USER1] = 0;
				board_led_ctrl(LED2,ON);
				etk_can_msg_transmit();
				printf("CAN channel 1 send one message.\r\n");
			}
			else if(key_up[BUTTON_USER1])
			{
				key_up[BUTTON_USER1] = 0;
				board_led_ctrl(LED2,OFF);
			}
			else if(key_down[BUTTON_USER2])
			{
				key_down[BUTTON_USER2] = 0;
				printf("Key2 pushed.\r\n");
			}
			else if(key_up[BUTTON_USER2])
			{
				key_up[BUTTON_USER2] = 0;
			}
			else if(key_down[BUTTON_USER3])
			{
				key_down[BUTTON_USER3] = 0;
				printf("Key3 pushed.\r\n");
			}
			else if(key_up[BUTTON_USER3])
			{
				key_up[BUTTON_USER3] = 0;
			}
		}
		if(Rx_flag)	//串口收到数据标志
		{
			RxBuffer[RxCounter] = 0;	//添加字符串结束符
			printf("%s",RxBuffer);
			Rx_flag = 0;		//必须清零
			RxCounter = 0;	//必须清零
		}
		
		etk_canfd_rcv_poll();	//查询CANFD接收
		
		if(can_rx_flag)		//如果收到数据，则将该帧数据发送回去
		{
			can_rx_flag = FALSE;
			
			can_tx_msg.head.bF.id.EID = can_rx_msg.head.bF.id.EID;
			can_tx_msg.head.bF.id.SID11 = can_rx_msg.head.bF.id.SID11;
			can_tx_msg.head.bF.id.SID = can_rx_msg.head.bF.id.SID;
			can_tx_msg.head.bF.ctrl.DLC = can_rx_msg.head.bF.ctrl.DLC;
			can_tx_msg.head.bF.ctrl.BRS = can_rx_msg.head.bF.ctrl.BRS;
			can_tx_msg.head.bF.ctrl.FDF = can_rx_msg.head.bF.ctrl.FDF;
			can_tx_msg.head.bF.ctrl.IDE = can_rx_msg.head.bF.ctrl.IDE;
			can_tx_msg.head.bF.ctrl.RTR = can_rx_msg.head.bF.ctrl.RTR;
			for(j=0;j<64;j++)
			{
				can_tx_msg.dat[j] = can_rx_msg.dat[j];
			}
			etk_can_msg_transmit();
			printf("CAN Received one message and send back.\r\n");
		}
		if(tick ==1)
		{
			tick = 0;
			board_key_get_value();		//按键扫描
			
			i++;
			if(i%50 ==0)
			{
				i = 0;
				board_led_toggle(LED1);		//运行指示灯
			}
		}		
	}
}


/********************************************************************
function: etk_can_rcv_poll
discript: CAN接收数据查询.
entrance: none.
return  : 读取的CAN消息数量
********************************************************************/
int8_t etk_canfd_rcv_poll(void)
{
	int8_t pin,res =0;
	CAN_RX_FIFO_EVENT 	rxFlags;
	uint8_t type,len;

	pin= CAN1_RX_INT();
	if(pin) 
	{
		do{
			len = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)(type&0x0F));
			len +=12;		
			DRV_CANFDSPI_ReceiveMessageGet(CANFD_CH1, CAN_RX_FIFO, &can_rx_msg.head, can_rx_msg.dat, MAX_DATA_BYTES);
		
			res = DRV_CANFDSPI_ReceiveChannelEventGet(CANFD_CH1,CAN_RX_FIFO,&rxFlags);
			if(res<0)
			{		
				res = 0;				
				break;
			}
		}while(rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT);
			
		can_rx_flag = TRUE;
	}
	return TRUE;
}


void etk_can_msg_transmit(void)
{
	if(0==etk_can_transmit_check_fifo(CANFD_CH1))
	{
		if(etk_can_transmit_msg(CANFD_CH1,&can_tx_msg) ==CAN_TRANS_SUCCESS)
		{
			//发送成功
		}
		else
		{
			//发送失败
		}
	}
}

#define IWDG_PER             IWDG_Prescaler_256 
#define IWDG_CNT             625 // 4s timeout
/********************************************************************
function: etk_watch_dog_init
discript: 看门狗初始化
entrance: 无
return  : 无
********************************************************************/
void etk_watch_dog_init(void)
{
	// First Open LSI Clock for IWDG
  RCC_LSICmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
 
  // If use IWDG, LSI will be opened force
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  // Set Prescaler
  IWDG_SetPrescaler(IWDG_PER);
  // Set reload value
  IWDG_SetReload(IWDG_CNT);
  // Set 0xAAAA To make sure not go into reset
  IWDG_ReloadCounter();
  // Enable WDG
  IWDG_Enable();
}


