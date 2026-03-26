/********************************************************************
filename : etk_can.c
discript : CAN端数据顶层处理
version  : V1.0
editor   : Icy - etk
time     : 2019.9.26
statement: All rights reserved.
contact  : edreamtek@163.com
           edreamtek.taobao.com
********************************************************************/

#include "etk_can.h"



//APP_DATA appData;

CAN_CONFIG config;
//CAN_OPERATION_MODE opMode;

// Transmit objects
CAN_TX_FIFO_CONFIG txConfig;
CAN_TX_FIFO_EVENT txFlags;
CAN_TX_QUEUE_CONFIG	txqConfig;
CAN_TEF_CONFIG tefConfig;
//CAN_TX_MSGOBJ txObj;
//uint8_t txd[MAX_DATA_BYTES];

// Receive objects
CAN_RX_FIFO_CONFIG rxConfig;
REG_CiFLTOBJ fObj;
REG_CiMASK mObj;
CAN_RX_FIFO_EVENT rxFlags;
//CAN_RX_MSGOBJ rxObj;
//uint8_t rxd[MAX_DATA_BYTES];

CAN_BUS_DIAGNOSTIC busDiagnostics;
uint8_t tec;
uint8_t rec;
CAN_ERROR_STATE errorFlags;
CAN_TX_FIFO_STATUS rx_status,tx_status;

uint8_t can_rx_flag;

/********************************************************************
function: etk_can_init
discript: CAN port初始化.
entrance: none.
return  : none
********************************************************************/
void etk_can_init(CAN_BITTIME_SETUP baud)
{
	etk_can_spi_port_init();
	etk_can_self_test();

	etk_canfd_cfg_init(baud);
}

/********************************************************************
function: etk_canfd_ch1_cfg_init
discript: CAN FD MCP2517FD初始化
entrance: none.
return  : none
********************************************************************/
void etk_canfd_cfg_init(CAN_BITTIME_SETUP baud)
{
	REG_CiFLTOBJ fObj;
	REG_CiMASK mObj;
	CAN_TX_FIFO_CONFIG txConfig;
	CAN_RX_FIFO_CONFIG rxConfig;
	CAN_CONFIG config;

	// Reset device
	DRV_CANFDSPI_Reset(CANFD_CH1);					//复位MCP2517FD
	// Enable ECC and initialize RAM
	DRV_CANFDSPI_EccEnable(CANFD_CH1);			//使能ECC

	DRV_CANFDSPI_RamInit(CANFD_CH1, 0xff);	//RAM初始化
	
	//0xE00-0xE03->0x60 0x40 0x00 0x00
	//使用外部40MHz时钟，1分频，系统时钟40MHz
	//DRV_CANFDSPI_WriteByte(CANFD_CH1,0xE00,0x20);		//CLKODIV 
		
	// Configure device
	DRV_CANFDSPI_ConfigureObjectReset(&config);				//MCP2517FD配置信息复位
	config.IsoCrcEnable = 1;
	config.StoreInTEF = 0;
	config.BitRateSwitchDisable = 0;		//Depends on the BRS bit on TX msg

	//CiCON->addr:0x00-03
	DRV_CANFDSPI_Configure(CANFD_CH1, &config);	//MCP2517FD配置

	// Setup TX FIFO
	DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
	txConfig.FifoSize = 11;
	txConfig.PayLoadSize = CAN_PLSIZE_64;
	txConfig.TxPriority = 1;

	//CiTXQCON->addr:0x50-53 + CAN_FIFO_CHn*12
	DRV_CANFDSPI_TransmitChannelConfigure(CANFD_CH1, CAN_TX_FIFO, &txConfig);

	// Setup RX FIFO
	DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxConfig);
	rxConfig.FifoSize = 15;
	rxConfig.PayLoadSize = CAN_PLSIZE_64;
	rxConfig.RxTimeStampEnable = 1;
	
	//CiFIFOCON1->addr:0x50-53 + CAN_FIFO_CHn*12
	DRV_CANFDSPI_ReceiveChannelConfigure(CANFD_CH1, CAN_RX_FIFO, &rxConfig);
	
	DRV_CANFDSPI_TimeStampModeConfigure(CANFD_CH1,CAN_TS_RES);
	DRV_CANFDSPI_TimeStampPrescalerSet(CANFD_CH1,40-1);	//40 clk 加1，40M晶体，单位1us
	DRV_CANFDSPI_TimeStampSet(CANFD_CH1,0);
	DRV_CANFDSPI_TimeStampEnable(CANFD_CH1);	//使能时间戳

	// Setup RX Filter
	// Setup RX Filter
	fObj.word = 0;
	fObj.bF.EXIDE = 0;
	fObj.bF.EID = 0;
	fObj.bF.SID = 0;
	
	//CiFLTCON0->0x1D0-0x1D3
	DRV_CANFDSPI_FilterObjectConfigure(CANFD_CH1, CAN_FILTER0, &fObj.bF);

	// Setup RX Mask
	mObj.word = 0;
	mObj.bF.MIDE = 0; 	//Only allow standard IDs		1 bit
	mObj.bF.MEID = 0;
	mObj.bF.MSID = 0;
	//CiMASK0
	DRV_CANFDSPI_FilterMaskConfigure(CANFD_CH1, CAN_FILTER0, &mObj.bF);

	// Link FIFO and Filter
	DRV_CANFDSPI_FilterToFifoLink(CANFD_CH1, CAN_FILTER0, CAN_RX_FIFO, true);

	// Setup Bit Time
	// CiNBTCFG->0x04-0x07
	DRV_CANFDSPI_BitTimeConfigure(CANFD_CH1, baud, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);

	// Setup Transmit and Receive Interrupts
	// IOCONN->0xE04-0xE07
	DRV_CANFDSPI_GpioModeConfigure(CANFD_CH1, GPIO_MODE_INT, GPIO_MODE_INT);
	//CiFOFICON0
	DRV_CANFDSPI_ReceiveChannelEventEnable(CANFD_CH1, CAN_RX_FIFO, CAN_RX_FIFO_NOT_EMPTY_EVENT);
	//CiINT->0x1C
	DRV_CANFDSPI_ModuleEventEnable(CANFD_CH1, CAN_RX_EVENT);

	// Select Normal Mode
	//CiCON->0x00-0x03
	DRV_CANFDSPI_OperationModeSelect(CANFD_CH1, CAN_NORMAL_MODE);

}



/********************************************************************
function: etk_can_self_test
discript: CAN FD自检
entrance: none.
return  : none
********************************************************************/
void etk_can_self_test(void)
{
	bool passed = etk_can1_test_ram_access();
	if (passed) 
	{
//		passed = etk_can_test_register_access();
//		if (passed) 
//		{
			__NOP();
//		}
	}
}


/********************************************************************
function: etk_can_transmit_check_fifo
discript: 检查CAN发送FIFO是否已满
entrance: none.
return  : 0，未满，-1已满
********************************************************************/
int8_t etk_can_transmit_check_fifo(CANFDSPI_MODULE_ID index)
{
	DRV_CANFDSPI_TransmitChannelEventGet(index, CAN_TX_FIFO, &txFlags);
	if((txFlags & CAN_TX_FIFO_NOT_FULL_EVENT))
		return 0;
	else
		return -1;
}

/********************************************************************
function: etk_can_transmit_msg
discript: CAN发送数据.
entrance: none.
return  : none
********************************************************************/
int8_t etk_can_transmit_msg(CANFDSPI_MODULE_ID index,CANFD_TX_MSG *tx_msg)
{
	// Load message and transmit
	uint8_t n = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)(tx_msg->head.bF.ctrl.DLC));

//	return DRV_CANFDSPI_TransmitChannelLoad_Direct(index, CAN_TX_FIFO, tx_msg->byte, n);
	return DRV_CANFDSPI_TransmitChannelLoad(index, CAN_TX_FIFO, &tx_msg->head, tx_msg->dat, n, true);
}

/********************************************************************
function: etk_read_can_bus_err_state
discript: 获取总线状态或错误
entrance: none.
return  : none
********************************************************************/
void etk_read_can_bus_err_state(CANFDSPI_MODULE_ID index)
{
	DRV_CANFDSPI_ErrorCountStateGet(index, &tec, &rec, &errorFlags);
//	printf("errflags can1:%x,%d,%d ",errorFlags,tec,rec);

//	DRV_CANFDSPI_ErrorCountStateGet(CANFD_CH2, &tec, &rec, &errorFlags);
//	printf("can2:%x,%d,%d\r\n",errorFlags,tec,rec);
		
	DRV_CANFDSPI_BusDiagnosticsGet(index,&busDiagnostics);
//	printf("busDiag can1:%x,%x ",busDiagnostics.word[0],busDiagnostics.word[1]);

//	DRV_CANFDSPI_BusDiagnosticsGet(CANFD_CH2,&busDiagnostics);
//	printf("can2:%x,%x\r\n",busDiagnostics.word[0],busDiagnostics.word[1]);
	
	DRV_CANFDSPI_TransmitChannelStatusGet(index,CAN_TX_FIFO,&tx_status);
//	printf("txqueStatus can1:%x ",tx_status);
	DRV_CANFDSPI_TransmitChannelStatusGet(index,CAN_RX_FIFO,&rx_status);
//	printf("%x ",tx_status);

//	DRV_CANFDSPI_TransmitChannelStatusGet(CANFD_CH2,CAN_TX_FIFO,&tx_status);
//	printf("can2:%x ",tx_status);
//	DRV_CANFDSPI_TransmitChannelStatusGet(CANFD_CH2,CAN_RX_FIFO,&rx_status);
//	printf("%x\r\n\r\n",tx_status);
}


//void CAN_TransmitMessageQueue(CANFDSPI_MODULE_ID index)
//{
//	uint8_t attempts = MAX_TXQUEUE_ATTEMPTS;

//	// Check if FIFO is not full
//	do {
//			DRV_CANFDSPI_TransmitChannelEventGet(index, CAN_TX_FIFO, &txFlags);
//			if (attempts == 0) 
//			{
//				DRV_CANFDSPI_ErrorCountStateGet(index, &tec, &rec, &errorFlags);
//				return;
//			}
//			attempts--;
//	}
//	while (!(txFlags & CAN_TX_FIFO_NOT_FULL_EVENT));

//	// Load message and transmit
//	uint8_t n = DRV_CANFDSPI_DlcToDataBytes(txObj.bF.ctrl.DLC);

//	DRV_CANFDSPI_TransmitChannelLoad(index, CAN_TX_FIFO, &txObj, txd, n, true);
//}


/********************************************************************
function: etk_can_test_register_access
discript: MCP2517FD register access test.
entrance: none.
return  : none
********************************************************************/
bool etk_can1_test_register_access(void)
{
	// Variables
	uint8_t length,i;
	bool good = false;
	uint8_t txd[MAX_DATA_BYTES],rxd[MAX_DATA_BYTES];

	// Verify read/write with different access length
	// Note: registers can be accessed in multiples of bytes
	for (length = 1; length <= MAX_DATA_BYTES; length++) 
	{
		for (i = 0; i < length; i++) 
		{
				txd[i] = rand() & 0x7f; // Bit 31 of Filter objects is not implemented
				rxd[i] = 0xff;
		}

		// Write data to registers
		DRV_CANFDSPI_WriteByteArray(CANFD_CH1, cREGADDR_CiFLTOBJ, txd, length);

		// Read data back from registers
		DRV_CANFDSPI_ReadByteArray(CANFD_CH1, cREGADDR_CiFLTOBJ, rxd, length);

		// Verify
		good = false;
		for (i = 0; i < length; i++) 
		{
			good = txd[i] == rxd[i];

			if (!good) 
			{
				// Data mismatch
				return false;
			}
		}
	}

	return true;
}

/********************************************************************
function: etk_can_test_ram_access
discript: MCP2517FD RAM access test.
entrance: none.
return  : none
********************************************************************/
bool etk_can1_test_ram_access(void)
{
	// Variables
	uint8_t length,i;
	bool good = false;
	uint8_t txd[MAX_DATA_BYTES],rxd[MAX_DATA_BYTES];

	// Verify read/write with different access length
	// Note: RAM can only be accessed in multiples of 4 bytes
	for (length = 4; length <= MAX_DATA_BYTES; length += 4) 
	{
		for (i = 0; i < length; i++) 
		{
			txd[i] = rand() & 0xff;
			rxd[i] = 0xff;
		}

		// Write data to RAM
		DRV_CANFDSPI_WriteByteArray(CANFD_CH1, cRAMADDR_START, txd, length);

		// Read data back from RAM
		DRV_CANFDSPI_ReadByteArray(CANFD_CH1, cRAMADDR_START, rxd, length);

		// Verify
		good = false;
		for (i = 0; i < length; i++) 
		{
			good = txd[i] == rxd[i];

			if (!good) 
			{
				// Data mismatch
				return false;
			}
		}
	}

	return true;
}

