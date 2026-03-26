/********************************************************************
filename : etk_can.h
discript : CAN端数据顶层处理
version  : V1.0
editor   : Icy - etk
time     : 2019.9.26
statement: All rights reserved.
contact  : edreamtek@163.com
           edreamtek.taobao.com
********************************************************************/

#ifndef __ETK_CAN_H__
#define __ETK_CAN_H__

#include "stm32f10x.h"
#include "drv_canfdspi_api.h"
#include "etk_spi_mcp2517fd.h"
#include "etk_led.h"
#include "etk_typedefine.h"

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************


//! Use RX and TX Interrupt pins to check FIFO status
#define CAN_USE_RX_INT

#define MAX_TXQUEUE_ATTEMPTS 1

#define CAN_TRANS_SUCCESS	0
#define CAN_TRANS_FAIL		-1


#define CAN1_ALL_INT()	(!CAN1_INT())	
#define CAN1_TX_INT()		(!CAN1_TX())
#define CAN1_RX_INT()		(!CAN1_RX())



typedef struct{
	uint8_t	cmd[2];
	CAN_TX_MSGOBJ head;
	uint8_t				dat[64];
}CAN_TX_MSG;

//! CAN TX Message Object
typedef union _CAN_TX_FRAME {
	CAN_TX_MSG msg;
	uint8_t byte[74];
} CAN_TX_FRAME;


typedef struct{
	CAN_TX_MSGOBJ head;
	uint8_t				dat[8];
}CAN20_TX_MSG;

typedef struct{
	CAN_TX_MSGOBJ head;
	uint8_t				dat[64];
}CANFD_TX_MSG;


typedef struct{
	CAN_RX_MSGOBJ head;
	uint8_t				dat[64];
}CAN_RX_MSG;

typedef struct{
	CAN_RX_MSGOBJ head;
	uint8_t				dat[8];
}CAN20_RX_MSG;

typedef struct{
	CAN_RX_MSGOBJ head;
	uint8_t				dat[64];
}CANFD_RX_MSG;


// Transmit Channels
#define CAN_TX_FIFO CAN_FIFO_CH2

// Receive Channels
#define CAN_RX_FIFO CAN_FIFO_CH1



extern CAN_BUS_DIAGNOSTIC busDiagnostics;
extern uint8_t tec;
extern uint8_t rec;
extern CAN_ERROR_STATE errorFlags;
extern CAN_TX_FIFO_STATUS tx_status;
extern uint8_t can_rx_flag;




//Init CANFD
void etk_can_init(CAN_BITTIME_SETUP);

//Cfg MCP2517FD
void etk_canfd_cfg_init(CAN_BITTIME_SETUP baud);

//Self test
void etk_can_self_test(void);

//检查发送FIFO是否已满
int8_t etk_can_transmit_check_fifo(CANFDSPI_MODULE_ID index);

//CAN send msg
int8_t etk_can_transmit_msg(CANFDSPI_MODULE_ID index,CANFD_TX_MSG *tx_msg);

//Add message to transmit FIFO
void CAN_TransmitMessageQueue(CANFDSPI_MODULE_ID index);

//获取总线状态或错误
void etk_read_can_bus_err_state(CANFDSPI_MODULE_ID index);


//Test SPI access
bool etk_can1_test_register_access(void);

//Test RAM access
bool etk_can1_test_ram_access(void);




#endif

