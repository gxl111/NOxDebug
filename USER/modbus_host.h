#ifndef __MODBUS_HOST_H
#define __MODBUS_HOST_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "modbus.h"
#include "timers.h"


#define H_RX_BUF_SIZE		64
#define H_TX_BUF_SIZE      	128




#define REG_P01		0x000A
#define REG_P02		0x000B
#define REG_P03		0x0032
#define REG_P04		0x0033
#define REG_P05		0x003D
#define REG_PXX 	REG_P05

typedef struct
{
	uint8_t RxBuf[H_RX_BUF_SIZE];
	uint8_t RxCount;
	uint8_t RxStatus;
	uint8_t RxNewFlag;

	uint8_t RspCode;

	uint8_t TxBuf[H_TX_BUF_SIZE];
	uint8_t TxCount;
	
	uint16_t Reg01H;		/* Host-sent register start address */
	uint16_t Reg02H;
	uint16_t Reg03H;		
	uint16_t Reg04H;

	uint8_t RegNum;			/* Number of registers */

	uint8_t fAck03H;
	uint8_t fAck06H;		
	uint8_t fAck10H;
	
}MODH_T;
typedef struct
{

	uint16_t P01;
	uint16_t P02;
	uint16_t P03;
	uint16_t P04;
    uint16_t P05;
	
}VAR_T_H;

extern VAR_T_H g_tVar_h;
//????????????
extern uint8_t electricity_data_buf[4];

extern SemaphoreHandle_t RS485send_SemaphoreHandle;
extern QueueHandle_t MODHx_SemaphoreHandle;
extern TimerHandle_t MODH_Timer;


extern void MODH_Send03H(uint8_t _addr, uint16_t _reg, uint16_t _num);

extern void Start_Receive_H(void);

extern void MODH_Poll(void);

extern void (*s_TIM_CallBack2)(void);

extern void MODH_ReciveNew(uint8_t _data);

extern void MODH_Send06H(uint8_t _addr, uint16_t _reg, uint16_t _value);

/* RS485 host: send buffer via UART5 with direction pin (used by MODH_SendWithCRC) */
extern void RS485_Send_Data_IT_H(uint8_t *pData, uint16_t Size);

#endif
