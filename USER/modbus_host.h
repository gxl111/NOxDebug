#ifndef __MODBUS_HOST_H
#define __MODBUS_HOST_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "modbus.h"
#include "timers.h"
#include "app_config.h"


#define H_RX_BUF_SIZE		64
#define H_TX_BUF_SIZE      	128
#define REG_P01		0x000A
#define REG_P02		0x000B
#define REG_P06     0x000F
#define REG_P03		0x0032
#define REG_P04		0x0033
#define REG_P05		0x003D
#define REG_PXX 	REG_P05
#define AO_MODULE_ADDR_REG  REG_P03

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
/* 4-20 mA module register values (NOx/O2 codes), filled by NOx task */
extern uint8_t electricity_data_buf[4];
/* 6-channel 4-20 mA module values: S1 NOx/O2, S2 NOx/O2, S3 NOx/O2. */
extern uint8_t electricity_data_buf_6ch[12];

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
