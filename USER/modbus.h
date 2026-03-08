#ifndef __MODBUS_H
#define __MODBUS_H

#include "main.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

/* Slave: USART1, TIM2 for RX timeout */
#define RX_TIMER htim2
#define MDSUARTx  huart1
#define MODBUSUART   USART1
#define RS485_EN_PORT RS485_RE_GPIO_Port
#define RS485_EN_PIN RS485_RE_Pin

/* Host: UART5, TIM3 for RX timeout */
#define RX_TIMER_H htim3
#define MDSUARTxH  huart5
#define MODBUSUARTH   UART5
#define RS485_EN_PORT_H RS485_RE_GPIO_Port
#define RS485_EN_PIN_H RS485_RE_Pin

/* RS485 direction state */
typedef struct {
    uint8_t isSending;
} RS485_State_t;

typedef struct
{
    uint32_t Bps;
    uint32_t usTimeOut;
} MODBUSBPS_T;



extern const MODBUSBPS_T ModbusBaudRate[];
extern const int MODBUS_BAUD_RATE_LEN;

extern const uint8_t s_CRCHi[];
extern const uint8_t s_CRCLo[];


extern uint8_t rx_data;
extern uint8_t rx_data_h;
extern RS485_State_t rs485_state;
extern RS485_State_t rs485_state_h;

extern uint16_t BEBufToUint16(uint8_t *_pBuf);
extern uint16_t CRC16_Modbus(uint8_t *_pBuf, uint16_t _usLen);
extern void RS485_Enable_TX(GPIO_TypeDef* port ,uint16_t pin) ;
extern void RS485_Enable_RX(GPIO_TypeDef* port ,uint16_t pin);
#endif
