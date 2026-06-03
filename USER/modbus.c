/**
 * @file    modbus.c
 * @brief   Modbus RTU common: CRC16 tables, BEBufToUint16, RS485 direction control, UART TX/RX callbacks.
 *          Baud rate table for 3.5 character timeout; shared by slave (USART1) and host (UART5).
 */
#include "modbus.h"
#include "modbus_slave.h"
#include "modbus_host.h"

/* CRC-16 (Modbus) high byte lookup table */
const uint8_t s_CRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;
/* CRC-16 (Modbus) low byte lookup table */
const uint8_t s_CRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};



/*
Baud rate	Bit rate	 Bit time	 Character time	  3.5 character times
  2400	    2400 bits/s	  417 us	      4.6 ms	      16 ms
  4800	    4800 bits/s	  208 us	      2.3 ms	      8.0 ms
  9600	    9600 bits/s	  104 us	      1.2 ms	      4.0 ms
 19200	   19200 bits/s    52 us	      573 us	      2.0 ms
 38400	   38400 bits/s	   26 us	      286 us	      1.75 ms(1.0 ms)
 115200	   115200 bit/s	  8.7 us	       95 us	      1.75 ms(0.33 ms); above 38400 fixed at 1750 us
*/

const MODBUSBPS_T ModbusBaudRate[] =
{
    {2400,	16000}, /* 2400 bps, 3.5 char time = 16000 us */
    {4800,	 8000},
    {9600,	 4000},
    {19200,	 2000},
    {38400,	 1750},
    {115200, 1750},
    {128000, 1750},
    {230400, 1750},
};
const int MODBUS_BAUD_RATE_LEN =(sizeof(ModbusBaudRate)/sizeof(ModbusBaudRate[0])) ;

/* Slave RX byte buffer */
uint8_t rx_data;
/* Host RX byte buffer */
uint8_t rx_data_h;


RS485_State_t rs485_state = {0};
RS485_State_t rs485_state_h={0};

#define MODBUS_TX_WATCHDOG_MS  100U

volatile uint32_t g_modbus_uart1_error_count = 0;
volatile uint32_t g_modbus_uart1_last_error = 0;
volatile uint32_t g_modbus_uart1_last_sr = 0;
volatile uint32_t g_modbus_uart1_rx_byte_count = 0;
volatile uint32_t g_modbus_uart1_tx_cplt_count = 0;
volatile uint32_t g_modbus_uart1_tx_watchdog_count = 0;
volatile uint32_t g_modbus_uart5_error_count = 0;
volatile uint32_t g_modbus_uart5_last_error = 0;
volatile uint32_t g_modbus_uart5_last_sr = 0;
volatile uint32_t g_modbus_uart5_rx_byte_count = 0;
volatile uint32_t g_modbus_uart5_tx_cplt_count = 0;
volatile uint32_t g_modbus_uart5_tx_watchdog_count = 0;

/*
 * CRC16_Modbus - Compute Modbus RTU CRC-16.
 * _pBuf : data buffer, _usLen : length.
 * Return: 16-bit CRC in Modbus wire order (high byte first on bus).
 * Uses precomputed hi/lo tables for speed vs per-byte polynomial step.
 * Note: returned value is already in byte order for appending to frame.
 */
uint16_t CRC16_Modbus(uint8_t *_pBuf, uint16_t _usLen)
{
    uint8_t ucCRCHi = 0xFF; /* CRC high byte init */
    uint8_t ucCRCLo = 0xFF; /* CRC low byte init */
    uint16_t usIndex;       /* table index */

    while (_usLen--)
    {
        usIndex = ucCRCHi ^ *_pBuf++; /* update CRC */
        ucCRCHi = ucCRCLo ^ s_CRCHi[usIndex];
        ucCRCLo = s_CRCLo[usIndex];
    }
    return ((uint16_t)ucCRCHi << 8 | ucCRCLo);
}

/*
 * BEBufToUint16 - Parse 2 bytes big-endian to uint16_t.
 * _pBuf[0] = high byte, _pBuf[1] = low byte.
 */
uint16_t BEBufToUint16(uint8_t *_pBuf)
{
    return (((uint16_t)_pBuf[0] << 8) | _pBuf[1]);
}

/*
 * RS485 TX/RX direction helpers; IT send switches DE pin automatically via TxCplt callback.
 */



void RS485_Enable_TX(GPIO_TypeDef* port ,uint16_t pin) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}


void RS485_Enable_RX(GPIO_TypeDef* port ,uint16_t pin) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    /* Slave USART */
    if (huart->Instance == MODBUSUART) {
        RS485_Enable_RX(RS485_EN_PORT,RS485_EN_PIN);
        rs485_state.isSending = 0;
        rs485_state.txStartTick = 0;
        g_modbus_uart1_tx_cplt_count++;
        (void)HAL_UART_Receive_IT(&MDSUARTx,&rx_data, 1);
    }
    /* Host UART */
    if (huart->Instance == MODBUSUARTH) {
        RS485_Enable_RX(RS485_EN_PORT_H,RS485_EN_PIN_H);
        rs485_state_h.isSending = 0;
        rs485_state_h.txStartTick = 0;
        g_modbus_uart5_tx_cplt_count++;
    }
    
}


/* Modbus RX byte callback: one byte per IRQ, re-arm IT receive */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    /* Slave */
    if (huart->Instance == MODBUSUART) {
        g_modbus_uart1_rx_byte_count++;
        if (!rs485_state.isSending) {
            MODS_ReciveNew(rx_data);
        }
        HAL_UART_Receive_IT(&MDSUARTx,&rx_data, 1);
    }
    /* Host */
    if (huart->Instance == MODBUSUARTH) {
        g_modbus_uart5_rx_byte_count++;
        MODH_ReciveNew(rx_data_h);
        HAL_UART_Receive_IT(&MDSUARTxH,&rx_data_h, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    uint32_t error_code = huart->ErrorCode;
    uint32_t sr = huart->Instance->SR;

    if (error_code == HAL_UART_ERROR_NONE) {
        if ((sr & UART_FLAG_PE) != 0U)  error_code |= HAL_UART_ERROR_PE;
        if ((sr & UART_FLAG_NE) != 0U)  error_code |= HAL_UART_ERROR_NE;
        if ((sr & UART_FLAG_FE) != 0U)  error_code |= HAL_UART_ERROR_FE;
        if ((sr & UART_FLAG_ORE) != 0U) error_code |= HAL_UART_ERROR_ORE;
    }

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

    if (huart->Instance == MODBUSUART) {
        uint8_t was_sending = (rs485_state.isSending != 0U);
        g_modbus_uart1_error_count++;
        g_modbus_uart1_last_error = error_code;
        g_modbus_uart1_last_sr = sr;
        if (!was_sending) {
            rs485_state.isSending = 0;
            rs485_state.txStartTick = 0;
            RS485_Enable_RX(RS485_EN_PORT, RS485_EN_PIN);
        }
        (void)HAL_UART_Receive_IT(&MDSUARTx, &rx_data, 1);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
    }

    if (huart->Instance == MODBUSUARTH) {
        uint8_t was_sending = (rs485_state_h.isSending != 0U);
        g_modbus_uart5_error_count++;
        g_modbus_uart5_last_error = error_code;
        g_modbus_uart5_last_sr = sr;
        if (!was_sending) {
            rs485_state_h.isSending = 0;
            rs485_state_h.txStartTick = 0;
            RS485_Enable_RX(RS485_EN_PORT_H, RS485_EN_PIN_H);
        }
        (void)HAL_UART_Receive_IT(&MDSUARTxH, &rx_data_h, 1);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
    }
}

void Modbus_ServiceTxWatchdog(void)
{
    uint32_t now = HAL_GetTick();

    if (rs485_state.isSending &&
        ((uint32_t)(now - rs485_state.txStartTick) > MODBUS_TX_WATCHDOG_MS)) {
        (void)HAL_UART_AbortTransmit(&MDSUARTx);
        rs485_state.isSending = 0;
        rs485_state.txStartTick = 0;
        g_modbus_uart1_tx_watchdog_count++;
        RS485_Enable_RX(RS485_EN_PORT, RS485_EN_PIN);
        (void)HAL_UART_Receive_IT(&MDSUARTx, &rx_data, 1);
    }

    if (rs485_state_h.isSending &&
        ((uint32_t)(now - rs485_state_h.txStartTick) > MODBUS_TX_WATCHDOG_MS)) {
        (void)HAL_UART_AbortTransmit(&MDSUARTxH);
        rs485_state_h.isSending = 0;
        rs485_state_h.txStartTick = 0;
        g_modbus_uart5_tx_watchdog_count++;
        RS485_Enable_RX(RS485_EN_PORT_H, RS485_EN_PIN_H);
        (void)HAL_UART_Receive_IT(&MDSUARTxH, &rx_data_h, 1);
    }
}

