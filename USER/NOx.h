/**
 * @file    NOx.h
 * @brief   NOx application: J1939 handling, tasks, and Modbus integration.
 *          Sensor math is in nox_sensor; this header ties app and protocol.
 */
#ifndef __NOX_H
#define __NOX_H

#include "main.h"
#include "J1939.h"
#include "nox_sensor.h"
#include "app_config.h"
#include <string.h>
#include <stdio.h>
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Parameter and sensor state come from nox_sensor.h */

/* FreeRTOS 栈调试：Watch 窗口可直接添加 */
extern volatile uint32_t g_nox_receive_stack_hwm;
extern volatile uint32_t g_freertos_stack_overflow_hits;
extern volatile char g_freertos_stack_overflow_task[];

void TxMsg_Init(J1939_MESSAGE *TxMsgPtr);
void hexArrayToString(const j1939_uint8_t *array, size_t length, char *result);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void RS485_Send_Data_IT(uint8_t *pData, uint16_t Size);

#endif /* __NOX_H */
