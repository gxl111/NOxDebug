/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RS485_DR1_Pin|RS485_DR2_Pin|SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS1_GPIO_Port, SD_CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS485_DR3_Pin|RS485_RE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level : 传感器电源 S1=PC0, S2=PC13, S3=PB9，默认低 */
  HAL_GPIO_WritePin(SENSOR_POWER0_GPIO_Port, SENSOR_POWER0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SENSOR_POWER1_GPIO_Port, SENSOR_POWER1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SENSOR_POWER2_GPIO_Port, SENSOR_POWER2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level : 阀门 J1-J9 默认关 */
  HAL_GPIO_WritePin(J1_IN_GPIO_Port, J1_IN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J2_IN_GPIO_Port, J2_IN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J3_IN_GPIO_Port, J3_IN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J4_IN_GPIO_Port, J4_IN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J5_IN_GPIO_Port, J5_IN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J6_IN_GPIO_Port, J6_IN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J7_IN_GPIO_Port, J7_IN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J8_IN_GPIO_Port, J8_IN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J9_IN_GPIO_Port, J9_IN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SENSOR_POWER0(PC0) SENSOR_POWER1(PC13) SENSOR_POWER2(PB9) */
  GPIO_InitStruct.Pin = SENSOR_POWER0_Pin | SENSOR_POWER1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = SENSOR_POWER2_Pin;
  HAL_GPIO_Init(SENSOR_POWER2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : J1(J1_IN) J2(J2_IN) J3(J3_IN) J4(J4_IN) - PA4,PA3,PA1,PA0 */
  GPIO_InitStruct.Pin = J1_IN_Pin | J2_IN_Pin | J3_IN_Pin | J4_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : J5_IN (PC3) */
  GPIO_InitStruct.Pin = J5_IN_Pin;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : J6 J7 J8 J9 - PB3,PB4,PB5,PB8 */
  GPIO_InitStruct.Pin = J6_IN_Pin | J7_IN_Pin | J8_IN_Pin | J9_IN_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_DR1_Pin RS485_DR2_Pin */
  GPIO_InitStruct.Pin = RS485_DR1_Pin|RS485_DR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS1_Pin */
  GPIO_InitStruct.Pin = SD_CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_DR3_Pin */
  GPIO_InitStruct.Pin = RS485_DR3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RS485_DR3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_RE_Pin */
  GPIO_InitStruct.Pin = RS485_RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : REMOTE_CRT_Pin */
  GPIO_InitStruct.Pin = REMOTE_CRT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(REMOTE_CRT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MCP2515_CS(PC10) MCP2515_INT(PC11)，与 NOx_RCT6.ioc / SPI2 第二路 CAN 一致 */
  HAL_GPIO_WritePin(MCP2515_CS_GPIO_Port, MCP2515_CS_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = MCP2515_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MCP2515_CS_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = MCP2515_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MCP2515_INT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
