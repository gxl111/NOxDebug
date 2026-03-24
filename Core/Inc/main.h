/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint32_t time_1s;
extern uint32_t time_1s_blow;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RS4851 huart1
#define SENSOR_POWER1_Pin GPIO_PIN_13
#define SENSOR_POWER1_GPIO_Port GPIOC
#define SENSOR_POWER0_Pin GPIO_PIN_0
#define SENSOR_POWER0_GPIO_Port GPIOC
#define RS485_DR1_Pin GPIO_PIN_1
#define RS485_DR1_GPIO_Port GPIOC
#define J5_IN_Pin GPIO_PIN_3
#define J5_IN_GPIO_Port GPIOC
#define J4_IN_Pin GPIO_PIN_0
#define J4_IN_GPIO_Port GPIOA
#define J3_IN_Pin GPIO_PIN_1
#define J3_IN_GPIO_Port GPIOA
#define J2_IN_Pin GPIO_PIN_3
#define J2_IN_GPIO_Port GPIOA
#define J1_IN_Pin GPIO_PIN_4
#define J1_IN_GPIO_Port GPIOA
#define SD_SPI_SCK_Pin GPIO_PIN_5
#define SD_SPI_SCK_GPIO_Port GPIOA
#define SD_SPI_MISO_Pin GPIO_PIN_6
#define SD_SPI_MISO_GPIO_Port GPIOA
#define SD_SPI_MOSI_Pin GPIO_PIN_7
#define SD_SPI_MOSI_GPIO_Port GPIOA
#define RS485_DR2_Pin GPIO_PIN_5
#define RS485_DR2_GPIO_Port GPIOC
#define RS485_DR3_Pin GPIO_PIN_2
#define RS485_DR3_GPIO_Port GPIOB
#define RS485_RE_Pin GPIO_PIN_12
#define RS485_RE_GPIO_Port GPIOB
#define REMOTE_CRT_Pin GPIO_PIN_7
#define REMOTE_CRT_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_8
#define SD_CS_GPIO_Port GPIOC
#define MCP2515_CS_Pin GPIO_PIN_10
#define MCP2515_CS_GPIO_Port GPIOC
#define MCP2515_INT_Pin GPIO_PIN_11
#define MCP2515_INT_GPIO_Port GPIOC
#define J6_IN_Pin GPIO_PIN_3
#define J6_IN_GPIO_Port GPIOB
#define J7_IN_Pin GPIO_PIN_4
#define J7_IN_GPIO_Port GPIOB
#define J8_IN_Pin GPIO_PIN_5
#define J8_IN_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB
#define J9_IN_Pin GPIO_PIN_8
#define J9_IN_GPIO_Port GPIOB
#define SENSOR_POWER2_Pin GPIO_PIN_9
#define SENSOR_POWER2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* MCP2515 SPI-CAN: CS=PC10, INT=PC11 (SPI2: PB13 SCK, PB14 MISO, PB15 MOSI) */
#define MCP2515_CS_Pin         GPIO_PIN_10
#define MCP2515_CS_GPIO_Port  GPIOC
#define MCP2515_INT_Pin       GPIO_PIN_11
#define MCP2515_INT_GPIO_Port GPIOC
/* Sensor power control GPIO (SENSOR_POWER_GPIO_ENABLE=1 in app_config.h): S1=PC0, S2=PC13, S3=PB9 */
#define SENSOR_POWER0_Pin       GPIO_PIN_0
#define SENSOR_POWER0_GPIO_Port GPIOC
#define SENSOR_POWER1_Pin       GPIO_PIN_13
#define SENSOR_POWER1_GPIO_Port GPIOC
#define SENSOR_POWER2_Pin       GPIO_PIN_9
#define SENSOR_POWER2_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
