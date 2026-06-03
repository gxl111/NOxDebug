/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "J1939.H"
#include <string.h>
#include <stdio.h>
#include "NOx.h"
#include "mcp2515_spi_can.h"
#include "oled.h"  
#include "sdcard.h"
#include "modbus_slave.h"
#include "app_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 1 = 仅跑 defaultTask：片内 CAN + MCP2515 周期发送加热帧 */
#define CAN_HEATER_TX_TEST_MODE 0
#define CAN_HEATER_TX_TEST_PERIOD_MS 100u
#define MCP2515_REINIT_PERIOD_MS 1000u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
char buffer[30];
char SD_FileName[] = "config.txt";
uint8_t WriteBuffer[] = "baudrate=115200\r\n";
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for NOx_Default */
osThreadId_t NOx_DefaultHandle;
const osThreadAttr_t NOx_Default_attributes = {
  .name = "NOx_Default",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ModBus_Slave */
osThreadId_t ModBus_SlaveHandle;
const osThreadAttr_t ModBus_Slave_attributes = {
  .name = "ModBus_Slave",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for NOx_Receive */
osThreadId_t NOx_ReceiveHandle;
const osThreadAttr_t NOx_Receive_attributes = {
  .name = "NOx_Receive",
  .stack_size = 1536 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ModBus_Host */
osThreadId_t ModBus_HostHandle;
const osThreadAttr_t ModBus_Host_attributes = {
  .name = "ModBus_Host",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void NOxDefault(void *argument);
void ModBusSlave(void *argument);
void NOxReceive(void *argument);
void ModBusHost(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
#if CAN_HEATER_TX_TEST_MODE
  /*
   * CAN/MCP 初始化放在 StartDefaultTask 里（调度器已运行后再做）。
   * 原因：若放在此处，J1939_Initialization 里已 Start CAN，RX 中断可能在
   * vTaskStartScheduler 之前进 FromISR；且 MCP2515_Init 里 HAL_Delay 依赖 TIM6，
   * 任一环节卡死都会表现为「永远进不了 StartDefaultTask」（其实尚未 osThreadNew）。
   */
#else
    
#if APP_USE_OLED
  OLED_Init();
  OLED_PrintASCIIString(0, 30, "waiting sd ", &afont16x8, OLED_COLOR_REVERSED);
  OLED_ShowFrame();
#else
#endif
  //WritetoSD(SD_FileName,WriteBuffer, sizeof(WriteBuffer));  
  handleConfig();
    


  J1939_Initialization();
	// ????????????????????????????????????????????
	g_hVarMutex = xSemaphoreCreateRecursiveMutex();
	configASSERT(g_hVarMutex != NULL);
//  HAL_CAN_Start(&hcan);
//  
//  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif
    
  

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  configASSERT(defaultTaskHandle != NULL);

#if !CAN_HEATER_TX_TEST_MODE
  /* creation of NOx_Default */
  NOx_DefaultHandle = osThreadNew(NOxDefault, NULL, &NOx_Default_attributes);
  configASSERT(NOx_DefaultHandle != NULL);

  /* creation of ModBus_Slave */
  ModBus_SlaveHandle = osThreadNew(ModBusSlave, NULL, &ModBus_Slave_attributes);
  configASSERT(ModBus_SlaveHandle != NULL);

  /* creation of NOx_Receive */
  NOx_ReceiveHandle = osThreadNew(NOxReceive, NULL, &NOx_Receive_attributes);
  configASSERT(NOx_ReceiveHandle != NULL);

  /* creation of ModBus_Host */
  ModBus_HostHandle = osThreadNew(ModBusHost, NULL, &ModBus_Host_attributes);
  configASSERT(ModBus_HostHandle != NULL);
#endif

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
#if CAN_HEATER_TX_TEST_MODE
  J1939_Initialization();
  uint32_t last_mcp_try_tick = 0U;
  TickType_t last_wake_tick = xTaskGetTickCount();
  (void)MCP2515_Init(MCP2515_BAUD_250K);
  J1939_MESSAGE tx_msg;
  TxMsg_Init(&tx_msg);
#endif
  /* Infinite loop */
  for(;;)
  {
#if CAN_HEATER_TX_TEST_MODE
    J1939_CAN_Transmit(&tx_msg);
    if (!MCP2515_IsReady()) {
      uint32_t now = HAL_GetTick();
      if ((now - last_mcp_try_tick) >= MCP2515_REINIT_PERIOD_MS) {
        last_mcp_try_tick = now;
        (void)MCP2515_Init(MCP2515_BAUD_250K);
      }
    } else {
      MCP2515_CAN_Frame_t mcp_heater;
      mcp_heater.id = J1939_HEATER_CAN_ID;
      mcp_heater.is_ext_id = true;
      mcp_heater.len = 8;
      for (int i = 0; i < 8; i++) {
        mcp_heater.data[i] = tx_msg.Mxe.Data[i];
      }
      (void)MCP2515_Send(&mcp_heater);
    }
    vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(CAN_HEATER_TX_TEST_PERIOD_MS));
#else
    vTaskDelay(pdMS_TO_TICKS(100));
#endif
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_NOxDefault */
/**
* @brief Function implementing the NOx_Default thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NOxDefault */
__weak void NOxDefault(void *argument)
{
  /* USER CODE BEGIN NOxDefault */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END NOxDefault */
}

/* USER CODE BEGIN Header_ModBusSlave */
/**
* @brief Function implementing the ModBus_Slave thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ModBusSlave */
__weak void ModBusSlave(void *argument)
{
  /* USER CODE BEGIN ModBusSlave */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ModBusSlave */
}

/* USER CODE BEGIN Header_NOxReceive */
/**
* @brief Function implementing the NOx_Receive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NOxReceive */
__weak void NOxReceive(void *argument)
{
  /* USER CODE BEGIN NOxReceive */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END NOxReceive */
}

/* USER CODE BEGIN Header_ModBusHost */
/**
* @brief Function implementing the ModBus_Host thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ModBusHost */
__weak void ModBusHost(void *argument)
{
  /* USER CODE BEGIN ModBusHost */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ModBusHost */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

