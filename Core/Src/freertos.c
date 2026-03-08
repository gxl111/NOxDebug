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
#include "oled.h"
#include "sdcard.h"
#include "modbus_slave.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ModBus_Slave */
osThreadId_t ModBus_SlaveHandle;
const osThreadAttr_t ModBus_Slave_attributes = {
  .name = "ModBus_Slave",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for NOx_Receive */
osThreadId_t NOx_ReceiveHandle;
const osThreadAttr_t NOx_Receive_attributes = {
  .name = "NOx_Receive",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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
  OLED_Init();
    
  OLED_PrintASCIIString(0, 30, "waiting sd ", &afont16x8, OLED_COLOR_REVERSED);
  OLED_ShowFrame();
  //WritetoSD(SD_FileName,WriteBuffer, sizeof(WriteBuffer));  
  handleConfig();
    


  J1939_Initialization();
	// ????????????????????????????????????????????
	g_hVarMutex = xSemaphoreCreateRecursiveMutex();
	configASSERT(g_hVarMutex != NULL);
//  HAL_CAN_Start(&hcan);
//  
//  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    
  

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

  /* creation of NOx_Default */
  NOx_DefaultHandle = osThreadNew(NOxDefault, NULL, &NOx_Default_attributes);

  /* creation of ModBus_Slave */
  ModBus_SlaveHandle = osThreadNew(ModBusSlave, NULL, &ModBus_Slave_attributes);

  /* creation of NOx_Receive */
  NOx_ReceiveHandle = osThreadNew(NOxReceive, NULL, &NOx_Receive_attributes);

  /* creation of ModBus_Host */
  ModBus_HostHandle = osThreadNew(ModBusHost, NULL, &ModBus_Host_attributes);

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
  /* Infinite loop */
  for(;;)
  {

    vTaskDelay(pdMS_TO_TICKS(100));
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

