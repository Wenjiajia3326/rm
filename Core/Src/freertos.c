/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "app_config.h"
#include "app_robot.h"
#include "app_remote.h"
#include "app_vision.h"
#include "bsp_uart.h"
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

/* USER CODE END Variables */
/* Definitions for RobotTask */
osThreadId_t RobotTaskHandle;
const osThreadAttr_t RobotTask_attributes = {
  .name = "RobotTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartRobotTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

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
  /* creation of RobotTask */
  RobotTaskHandle = osThreadNew(StartRobotTask, NULL, &RobotTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartRobotTask */
/**
  * @brief  Function implementing the RobotTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRobotTask */
void StartRobotTask(void *argument)
{
  /* USER CODE BEGIN StartRobotTask */
  (void)argument;

  /* 初始化顺序：串口 -> 日志 -> 整机状态 -> 各应用模块 */
  BSP_Uart_Init();
  BSP_Log_Init();
  LOG("\r\n=== electB fw boot ===\r\n");

  App_Robot_Init();
  App_Remote_Init();
  App_Vision_Init();

  LOG("mode=%s period=%ums\r\n",
      App_Robot_ModeName(App_Robot_GetMode()),
      (unsigned int)APP_ROBOT_PERIOD_MS);

  /* 10 ms 主环 */
  for(;;)
  {
    App_Remote_Update();      /* 遥控帧：模式/档位、摇杆、按键、失联判断 */
    App_Vision_Update();      /* 视觉帧：路线锁存、灯色时效、在线状态 */

    osDelay(APP_ROBOT_PERIOD_MS);
  }
  /* USER CODE END StartRobotTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

