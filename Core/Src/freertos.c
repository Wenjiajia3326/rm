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
#include "app_arm.h"
#include "app_chassis.h"
#include "app_config.h"
#include "app_robot.h"
#include "app_selftest.h"
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
    // 按原理图约定轮位，实物接线和转动方向仍需核对。
    const ChassisMotorMap motor_map = // 轮位与电路板接口的对应表，建立后不修改。
    {
        .front_left  = BSP_MOTOR_1,
        .front_right = BSP_MOTOR_2,
        .rear_left   = BSP_MOTOR_3,
        .rear_right  = BSP_MOTOR_4
    };
    const TickType_t period_ticks = pdMS_TO_TICKS(CHASSIS_CONTROL_PERIOD_MS); // 把配置的10毫秒换成RTOS节拍数。
    TickType_t last_wake_tick; // 保存周期等待的计划时间基准，不是毫秒时间。
    uint8_t chassis_ready; // 启动结果：1表示准备成功，0表示失败。
    uint8_t mode;           // 本轮模式，避免重复读取。
    uint8_t drive_ok;       // 本周期是否允许底盘出力。
    float drive_scale;      // 互锁限速比例，1.0 = 满速。
    ChassisSpeed manual_target; // 摇杆换算出的实际速度目标。
    uint32_t previous_ms; // 上一轮读取时间的时刻，单位毫秒。
    uint32_t current_ms; // 本轮读取时间的时刻，单位毫秒。
    uint32_t elapsed_ms; // 两次读取时间之间经过的毫秒数。
    float dt_s; // 将实际间隔换算为秒，供测速、斜坡和PI使用。

    (void)argument; // 任务暂时不用传入参数，明确告诉编译器。

    // 先准备通信和整机状态，再初始化各应用模块。
    BSP_Uart_Init();
    BSP_Log_Init();
    App_Robot_Init();
    App_Remote_Init();
    App_Vision_Init();
    App_Arm_Init();

    LOG("\r\n=== AB fw boot ===\r\n");
    LOG("mode=%s period=%ums\r\n",
        App_Robot_GetModeName(App_Robot_GetMode()),
        (unsigned int)CHASSIS_CONTROL_PERIOD_MS);

#if (APP_SELFTEST_ENABLE != 0)
    (void)App_Selftest_Run();
#endif

    // 仅启动一次，保存底盘是否准备成功。
    chassis_ready = App_Chassis_Start(); // 检查配置并初始化底盘，把启动结果保存下来。

    // 在进入循环前建立时间起点，避免第一次使用未初始化的值。
    previous_ms = HAL_GetTick(); // 读取当前毫秒计时，作为第一次计算间隔的起点。
    last_wake_tick = xTaskGetTickCount(); // 读取RTOS当前节拍，作为周期等待的起点。

    for (;;)
    {
        // 按计划节拍等待，实际采样间隔仍由毫秒计时计算。
        vTaskDelayUntil(&last_wake_tick, period_ticks); // 等到下个计划时刻；传地址让函数更新这个时间基准。

        // 通信始终更新，不受底盘配置是否准备成功影响。
        App_Remote_Update(); // 处理遥控帧、模式、按键和失联判断。
        App_Vision_Update(); // 处理视觉路线、灯色和在线状态。
        App_Robot_Update();  // 推进整机状态：自动流程、调试推进。
        App_Arm_Update();    // 推进机械臂分段动作。

        current_ms = HAL_GetTick(); // 记录本轮实际开始处理的毫秒时刻。
        elapsed_ms = current_ms - previous_ms; // 本次减上次，得到实际间隔。

        // 毫秒计时尚未变化时，不进行本次计算。
        if (elapsed_ms == 0U)
        {
            continue;
        }

        previous_ms = current_ms; // 保存本次时刻，供下一轮相减。
        dt_s = (float)elapsed_ms / 1000.0f; // 毫秒除以1000变成秒，例如10毫秒变成0.01秒。

        if (chassis_ready == 0)
        {
            continue;
        }

        mode = App_Robot_GetMode();

        // 允许底盘出力：链路正常、机械臂空闲、无故障，且模式允许行驶。
        // 自动也要链路：自动是靠遥控把模式切进来的，链路在说明手柄还在。
        drive_ok = (uint8_t)(
            (App_Remote_IsLinkOk() != 0U) &&
            (App_Arm_IsBusy() == 0U) &&
            (App_Chassis_GetStatus() != CHASSIS_STATUS_FAULT) &&
            ((mode == APP_MODE_COMP_MANUAL_V) ||
             (mode == APP_MODE_CHASSIS_DEBUG_V) ||
             (mode == APP_MODE_COMP_AUTO_V)));

        // 只有手动模式提交摇杆目标；自动阶段的目标由底盘任务自己生成。
        if ((drive_ok != 0U) &&
            ((mode == APP_MODE_COMP_MANUAL_V) || (mode == APP_MODE_CHASSIS_DEBUG_V)))
        {
            // 互锁限速：收纳空载 100%，臂未收纳 50%，携块 70%，同时满足取小。
            // 动作进行中已由 drive_ok 直接关掉，不在这里。
            drive_scale = (float)CHASSIS_LIMIT_NORMAL_PCT / 100.0f;

            if (App_Arm_IsStowed() == 0U)
            {
                drive_scale = (float)CHASSIS_LIMIT_ARM_OUT_PCT / 100.0f;
            }

            if ((App_Robot_GetLoad() == LOAD_CARRY_V) &&
                (((float)CHASSIS_LIMIT_CARRY_PCT / 100.0f) < drive_scale))
            {
                drive_scale = (float)CHASSIS_LIMIT_CARRY_PCT / 100.0f;
            }

            // 地形档选"隧道"时压到 40%，由操作手在进隧道前切档
            if ((App_Robot_GetTerrain() == TERRAIN_TUNNEL_V) &&
                (((float)CHASSIS_LIMIT_TUNNEL_PCT / 100.0f) < drive_scale))
            {
                drive_scale = (float)CHASSIS_LIMIT_TUNNEL_PCT / 100.0f;
            }

            manual_target.vx = (float)App_Remote_GetVx() * CHASSIS_MAX_LINEAR_MPS * drive_scale / 100.0f;
            manual_target.vy = (float)App_Remote_GetVy() * CHASSIS_MAX_LINEAR_MPS * drive_scale / 100.0f;
            manual_target.wz = (float)App_Remote_GetWz() * CHASSIS_MAX_ANGULAR_RPS * drive_scale / 100.0f;
            (void)App_Chassis_SetTarget(&manual_target);
        }

        // 不许出力时返回0属于正常情况，不能据此取消启动成功状态。
        (void)App_Chassis_Run(&motor_map, drive_ok, dt_s); // 传轮位表、本周期许可和实际间隔。
    }
  /* USER CODE END StartRobotTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

