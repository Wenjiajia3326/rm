/**
 * @file    bsp_config.h
 * @brief   板级配置：串口 DMA 句柄声明、缓冲大小。
 * @owner   硬件/电控A（只放板级信息，业务参数见 application/app_config.h）
 */

#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

#include "usart.h"      /* CubeMX 生成：huart1/3/4/5 */

/* CubeMX 把 DMA 句柄定义在 usart.c 里，这里声明供其它文件使用 */
extern DMA_HandleTypeDef hdma_usart3_rx;    /**< USART3_RX → DMA1_Channel3，视觉 */
extern DMA_HandleTypeDef hdma_uart4_rx;     /**< UART4_RX  → DMA2_Channel3，遥控 */

/** DMA 通道缓冲大小（DMA 直接写这块内存，同时就是软件读缓冲） */
#define BSP_UART_DMA_BUF_SIZE   256U

/** 中断通道（舵机）缓冲大小 */
#define BSP_UART_IT_BUF_SIZE    64U

// CubeMX生成的定时器句柄声明。
#include "tim.h"

// 电机接口对应的编码器定时器，不代表实际轮位。
#define BSP_MOTOR_1_ENCODER_TIM    (&htim2)
#define BSP_MOTOR_2_ENCODER_TIM    (&htim3)
#define BSP_MOTOR_3_ENCODER_TIM    (&htim4)
#define BSP_MOTOR_4_ENCODER_TIM    (&htim8)



// M1：TIM5_CH2输出PWM，PB0连接驱动IN2。
#define BSP_MOTOR_1_PWM_TIM        (&htim5)
#define BSP_MOTOR_1_PWM_CHANNEL    TIM_CHANNEL_2
#define BSP_MOTOR_1_IN2_PORT       M1_IN2_GPIO_Port
#define BSP_MOTOR_1_IN2_PIN        M1_IN2_Pin

// M2：TIM5_CH3输出PWM，PB4连接驱动IN2。
#define BSP_MOTOR_2_PWM_TIM        (&htim5)
#define BSP_MOTOR_2_PWM_CHANNEL    TIM_CHANNEL_3
#define BSP_MOTOR_2_IN2_PORT       M2_IN2_GPIO_Port
#define BSP_MOTOR_2_IN2_PIN        M2_IN2_Pin

// M3：TIM5_CH4输出PWM，PB9连接驱动IN2。
#define BSP_MOTOR_3_PWM_TIM        (&htim5)
#define BSP_MOTOR_3_PWM_CHANNEL    TIM_CHANNEL_4
#define BSP_MOTOR_3_IN2_PORT       M3_IN2_GPIO_Port
#define BSP_MOTOR_3_IN2_PIN        M3_IN2_Pin

// M4：TIM1_CH1输出PWM，PC5连接驱动IN2。
#define BSP_MOTOR_4_PWM_TIM        (&htim1)
#define BSP_MOTOR_4_PWM_CHANNEL    TIM_CHANNEL_1
#define BSP_MOTOR_4_IN2_PORT       M4_IN2_GPIO_Port
#define BSP_MOTOR_4_IN2_PIN        M4_IN2_Pin


#endif /* BSP_CONFIG_H */
