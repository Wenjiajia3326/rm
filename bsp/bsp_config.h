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

#endif /* BSP_CONFIG_H */
