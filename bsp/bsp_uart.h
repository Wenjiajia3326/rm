/**
 * @file    bsp_uart.h
 * @brief   串口缓冲：DMA 循环单缓冲（视觉/遥控）+ 逐字节中断（舵机）。
 * @owner   电控B
 *
 * 本模块只负责"把收到的字节放进缓冲、按索引读出"，不做协议解析。
 * 协议层用 GetLength / ReadByte / AddReadIndex 就地扫描，不复制整帧。
 *
 * 并发模型：单生产者（DMA 硬件或接收中断）写缓冲，单消费者（任务）读缓冲；
 * 读写索引都是 16 位对齐变量，Cortex-M3 上单次读写是原子的，读侧不需要关中断。
 */

#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

/* 串口通道编号 */
#define BSP_UART_VISION         0U      /**< USART3：视觉 */
#define BSP_UART_REMOTE         1U      /**< UART4：遥控 */
#define BSP_UART_SERVO          2U      /**< UART5：总线舵机 */

/* 通道数量：仅用于数组长度，不对应任何通道 */
#define BSP_UART_NUM            3U

/**
 * @brief 初始化三路串口接收
 *
 * DMA 通道启动 Circular 模式接收（之后不再干预）；中断通道启动 1 字节中断接收。
 *
 * @param  无
 * @return 无
 * @note   必须在 MX_*_UART_Init() 之后调用
 */
void BSP_Uart_Init(void);

/**
 * @brief 获取缓冲区中未处理的字节数
 * @param port 通道编号（BSP_UART_xxx）
 * @return 未处理字节数（0 表示无数据）
 */
uint16_t BSP_Uart_GetAvailableDataSize(uint8_t port);

/**
 * @brief 获取缓冲区剩余空间
 * @param port 通道编号
 * @return 剩余可写字节数
 */
uint16_t BSP_Uart_GetFreeSpace(uint8_t port);

/**
 * @brief 读取缓冲区中相对读指针的第 index 个字节
 * @param port  通道编号
 * @param index 相对偏移（0 = 下一个待处理字节），超过缓冲长度自动绕回
 * @return 该位置的字节
 * @note   不移动读指针；调用者先用 GetLength 确认 index 在有效范围内
 */
uint8_t BSP_Uart_ReadByte(uint8_t port, uint16_t index);

/**
 * @brief 读指针前移
 * @param port   通道编号
 * @param length 前移的字节数
 * @return 无
 * @note   只有任务侧调用（单消费者）
 */
void BSP_Uart_AddReadIndex(uint8_t port, uint16_t length);

/**
 * @brief 阻塞发送一段数据
 * @param port   通道编号
 * @param data   数据首地址
 * @param length 字节数
 * @note   超时 10 ms；不要在 10 ms 主环里发送长数据
 */
void BSP_Uart_SendData(uint8_t port, const uint8_t *data, uint16_t length);

#endif /* BSP_UART_H */
