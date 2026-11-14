/**
 * @file    bsp_log.h
 * @brief   日志输出接口：固定走板载 CH340（USART1），阻塞发送，可整体开关。
 * @owner   电控B
 *
 * 设计说明：
 * 1) 只走 USART1（PA9/PA10 → 板载 CH340），调用方不需要指定通道；
 * 2) 发送是阻塞的：80 字节 @115200 约 6.9 ms，因此不要放在 10 ms 主环的每帧路径里；
 * 3) 本文件只提供函数；开关 APP_LOG_ENABLE 与 LOG(...) 宏在 application/app_config.h，
 *    这样 bsp 层不依赖 application 层（分层方向：app -> bsp）。
 *
 * 用法：
 *   LOG("mode -> %s\r\n", App_Robot_ModeName(mode));
 *   LOG("rx=%lu lost=%lu\r\n", Remote_GetFrameCount(), Remote_GetLostCount());
 */

#ifndef BSP_LOG_H
#define BSP_LOG_H

#include <stdint.h>

/** 单条日志最大长度（字节）。80 B @115200 8N1 约阻塞 6.9 ms；超长会被截断 */
#define BSP_LOG_MAX_LEN         80U

/**
 * @brief 初始化日志通道
 *
 * 在 MX_USART1_UART_Init() 之后调用一次。当前无需配置，
 * 保留接口以便以后改成 DMA 发送时把初始化放在这里。
 *
 * @param  无
 * @return 无
 */
void BSP_Log_Init(void);

/**
 * @brief 直接发送一段字符串到日志口（不做格式化）
 *
 * @param str 以 '\0' 结尾的字符串（调用者保证；本函数不判空）
 * @return 无
 *
 * @note  仅任务上下文调用；阻塞发送，超时 10 ms
 */
void BSP_Log_Print(const char *str);

/**
 * @brief 格式化并发送一条日志（printf 风格）
 *
 * 先用 vsnprintf 把内容写入内部静态缓冲（最多 BSP_LOG_MAX_LEN-1 字节，超出截断并补 '\0'），
 * 再调用 BSP_Log_Print() 阻塞发送。
 *
 * @param fmt printf 风格格式串（调用者保证非空）
 * @param ... 与格式串对应的可变参数
 * @return 无
 *
 * @note  不可重入：内部使用静态缓冲，只能在一个任务里调用
 * @note  不支持 %f（不链接浮点 printf，省 FLASH）；小数请先换算成整数再打
 */
void BSP_Log_PrintFormat(const char *fmt, ...);

#endif /* BSP_LOG_H */
