/**
 * @file    bsp_log.c
 * @brief   日志输出实现：vsnprintf 格式化到静态缓冲 + 阻塞发送到 USART1（板载 CH340）。
 * @owner   电控B
 *
 * 为什么不用 printf 重定向：重定向要改 syscalls.c 的 _write，且 printf 内部有全局锁、
 * 开销更大；这里只用一个 80 字节静态缓冲 + vsnprintf，长度可控、行为可预测。
 */

#include "bsp_log.h"
#include "usart.h"      /* huart1 */

#include <stdarg.h>     /* va_list / va_start / va_end */
#include <stdio.h>      /* vsnprintf */
#include <string.h>     /* strlen */

/** 格式化缓冲：静态分配，避免每次调用在任务栈上开大数组 */
static char s_log_buf[BSP_LOG_MAX_LEN];

/**
 * @brief 初始化日志通道
 *
 * 当前无需配置（无缓冲队列、无 DMA）。保留接口，方便以后改成 DMA 发送。
 *
 * @param  无
 * @return 无
 */
void BSP_Log_Init(void)
{
    s_log_buf[0] = '\0';
}

/**
 * @brief 直接发送一段字符串到 USART1
 *
 * @param str 以 '\0' 结尾的字符串，调用者保证非空
 * @return 无
 *
 * @note  超时 10 ms：正常 80 字节约 7 ms 发完；超时则丢弃剩余数据，不卡死调用者
 * @note  长度由 strlen 决定
 */
void BSP_Log_Print(const char *str)
{
    uint16_t len;

    len = (uint16_t)strlen(str);
    if (len == 0U)
    {
        return;
    }

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)str, len, 10U);
}

/**
 * @brief 格式化并发送一条日志
 *
 * @param fmt printf 风格格式串，调用者保证非空
 * @param ... 可变参数
 * @return 无
 *
 * @note  关键的返回值处理：n <= 0 表示本次没有格式化出任何内容，
 *        此时 s_log_buf 仍是上一条日志的内容，必须直接返回，
 *        否则会把上一条重复发出去（日志里表现为莫名重复行，很难查）
 * @note  支持 %d %u %ld %lu %x %s %c %% ；不支持 %f
 * @note  不可重入：内部使用静态缓冲 s_log_buf
 */
void BSP_Log_PrintFormat(const char *fmt, ...)
{
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(s_log_buf, sizeof(s_log_buf), fmt, ap);
    va_end(ap);

    if (n <= 0)
    {
        return;
    }

    BSP_Log_Print(s_log_buf);
}
