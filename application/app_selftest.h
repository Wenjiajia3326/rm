/**
 * @file    app_selftest.h
 * @brief   上电自测：用协议文档给的样例帧验证解析与基础状态机。
 * @owner   电控B
 *
 * 只在 APP_SELFTEST_ENABLE = 1 时编译；比赛固件置 0 后整段消失。
 * 用例不触发舵机与电机动作，只跑纯逻辑。
 */

#ifndef APP_SELFTEST_H
#define APP_SELFTEST_H

#include <stdint.h>

/**
 * @brief 跑一遍全部自测并打印结果
 * @param  无
 * @return 0 = 全部通过；非 0 = 失败用例个数
 * @note   需在 BSP_Uart_Init() 之后调用（依赖串口缓冲区）
 * @note   阻塞发送若干行日志，仅上电跑一次
 */
uint8_t App_Selftest_Run(void);

#endif /* APP_SELFTEST_H */
