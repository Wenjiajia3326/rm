/**
 * @file    remote_protocol.h
 * @brief   遥控帧解析（TYPE 0x20，11 字节定长）：字节流 -> RemoteData_t
 * @owner   电控B
 *
 * 本文件只负责"字节流还原成结构体"，不做任何控制逻辑：
 * 按键含义、边沿检测、模式切换由 app_remote 负责；
 * 帧结构常量与按键位定义集中放在 app_config.h。
 */

#ifndef REMOTE_PROTOCOL_H
#define REMOTE_PROTOCOL_H

#include <stdint.h>

/** 一帧遥控数据的解析结果 */
typedef struct
{
    int8_t  vx;         ///< 前后，-100~100（百分比满速），+ 为前
    int8_t  vy;         ///< 平移，-100~100，+ 为左
    uint8_t btn_low;    ///< 低 8 键，位定义见 app_config.h 的 RC_BTN_xxx
    uint8_t btn_high;   ///< 高 8 键，位定义见 app_config.h 的 RC_BTN_xxx
    uint8_t seq;        ///< 帧序号 0~255 回绕，用于统计丢帧
} RemoteData_t;

/**
 * @brief 复位解析器（上电或需要重新同步时调用）
 */
void Remote_Init(void);

/**
 * @brief 每收到一个串口字节调用一次
 * @param byte 串口收到的字节
 * @param out  解析结果，仅在返回 1 时有效
 * @return 1 = 收到一帧完整且校验正确的遥控数据，0 = 还没收齐或校验失败
 * @note  不阻塞；内部维护 11 字节滑动窗口，帧头不符时左移一位继续找
 * @note  必须在串口接收上下文（中断或接收任务）中调用，不可并发调用
 */
uint8_t Remote_ParseByte(uint8_t byte, RemoteData_t *out);

#endif /* REMOTE_PROTOCOL_H */
