/**
 * @file    vision_protocol.h
 * @brief   视觉帧解析（0x10 选路 / 0x11 灯色 / 0x12 心跳）：从串口缓冲就地取帧。
 * @owner   电控B
 *
 * 帧结构（《萝卜杯_视觉串口协议 V1.0》）：
 *   [0]AA [1]55 [2]VER=0x01 [3]TYPE [4]LEN [5..]DATA [5+LEN]CRC8
 *   0x10 / 0x11：LEN=4，总长 10
 *   0x12       ：LEN=2，总长 8
 *
 * 本模块只做"字节 -> 结构体"；时效、锁定、在线判断在 app_vision。
 */

#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdint.h>

/** 一帧视觉数据（按 type 取对应字段，未使用的字段为 0） */
typedef struct
{
    uint8_t type;       /**< 0x10 选路 / 0x11 灯色 / 0x12 心跳 */
    uint8_t route;      /**< 0x10：路线 1~3；0 = 没看见；其它帧为 0 */
    uint8_t light;      /**< 0x11：1 红 / 2 绿 / 3 黄；0 = 没看见；其它帧为 0 */
    uint8_t state;      /**< 0x12：bit0 = 1 表示视觉在线；其它帧为 0 */
    uint8_t conf;       /**< 0x10/0x11：置信度 0~100 */
    uint8_t ttl_50ms;   /**< 0x10/0x11：有效期，单位 50 ms */
    uint8_t seq;        /**< 0x10/0x11：帧序号，0~255 回绕 */
    uint8_t fps;        /**< 0x12：视觉自报帧率，0 表示暂不提供 */
} VisionFrame_t;

/**
 * @brief 复位解析器（seq 基准）
 * @param  无
 * @return 无
 * @note   上电调用一次
 */
void Vision_Init(void);

/**
 * @brief 重设 seq 期望基准，下一帧不参与丢帧判断
 * @param  无
 * @return 无
 * @note   由 app_vision 在 vision_ok 从 0 变 1 时调用（视觉端可能重启，seq 从 0 重来）
 */
void Vision_ResetSeqExpectation(void);

/**
 * @brief 从视觉缓冲中取一帧完整数据
 *
 * 就地扫描：帧头/版本/类型/长度/CRC 任一不符就把读指针前移 1 字节继续找；
 * 0x10/0x11 帧若 seq 与上一帧相同视为重复帧，丢弃后继续找下一帧。
 *
 * @param out 解析结果（仅返回 1 时写入）
 * @return 1 = 取到一帧有效数据
 *         0 = 数据不足一帧，或当前没有有效帧（无效字节已被丢弃）
 * @note   非阻塞；建议主环里 while 循环取到返回 0 为止
 * @note   只有任务侧调用（单消费者）
 */
uint8_t Vision_GetFrame(VisionFrame_t *out);

#endif /* VISION_PROTOCOL_H */
