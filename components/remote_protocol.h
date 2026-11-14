/**
 * @file    remote_protocol.h
 * @brief   遥控帧解析（TYPE 0x20，15 字节定长）：从串口缓冲就地取帧。
 * @owner   电控B
 *
 * 帧结构（与遥控帧协议 V1.3 一致）：
 *   [0]AA [1]55 [2]VER=0x02 [3]TYPE=0x20 [4]LEN=9
 *   [5]vx [6]vy [7]btn_low [8]btn_high
 *   [9]mode [10]terrain [11]level [12]load [13]seq [14]CRC8
 */

#ifndef REMOTE_PROTOCOL_H
#define REMOTE_PROTOCOL_H

#include <stdint.h>

/** 一帧遥控数据（手持端"已生效"的值，不是候选） */
typedef struct
{
    int8_t  vx;         /**< 前后，-100~100，+ 为前 */
    int8_t  vy;         /**< 平移，-100~100，+ 为左 */
    uint8_t btn_low;    /**< 低 8 键：动作 + 旋转 */
    uint8_t btn_high;   /**< 高 8 键：bit0 精细切换、bit1 急停 */
    uint8_t mode;       /**< 模式：APP_MODE_*_V */
    uint8_t terrain;    /**< 地形档：TERRAIN_*_V */
    uint8_t level;      /**< 高度档索引 */
    uint8_t load;       /**< 载荷：LOAD_EMPTY_V / LOAD_CARRY_V */
    uint8_t seq;        /**< 帧序号 */
} RemoteData_t;

/**
 * @brief 复位统计量（丢帧基准与计数）
 * @param  无
 * @return 无
 * @note   上电调用一次
 */
void Remote_Init(void);

/**
 * @brief 从遥控缓冲中取一帧完整数据
 *
 * 就地扫描：帧头/版本/类型/长度/CRC 任一项不符就把读指针前移 1 字节继续找，
 * 找到完整有效帧后整体前移一帧。
 *
 * @param out 解析结果（仅返回 1 时写入）
 * @return 1 = 取到一帧有效数据
 *         0 = 数据不足一帧，或当前没有有效帧（无效字节已被丢弃）
 * @note   非阻塞；建议主环里 while 循环取到返回 0 为止
 * @note   只有任务侧调用（单消费者）
 */
uint8_t Remote_GetFrame(RemoteData_t *out);

/**
 * @brief 取累计丢帧数（按 seq 跳变估算）
 * @param  无
 * @return 丢帧总数
 * @note   仅诊断用，不参与控制
 */
uint32_t Remote_GetLostCount(void);

/**
 * @brief 取累计有效帧数
 * @param  无
 * @return 有效帧总数
 * @note   30 Hz 下每秒增量约 30
 */
uint32_t Remote_GetFrameCount(void);

#endif /* REMOTE_PROTOCOL_H */
