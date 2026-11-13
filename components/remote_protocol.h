/**
 * @file    remote_protocol.h
 * @brief   遥控帧解析（TYPE 0x20，15 字节定长）：字节流 -> RemoteData_t，并按 seq 统计丢帧。
 * @owner   电控B
 *
 * 帧结构（与遥控帧协议 V1.3 一致）：
 *   [0]AA [1]55 [2]VER=0x02 [3]TYPE=0x20 [4]LEN=9
 *   [5]vx [6]vy [7]btn_low [8]btn_high
 *   [9]mode [10]terrain [11]level [12]load [13]seq [14]CRC8
 * CRC 覆盖 [2]~[13] 共 12 字节。
 *
 * 本文件只做"字节流还原成结构体 + 统计"，不含任何控制逻辑：
 * 按键边沿、动作分发、超时判断由 app_remote 负责。
 */

#ifndef REMOTE_PROTOCOL_H
#define REMOTE_PROTOCOL_H

#include <stdint.h>

/** 一帧遥控数据的解析结果（全部为手持端"已生效"的值，不是候选） */
typedef struct
{
    int8_t  vx;         ///< 前后，-100~100（百分比满速），+ 为前
    int8_t  vy;         ///< 平移，-100~100，+ 为左
    uint8_t btn_low;    ///< 低 8 键：A/B/C/D + 旋转 4 键，位定义见 app_config.h 的 RC_BTN_xxx
    uint8_t btn_high;   ///< 高 8 键：bit0 精细切换、bit1 急停，其余预留
    uint8_t mode;       ///< 当前模式，取值 APP_MODE_*_V
    uint8_t terrain;    ///< 地形档，取值 TERRAIN_*_V
    uint8_t level;      ///< 高度档索引，0 ~ LEVEL_NUM-1
    uint8_t load;       ///< 载荷状态，LOAD_EMPTY_V / LOAD_CARRY_V
    uint8_t seq;        ///< 帧序号 0~255 回绕
} RemoteData_t;

/**
 * @brief 复位解析器与统计量
 *
 * 上电调用一次；把滑动窗口长度、seq 基准、丢帧/收帧计数全部清零。
 *
 * @param  无
 * @return 无
 *
 * @note   可在任务上下文调用；不要在解析过程中被中断上下文并发调用
 */
void Remote_Init(void);

/**
 * @brief 送入一个串口字节，尝试解析出一帧完整遥控数据
 *
 * 维护 15 字节滑动窗口：帧头不符时左移一位继续找；收满一帧后依次校验
 * 版本/类型/长度/字段范围/CRC，全部通过才填充 out 并更新统计量。
 *
 * @param byte 刚从 UART 收到的原始字节
 * @param out  解析结果；**仅在返回 1 时被写入**，返回 0 时不修改其内容：
 *               - vx/vy         : 摇杆，-100~100
 *               - btn_low/high  : 按键位图（RC_BTN_xxx）
 *               - mode/terrain/level/load : 手持端已生效的模式与档位
 *               - seq           : 帧序号
 * @return 1 = 收到一帧完整且校验通过的数据（out 已填充，统计已更新）
 *         0 = 未收齐，或帧头/版本/类型/长度/字段范围/CRC 任一不符（该帧丢弃）
 *
 * @note   调用上下文：串口接收任务，单线程调用，不可重入
 * @note   字段范围检查会拒收越界值（mode/terrain/level/load），用于发现手持端与主控档数配置不一致
 * @note   不阻塞；最坏情况每字节做一次 14 字节窗口左移
 */
uint8_t Remote_ParseByte(uint8_t byte, RemoteData_t *out);

/**
 * @brief 取累计丢帧数（按 seq 跳变估算）
 * @param  无
 * @return 自 Remote_Init() 以来由 seq 跳变得出的丢帧总数
 * @note   只反映"帧序号被跳过"；整段链路断连由 app_remote 的超时另行判断
 * @note   仅用于调试与链路质量评估，不参与控制
 */
uint32_t Remote_GetLostCount(void);

/**
 * @brief 取累计有效帧数
 * @param  无
 * @return 自 Remote_Init() 以来成功解析的帧数
 * @note   30 Hz 正常时每秒增量约 30，明显偏低即链路质量差
 */
uint32_t Remote_GetFrameCount(void);

#endif /* REMOTE_PROTOCOL_H */
