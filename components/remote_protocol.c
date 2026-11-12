/**
 * @file    remote_protocol.c
 * @brief   遥控帧解析：11 字节定长 + CRC8 校验，滑动窗口找帧头。
 * @owner   电控B
 *
 * 帧结构（偏移与协议表一致，便于对照）：
 *   [0] AA  [1] 55  [2] VER  [3] TYPE  [4] LEN
 *   [5] vx  [6] vy  [7] btn_low  [8] btn_high  [9] seq  [10] CRC8
 * CRC 覆盖 [2]~[9] 共 8 字节。
 *
 * 解析策略：不区分“找帧头/收数据”多个状态，只维护一个 11 字节滑动窗口；
 * 每来一个字节就进窗口，然后检查窗口前两字节是否为 AA 55：
 *   - 不是：丢掉窗口里最老的字节（整体左移一位），继续找；
 *   - 是  ：等收满 11 字节后校验版本/类型/长度/CRC。
 */

#include "remote_protocol.h"
#include "app_config.h"
#include "crc8.h"

/** 滑动窗口：始终保存最近收到的至多 11 个字节，偏移与协议表一一对应 */
static uint8_t s_frame[APP_FRAME_LEN_REMOTE];

/** 窗口内当前有效字节数，取值 0~11；等于 11 表示可以尝试校验整帧 */
static uint8_t s_len;

/** CRC 覆盖长度：VER~seq（不含帧头 AA 55，也不含 CRC 本身），即 11-3 = 8 字节 */
#define REMOTE_CRC_SPAN                 (APP_FRAME_LEN_REMOTE - 3U)

/**
 * @brief 复位解析器，清空滑动窗口
 *
 * 上电初始化或需要丢弃当前半帧、重新寻找帧头时调用。
 *
 * @param  无
 * @return 无
 *
 * @note   只清 s_len，不清 s_frame 内容（后续写入会覆盖，没必要多一次清零开销）
 * @note   可在中断中调用；不要在解析过程中被另一个上下文并发调用
 */
void Remote_Init(void)
{
    s_len = 0U;
}

/**
 * @brief 送入一个串口字节，尝试解析出一帧完整遥控数据
 *
 * 每调用一次只消耗一个字节：先把字节压入滑动窗口，再判断窗口前两字节是否为帧头；
 * 帧头不符时左移一位继续找，帧头相符且收满 11 字节时做版本/类型/长度与 CRC 校验。
 *
 * @param byte 刚从 UART 收到的原始字节
 * @param out  解析结果输出；**仅在返回 1 时被写入**，返回 0 时不修改其内容：
 *               - vx       : 前后，-100~100，+ 为前
 *               - vy       : 平移，-100~100，+ 为左
 *               - btn_low  : 低 8 键位图（RC_BTN_ARM_xxx / RC_BTN_ROT_xxx）
 *               - btn_high : 高 8 键位图（RC_BTN_MODE/OK/FINE_MODE/ESTOP/预留）
 *               - seq      : 帧序号 0~255，用于统计丢帧
 * @return 1 = 收到一帧完整且校验正确的数据（out 已填充）
 *         0 = 还没收齐，或帧头/版本/类型/长度/CRC 任意一项不符（该帧内容已丢弃，out 未动）
 *
 * @note   调用上下文：串口接收中断或串口接收任务，必须单线程调用，不可重入
 * @note   不阻塞；最坏情况每个字节做一次 10 字节的窗口左移，115200 波特率下可忽略
 * @note   解析失败只丢当前候选帧，不需要调用者清空接收缓冲，下一字节会自动重新同步
 */
uint8_t Remote_ParseByte(uint8_t byte, RemoteData_t *out)
{
    uint8_t i;

    if (out == 0)
    {
        return 0U;
    }

    /* 1. 新字节先进窗口；若计数器异常（理论上不会超过 11）则先复位，避免越界写 */
    if (s_len >= APP_FRAME_LEN_REMOTE)
    {
        s_len = 0U;
    }
    s_frame[s_len] = byte;
    s_len++;

    /* 2. 不够两个字节，无法判断帧头 */
    if (s_len < 2U)
    {
        return 0U;
    }

    /* 3. 帧头不对：丢掉最老的一个字节，整体左移，继续往后找 */
    if ((s_frame[0] != APP_FRAME_HEAD0) || (s_frame[1] != APP_FRAME_HEAD1))
    {
        for (i = 0U; i < (APP_FRAME_LEN_REMOTE - 1U); i++)
        {
            s_frame[i] = s_frame[i + 1U];
        }
        s_len--;
        return 0U;
    }

    /* 4. 帧头已对但还没收满：继续等 */
    if (s_len < APP_FRAME_LEN_REMOTE)
    {
        return 0U;
    }

    /* 5. 收满一帧：先查版本/类型/长度，再查 CRC；任一不符则整帧丢弃 */
    s_len = 0U;

    if ((s_frame[2] != APP_PROTOCOL_VER) ||
        (s_frame[3] != APP_FRAME_TYPE_REMOTE) ||
        (s_frame[4] != APP_FRAME_LEN_REMOTE_DATA))
    {
        return 0U;
    }

    if (Crc8_Calc(&s_frame[2], REMOTE_CRC_SPAN) != s_frame[APP_FRAME_LEN_REMOTE - 1U])
    {
        return 0U;
    }

    /* 6. 校验通过，按协议表偏移填充结果 */
    out->vx       = (int8_t)s_frame[5];
    out->vy       = (int8_t)s_frame[6];
    out->btn_low  = s_frame[7];
    out->btn_high = s_frame[8];
    out->seq      = s_frame[9];

    return 1U;
}
