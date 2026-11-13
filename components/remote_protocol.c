/**
 * @file    remote_protocol.c
 * @brief   遥控帧解析实现：15 字节定长 + CRC8 + seq 丢帧统计。
 * @owner   电控B
 *
 * 解析策略：只维护一个 15 字节滑动窗口。
 * 每来一个字节就压入窗口尾部；若窗口前两字节不是帧头，则丢掉最老字节整体左移；
 * 帧头正确且收满 15 字节后，依次校验版本/类型/长度/CRC/字段范围。
 */

#include "remote_protocol.h"
#include "app_config.h"
#include "crc8.h"

/** 滑动窗口：始终保存最近收到的至多 15 个字节，偏移与协议表一一对应 */
static uint8_t s_frame[APP_FRAME_LEN_REMOTE];

/** 窗口内有效字节数（0~15），等于 15 时表示可以尝试校验整帧 */
static uint8_t s_len;

/** 是否已建立 seq 基准（首帧只建立基准，不参与丢帧计算） */
static uint8_t s_have_last_seq;

/** 上一帧的 seq */
static uint8_t s_last_seq;

/** 累计丢帧数（按 seq 跳变估算） */
static uint32_t s_lost_count;

/** 累计有效帧数 */
static uint32_t s_frame_count;

/**
 * @brief 复位解析器与统计量
 *
 * 把窗口长度、seq 基准、两个计数清零。缓冲区内容不清（后续写入会覆盖）。
 *
 * @param  无
 * @return 无
 *
 * @note   上电调用一次；重复调用会清零丢帧/收帧统计
 */
void Remote_Init(void)
{
    s_len = 0U;
    s_have_last_seq = 0U;
    s_last_seq = 0U;
    s_lost_count = 0U;
    s_frame_count = 0U;
}

/**
 * @brief 校验解析出的档位/模式字段是否在合法范围
 *
 * @param d 已通过 CRC 校验的解析结果（不可为 NULL）
 * @return 1 = 所有字段合法，0 = 存在越界字段
 *
 * @note   数量取自 app_config.h；手持端与主控配置不一致时，这里会持续拒收
 * @note   纯判断，无副作用
 */
static uint8_t Remote_IsFieldValid(const RemoteData_t *d)
{
    if (d->mode >= APP_MODE_NUM)
    {
        return 0U;
    }

    if (d->terrain >= TERRAIN_NUM)
    {
        return 0U;
    }

    if (d->level >= LEVEL_NUM)
    {
        return 0U;
    }

    if (d->load > LOAD_CARRY_V)
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 用本帧 seq 更新丢帧统计
 *
 * @param seq 本帧序号（0~255）
 * @return 无
 *
 * @note   delta = (uint8_t)(seq - last_seq)，天然处理 255->0 回绕：
 *         delta==0 视为重复帧（不记丢帧）、delta==1 正常、delta>1 记 (delta-1) 个丢帧
 * @note   首帧只建立基准，不计丢帧
 */
static void Remote_UpdateSeqStats(uint8_t seq)
{
    uint8_t delta;

    if (s_have_last_seq == 0U)
    {
        s_have_last_seq = 1U;
        s_last_seq = seq;
        return;
    }

    delta = (uint8_t)(seq - s_last_seq);

    if (delta > 1U)
    {
        s_lost_count += (uint32_t)(delta - 1U);
    }

    s_last_seq = seq;
}

/**
 * @brief 送入一个串口字节，尝试解析出一帧完整遥控数据
 *
 * @param byte 串口收到的原始字节
 * @param out  解析结果；仅返回 1 时写入，返回 0 时保持原内容
 * @return 1 = 成功解析一帧（out 已填充）；0 = 未收齐或校验失败
 *
 * @note   单线程调用（串口接收任务），不可重入
 * @note   最坏情况每字节做一次 14 字节窗口左移，115200 下可忽略
 */
uint8_t Remote_ParseByte(uint8_t byte, RemoteData_t *out)
{
    uint8_t i;
    RemoteData_t d;

    if (out == 0)
    {
        return 0U;
    }

    /* 1. 新字节进窗口；防御性复位，避免越界写 */
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

    /* 3. 帧头不符：丢掉最老的一个字节，整体左移，继续找 */
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

    /* 5. 收满一帧：先查版本/类型/长度，再查 CRC */
    s_len = 0U;

    if ((s_frame[2] != APP_PROTOCOL_VER_RC) ||
        (s_frame[3] != APP_FRAME_TYPE_REMOTE) ||
        (s_frame[4] != APP_FRAME_LEN_REMOTE_DATA))
    {
        return 0U;
    }

    if (Crc8_Calc(&s_frame[2], APP_FRAME_RC_CRC_SPAN) != s_frame[APP_FRAME_LEN_REMOTE - 1U])
    {
        return 0U;
    }

    /* 6. 按协议偏移填充结果 */
    d.vx       = (int8_t)s_frame[5];
    d.vy       = (int8_t)s_frame[6];
    d.btn_low  = s_frame[7];
    d.btn_high = s_frame[8];
    d.mode     = s_frame[9];
    d.terrain  = s_frame[10];
    d.level    = s_frame[11];
    d.load     = s_frame[12];
    d.seq      = s_frame[13];

    /* 7. 字段越界（手持端与主控档数配置不一致）直接丢弃 */
    if (Remote_IsFieldValid(&d) == 0U)
    {
        return 0U;
    }

    /* 8. 有效帧：更新统计并交出结果 */
    Remote_UpdateSeqStats(d.seq);
    s_frame_count++;
    *out = d;

    return 1U;
}

/**
 * @brief 取累计丢帧数
 * @param  无
 * @return 由 seq 跳变估算的丢帧总数
 * @note   仅用于诊断，不参与控制
 */
uint32_t Remote_GetLostCount(void)
{
    return s_lost_count;
}

/**
 * @brief 取累计有效帧数
 * @param  无
 * @return 成功解析的帧数
 * @note   30 Hz 下每秒增量约 30
 */
uint32_t Remote_GetFrameCount(void)
{
    return s_frame_count;
}
