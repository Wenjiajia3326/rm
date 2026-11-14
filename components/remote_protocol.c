/**
 * @file    remote_protocol.c
 * @brief   遥控帧解析：从 bsp_uart 缓冲就地扫描，不复制整帧。
 * @owner   电控B
 */

#include "remote_protocol.h"
#include "app_config.h"
#include "bsp_uart.h"
#include "crc8.h"

/* 协议字段偏移（与遥控帧协议 V1.3 表格一一对应） */
#define REMOTE_OFFSET_VER       2U
#define REMOTE_OFFSET_TYPE      3U
#define REMOTE_OFFSET_LEN       4U
#define REMOTE_OFFSET_VX        5U
#define REMOTE_OFFSET_VY        6U
#define REMOTE_OFFSET_BTN_LOW   7U
#define REMOTE_OFFSET_BTN_HIGH  8U
#define REMOTE_OFFSET_MODE      9U
#define REMOTE_OFFSET_TERRAIN   10U
#define REMOTE_OFFSET_LEVEL     11U
#define REMOTE_OFFSET_LOAD      12U
#define REMOTE_OFFSET_SEQ       13U
#define REMOTE_OFFSET_CRC       14U

/** 上一帧序号 */
static uint8_t s_last_seq;

/** 是否已建立 seq 基准 */
static uint8_t s_have_last_seq;

/** 累计丢帧数 */
static uint32_t s_lost_count;

/** 累计有效帧数 */
static uint32_t s_frame_count;

void Remote_Init(void)
{
    s_last_seq = 0U;
    s_have_last_seq = 0U;
    s_lost_count = 0U;
    s_frame_count = 0U;
}

/**
 * @brief 计算缓冲中当前候选帧的 CRC（覆盖 VER~seq）
 * @param  无
 * @return 计算出的 CRC
 * @note   数据在环形缓冲里不连续，逐字节用 Crc8_Update 累计
 */
static uint8_t Remote_CalcCrc(void)
{
    uint8_t crc = 0x00U;
    uint16_t i;

    for (i = REMOTE_OFFSET_VER; i < REMOTE_OFFSET_CRC; i++)
    {
        crc = Crc8_Update(crc, BSP_Uart_ReadByte(BSP_UART_REMOTE, i));
    }

    return crc;
}

/**
 * @brief 校验档位/模式字段是否合法
 * @param data 候选帧数据
 * @return 1 = 合法，0 = 越界
 * @note  手持端与主控档数配置不一致时这里会持续拒收，便于发现问题
 */
static uint8_t Remote_IsFieldValid(const RemoteData_t *data)
{
    if (data->mode >= APP_MODE_NUM)
    {
        return 0U;
    }

    if (data->terrain >= TERRAIN_NUM)
    {
        return 0U;
    }

    if (data->level >= LEVEL_NUM)
    {
        return 0U;
    }

    if (data->load > LOAD_CARRY_V)
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 用本帧 seq 更新丢帧统计
 * @param seq 本帧序号
 * @return 无
 * @note  delta 为 uint8 差值，天然处理 255→0 回绕；首帧只建立基准
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

uint8_t Remote_GetFrame(RemoteData_t *out)
{
    RemoteData_t data;

    while (BSP_Uart_GetLength(BSP_UART_REMOTE) >= APP_FRAME_LEN_REMOTE)
    {
        /* 帧头不对：丢 1 字节继续找 */
        if ((BSP_Uart_ReadByte(BSP_UART_REMOTE, 0U) != APP_FRAME_HEAD0) ||
            (BSP_Uart_ReadByte(BSP_UART_REMOTE, 1U) != APP_FRAME_HEAD1))
        {
            BSP_Uart_AddReadIndex(BSP_UART_REMOTE, 1U);
            continue;
        }

        /* 版本/类型/长度不对：丢 1 字节继续找 */
        if ((BSP_Uart_ReadByte(BSP_UART_REMOTE, REMOTE_OFFSET_VER) != APP_PROTOCOL_VER_RC) ||
            (BSP_Uart_ReadByte(BSP_UART_REMOTE, REMOTE_OFFSET_TYPE) != APP_FRAME_TYPE_REMOTE) ||
            (BSP_Uart_ReadByte(BSP_UART_REMOTE, REMOTE_OFFSET_LEN) != APP_FRAME_LEN_REMOTE_DATA))
        {
            BSP_Uart_AddReadIndex(BSP_UART_REMOTE, 1U);
            continue;
        }

        /* CRC 不对：丢 1 字节继续找 */
        if (Remote_CalcCrc() != BSP_Uart_ReadByte(BSP_UART_REMOTE, REMOTE_OFFSET_CRC))
        {
            BSP_Uart_AddReadIndex(BSP_UART_REMOTE, 1U);
            continue;
        }

        /* 有效帧：按偏移填入结构体 */
        data.vx       = (int8_t)BSP_Uart_ReadByte(BSP_UART_REMOTE, REMOTE_OFFSET_VX);
        data.vy       = (int8_t)BSP_Uart_ReadByte(BSP_UART_REMOTE, REMOTE_OFFSET_VY);
        data.btn_low  = BSP_Uart_ReadByte(BSP_UART_REMOTE, REMOTE_OFFSET_BTN_LOW);
        data.btn_high = BSP_Uart_ReadByte(BSP_UART_REMOTE, REMOTE_OFFSET_BTN_HIGH);
        data.mode     = BSP_Uart_ReadByte(BSP_UART_REMOTE, REMOTE_OFFSET_MODE);
        data.terrain  = BSP_Uart_ReadByte(BSP_UART_REMOTE, REMOTE_OFFSET_TERRAIN);
        data.level    = BSP_Uart_ReadByte(BSP_UART_REMOTE, REMOTE_OFFSET_LEVEL);
        data.load     = BSP_Uart_ReadByte(BSP_UART_REMOTE, REMOTE_OFFSET_LOAD);
        data.seq      = BSP_Uart_ReadByte(BSP_UART_REMOTE, REMOTE_OFFSET_SEQ);

        BSP_Uart_AddReadIndex(BSP_UART_REMOTE, APP_FRAME_LEN_REMOTE);

        if (Remote_IsFieldValid(&data) == 0U)
        {
            continue;
        }

        Remote_UpdateSeqStats(data.seq);
        s_frame_count++;
        *out = data;

        return 1U;
    }

    return 0U;
}

uint32_t Remote_GetLostCount(void)
{
    return s_lost_count;
}

uint32_t Remote_GetFrameCount(void)
{
    return s_frame_count;
}
