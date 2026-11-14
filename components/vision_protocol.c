/**
 * @file    vision_protocol.c
 * @brief   视觉帧解析：从 bsp_uart 缓冲就地扫描，按 TYPE 区分帧长，不复制整帧。
 * @owner   电控B
 */

#include "vision_protocol.h"
#include "app_config.h"
#include "bsp_uart.h"
#include "crc8.h"

/* 帧内偏移 */
#define VISION_OFFSET_VER       2U
#define VISION_OFFSET_TYPE      3U
#define VISION_OFFSET_LEN       4U
#define VISION_OFFSET_DATA      5U

/* 帧结构长度 */
#define VISION_HEADER_SIZE      5U      /**< AA 55 VER TYPE LEN */
#define VISION_CRC_SIZE         1U      /**< 帧尾 CRC */

/* 数据段下标（相对 VISION_OFFSET_DATA，仅 0x10 / 0x11 使用） */
#define VISION_DATA_VALUE       0U
#define VISION_DATA_CONF        1U
#define VISION_DATA_TTL         2U
#define VISION_DATA_SEQ         3U

/* 心跳数据段下标（0x12 使用） */
#define VISION_HB_DATA_STATE    0U
#define VISION_HB_DATA_FPS      1U

/** 上一帧序号 */
static uint8_t s_last_seq;

/** 是否已建立 seq 基准 */
static uint8_t s_have_last_seq;

/**
 * @brief 读取视觉缓冲中相对读指针的第 index 个字节
 * @param index 相对偏移
 * @return 该位置字节
 * @note   BSP_Uart_ReadByte 的短别名，避免每行都写通道宏
 */
static uint8_t Vision_ReadByte(uint16_t index)
{
    return BSP_Uart_ReadByte(BSP_UART_VISION, index);
}

void Vision_Init(void)
{
    s_last_seq = 0U;
    s_have_last_seq = 0U;
}

void Vision_ResetSeqExpectation(void)
{
    s_have_last_seq = 0U;
}

/**
 * @brief 按帧类型取整帧长度
 * @param type 帧类型字段
 * @return 整帧字节数；未知类型返回 0
 */
static uint8_t Vision_GetFrameLength(uint8_t type)
{
    switch (type)
    {
        case APP_FRAME_TYPE_VISION_ROUTE:
        case APP_FRAME_TYPE_VISION_LIGHT:
            return APP_FRAME_LEN_VISION_ROUTE;

        case APP_FRAME_TYPE_VISION_HB:
            return APP_FRAME_LEN_VISION_HB;

        default:
            return 0U;
    }
}

/**
 * @brief 计算缓冲中当前候选帧的 CRC（覆盖 VER~DATA）
 * @param  无（数据段长度直接从缓冲的 LEN 字段读取）
 * @return 计算出的 CRC 值，供与帧尾 CRC 比较
 */
static uint8_t Vision_GetCalcCrc(void)
{
    uint8_t crc = 0x00U;
    uint16_t span = (uint16_t)(VISION_HEADER_SIZE + Vision_ReadByte(VISION_OFFSET_LEN));
    uint16_t i;

    for (i = VISION_OFFSET_VER; i < span; i++)
    {
        crc = Crc8_Update(crc, Vision_ReadByte(i));
    }

    return crc;
}

/**
 * @brief 判断业务帧的 seq 是否接受（去重）
 * @param seq 本帧序号
 * @return 1 = 接受；0 = 重复帧，应丢弃
 * @note  视觉端重启导致的 seq 复位由 Vision_ResetSeqExpectation() 处理
 */
static uint8_t Vision_SeqJudge(uint8_t seq)
{
    uint8_t delta;

    if (s_have_last_seq == 0U)
    {
        s_have_last_seq = 1U;
        s_last_seq = seq;
        return 1U;
    }

    delta = (uint8_t)(seq - s_last_seq);

    if (delta == 0U)
    {
        return 0U;
    }

    s_last_seq = seq;

    return 1U;
}

uint8_t Vision_GetFrame(VisionFrame_t *out)
{
    uint8_t type;
    uint8_t frame_length;

    while (BSP_Uart_GetAvailableDataSize(BSP_UART_VISION) >= VISION_HEADER_SIZE)
    {
        VisionFrame_t frame = {0};

        /* 帧头不对：丢 1 字节继续找 */
        if ((Vision_ReadByte(0U) != APP_FRAME_HEAD0) ||
            (Vision_ReadByte(1U) != APP_FRAME_HEAD1))
        {
            BSP_Uart_AddReadIndex(BSP_UART_VISION, 1U);
            continue;
        }

        /* 版本不对：丢 1 字节继续找 */
        if (Vision_ReadByte(VISION_OFFSET_VER) != APP_PROTOCOL_VER)
        {
            BSP_Uart_AddReadIndex(BSP_UART_VISION, 1U);
            continue;
        }

        type = Vision_ReadByte(VISION_OFFSET_TYPE);
        frame_length = Vision_GetFrameLength(type);

        /* 未知类型：丢 1 字节继续找 */
        if (frame_length == 0U)
        {
            BSP_Uart_AddReadIndex(BSP_UART_VISION, 1U);
            continue;
        }

        /* LEN 与类型不符：丢 1 字节继续找 */
        if (Vision_ReadByte(VISION_OFFSET_LEN) !=
            (uint8_t)(frame_length - VISION_HEADER_SIZE - VISION_CRC_SIZE))
        {
            BSP_Uart_AddReadIndex(BSP_UART_VISION, 1U);
            continue;
        }

        /* 还没收满：等下一次 */
        if (BSP_Uart_GetAvailableDataSize(BSP_UART_VISION) < frame_length)
        {
            return 0U;
        }

        /* CRC 不对：丢 1 字节继续找 */
        if (Vision_GetCalcCrc() != Vision_ReadByte((uint16_t)(frame_length - 1U)))
        {
            BSP_Uart_AddReadIndex(BSP_UART_VISION, 1U);
            continue;
        }

        /* 有效帧：按类型填字段 */
        frame.vision_type = type;

        if (type == APP_FRAME_TYPE_VISION_HB)
        {
            frame.vision_state = Vision_ReadByte(VISION_OFFSET_DATA + VISION_HB_DATA_STATE);
            frame.vision_fps = Vision_ReadByte(VISION_OFFSET_DATA + VISION_HB_DATA_FPS);
        }
        else
        {
            frame.vision_conf = Vision_ReadByte(VISION_OFFSET_DATA + VISION_DATA_CONF);
            frame.vision_ttl_50ms = Vision_ReadByte(VISION_OFFSET_DATA + VISION_DATA_TTL);
            frame.vision_seq = Vision_ReadByte(VISION_OFFSET_DATA + VISION_DATA_SEQ);

            if (type == APP_FRAME_TYPE_VISION_ROUTE)
            {
                frame.vision_route = Vision_ReadByte(VISION_OFFSET_DATA + VISION_DATA_VALUE);
            }
            else
            {
                frame.vision_light = Vision_ReadByte(VISION_OFFSET_DATA + VISION_DATA_VALUE);
            }
        }

        BSP_Uart_AddReadIndex(BSP_UART_VISION, frame_length);

        /* 心跳没有 seq，不做去重 */
        if (type == APP_FRAME_TYPE_VISION_HB)
        {
            *out = frame;
            return 1U;
        }

        /* 业务帧：重复 seq 丢弃后继续找下一帧 */
        if (Vision_SeqJudge(frame.vision_seq) == 0U)
        {
            continue;
        }

        *out = frame;
        return 1U;
    }

    return 0U;
}
