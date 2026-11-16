/**
 * @file    bsp_uart.c
 * @brief   串口缓冲实现：DMA 循环单缓冲 / 逐字节中断，统一读写索引接口。
 * @owner   电控B
 *
 * DMA 通道：写索引不存变量，实时用 (buffer_size - CNDTR) 算出；
 * 中断通道：写索引在接收中断里推进。两种通道对外接口完全一致。
 */

#include "bsp_uart.h"
#include "bsp_config.h"

/** 单路通道状态 */
typedef struct
{
    UART_HandleTypeDef *huart;          /**< HAL 句柄 */
    DMA_HandleTypeDef  *hdma;           /**< 接收 DMA 句柄；0 表示逐字节中断 */
    uint8_t            *buffer;         /**< 缓冲首地址 */
    uint16_t            buffer_size;    /**< 缓冲大小（字节） */
    uint16_t            read_index;     /**< 读索引：只有任务侧修改 */
    volatile uint16_t   write_index;    /**< 写索引：只有中断通道使用 */
    uint8_t             it_byte;        /**< 逐字节中断的接收单元 */
} BspUartChannel_t;

/** 视觉口缓冲（DMA 直接写） */
static uint8_t s_buffer_vision[BSP_UART_DMA_BUF_SIZE];

/** 遥控口缓冲（DMA 直接写） */
static uint8_t s_buffer_remote[BSP_UART_DMA_BUF_SIZE];

/** 舵机口缓冲（中断写） */
static uint8_t s_buffer_servo[BSP_UART_IT_BUF_SIZE];

/** 三路通道 */
static BspUartChannel_t s_channel[BSP_UART_NUM];

/**
 * @brief 计算通道当前的写索引
 * @param channel 通道
 * @return 写索引（0 ~ buffer_size-1）
 * @note  DMA 通道：buffer_size - CNDTR（DMA 剩余计数）；中断通道：write_index
 */
static uint16_t BSP_Uart_CalcWriteIndex(const BspUartChannel_t *channel)
{
    if (channel->hdma != 0)
    {
        return (uint16_t)(channel->buffer_size - __HAL_DMA_GET_COUNTER(channel->hdma));
    }

    return channel->write_index;
}

/**
 * @brief 中断通道写入一个字节
 * @param channel 通道（中断通道）
 * @param byte    待写入字节
 * @return 无
 * @note  缓冲满时丢弃本字节：保住"单写者"约束，避免中断去改 read_index
 * @note  仅在接收中断里调用
 */
static void BSP_Uart_PushByte(BspUartChannel_t *channel, uint8_t byte)
{
    uint16_t next = (uint16_t)((channel->write_index + 1U) % channel->buffer_size);

    if (next == channel->read_index)
    {
        return;
    }

    channel->buffer[channel->write_index] = byte;
    channel->write_index = next;
}

void BSP_Uart_Init(void)
{
    s_channel[BSP_UART_VISION].huart       = &huart3;
    s_channel[BSP_UART_VISION].hdma        = &hdma_usart3_rx;
    s_channel[BSP_UART_VISION].buffer      = s_buffer_vision;
    s_channel[BSP_UART_VISION].buffer_size = BSP_UART_DMA_BUF_SIZE;
    s_channel[BSP_UART_VISION].read_index  = 0U;
    s_channel[BSP_UART_VISION].write_index = 0U;

    s_channel[BSP_UART_REMOTE].huart       = &huart4;
    s_channel[BSP_UART_REMOTE].hdma        = &hdma_uart4_rx;
    s_channel[BSP_UART_REMOTE].buffer      = s_buffer_remote;
    s_channel[BSP_UART_REMOTE].buffer_size = BSP_UART_DMA_BUF_SIZE;
    s_channel[BSP_UART_REMOTE].read_index  = 0U;
    s_channel[BSP_UART_REMOTE].write_index = 0U;

    s_channel[BSP_UART_SERVO].huart        = &huart5;
    s_channel[BSP_UART_SERVO].hdma         = 0;
    s_channel[BSP_UART_SERVO].buffer       = s_buffer_servo;
    s_channel[BSP_UART_SERVO].buffer_size  = BSP_UART_IT_BUF_SIZE;
    s_channel[BSP_UART_SERVO].read_index   = 0U;
    s_channel[BSP_UART_SERVO].write_index  = 0U;

    /* DMA 通道：启动循环接收后不再干预；中断通道：启动 1 字节中断接收 */
    (void)HAL_UART_Receive_DMA(&huart3, s_buffer_vision, BSP_UART_DMA_BUF_SIZE);
    (void)HAL_UART_Receive_DMA(&huart4, s_buffer_remote, BSP_UART_DMA_BUF_SIZE);
    (void)HAL_UART_Receive_IT(&huart5, &s_channel[BSP_UART_SERVO].it_byte, 1U);
}

uint16_t BSP_Uart_GetAvailableDataSize(uint8_t port)
{
    const BspUartChannel_t *channel = &s_channel[port];

    return (uint16_t)((BSP_Uart_CalcWriteIndex(channel) + channel->buffer_size - channel->read_index)
                      % channel->buffer_size);
}

uint16_t BSP_Uart_GetFreeSpace(uint8_t port)
{
    return (uint16_t)(s_channel[port].buffer_size - BSP_Uart_GetAvailableDataSize(port));
}

uint8_t BSP_Uart_ReadByte(uint8_t port, uint16_t index)
{
    uint16_t position = (uint16_t)((s_channel[port].read_index + index) % s_channel[port].buffer_size);

    return s_channel[port].buffer[position];
}

void BSP_Uart_AddReadIndex(uint8_t port, uint16_t length)
{
    s_channel[port].read_index =
        (uint16_t)((s_channel[port].read_index + length) % s_channel[port].buffer_size);
}

void BSP_Uart_SendData(uint8_t port, const uint8_t *data, uint16_t length)
{
    (void)HAL_UART_Transmit(s_channel[port].huart, (uint8_t *)data, length, 10U);
}

/**
 * @brief 重启 DMA 接收（UART 错误后的恢复）
 *
 * @param port 通道编号
 * @return 无
 *
 * @note   重启前必须把读索引清零：HAL_UART_Receive_DMA 会把 DMA 的 CNDTR 复位成
 *         buffer_size，写索引随之回到 0；若读索引还停在原处，新旧数据会错位。
 *         丢掉的旧数据不用在意，协议层靠帧头 + CRC 重新同步。
 * @note   先 Abort 再 Receive：错误后 HAL 的接收状态可能不是 READY，直接 Receive 会返回 BUSY。
 * @note   仅中断上下文调用（HAL_UART_ErrorCallback）。
 */
static void BSP_Uart_RestartDmaRx(uint8_t port)
{
    BspUartChannel_t *channel = &s_channel[port];

    channel->read_index = 0U;

    (void)HAL_UART_AbortReceive(channel->huart);
    (void)HAL_UART_Receive_DMA(channel->huart, channel->buffer, channel->buffer_size);
}

/**
 * @brief HAL 接收完成回调（覆盖弱符号）
 * @param huart 触发的串口句柄
 * @return 无
 * @note  回调是所有串口共用的，必须按实例区分；只有舵机口（UART5）走这里
 * @note  中断上下文：禁止阻塞调用
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        BSP_Uart_PushByte(&s_channel[BSP_UART_SERVO], s_channel[BSP_UART_SERVO].it_byte);
        (void)HAL_UART_Receive_IT(huart, &s_channel[BSP_UART_SERVO].it_byte, 1U);
    }
}

/**
 * @brief HAL 串口错误回调（覆盖弱符号）
 *
 * @param huart 触发的串口句柄
 * @return 无
 *
 * @note  串口溢出（ORE）、帧错误等会中止接收：
 *        - 舵机口是逐字节中断 → 重新启动 1 字节接收；
 *        - 视觉/遥控是 DMA 循环接收 → 重新启动 DMA（否则该口永久静默）。
 * @note  中断上下文：这里不做任何阻塞操作，也不打日志（日志阻塞约 7 ms）。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        (void)HAL_UART_Receive_IT(huart, &s_channel[BSP_UART_SERVO].it_byte, 1U);
        return;
    }

    if (huart->Instance == USART3)
    {
        BSP_Uart_RestartDmaRx(BSP_UART_VISION);
        return;
    }

    if (huart->Instance == UART4)
    {
        BSP_Uart_RestartDmaRx(BSP_UART_REMOTE);
        return;
    }
}
