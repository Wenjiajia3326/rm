/**
 * @file    crc8.c
 * @brief   CRC-8/ATM 实现：多项式 0x07，初值 0x00，不反射，结果不异或。
 * @owner   电控B
 */

#include "crc8.h"

/**
 * @brief 计算一段数据的 CRC-8/ATM 校验值
 *
 * 逐个字节做异或后按位左移，最高位为 1 时异或多项式 0x07；处理 len 个字节后返回结果。
 *
 * @param data 待校验数据首地址；调用者保证 [data, data+len) 有效，不可为 NULL
 * @param len  待校验字节数；允许为 0，此时返回初值 0x00
 * @return     8 位校验值；同一段数据每次结果相同
 *
 * @note   不使用查表，执行时间与 len 成正比（len=8 时 64 次内层循环，72 MHz 下可忽略）
 * @note   无静态数据、可重入，可在中断中调用
 * @note   自检：对字符串 "123456789"（len=9）计算应得 0xF4
 */
uint8_t Crc8_Calc(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00U;  /* 初值 0x00，与协议约定的 CRC-8/ATM 一致 */
    uint16_t i;
    uint8_t b;

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (b = 0U; b < 8U; b++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}
