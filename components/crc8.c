/**
 * @file    crc8.c
 * @brief   CRC-8/ATM 实现：多项式 0x07，初值 0x00，不反射，结果不异或。
 * @owner   电控B
 */

#include "crc8.h"

uint8_t Crc8_Update(uint8_t crc, uint8_t byte)
{
    uint8_t bit;

    crc ^= byte;

    for (bit = 0U; bit < 8U; bit++)
    {
        if ((crc & 0x80U) != 0U)
        {
            crc = (uint8_t)((crc << 1) ^ 0x07U);
        }
        else
        {
            crc = (uint8_t)(crc << 1);
        }
    }

    return crc;
}

uint8_t Crc8_Calc(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00U;
    uint16_t i;

    for (i = 0U; i < len; i++)
    {
        crc = Crc8_Update(crc, data[i]);
    }

    return crc;
}
