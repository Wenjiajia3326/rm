/**
 * @file    crc8.h
 * @brief   CRC-8/ATM（多项式 0x07，初值 0x00），各协议帧共用。
 * @owner   电控B
 */

#ifndef CRC8_H
#define CRC8_H

#include <stdint.h>

/**
 * @brief 推进一个字节的 CRC 计算
 * @param crc  上一次的计算结果（首次调用传 0）
 * @param byte 本次参与计算的字节
 * @return 更新后的 CRC 值
 * @note   用于数据不连续存放（如环形缓冲）时逐字节累计
 */
uint8_t Crc8_Update(uint8_t crc, uint8_t byte);

/**
 * @brief 计算一段连续数据的 CRC-8/ATM
 * @param data 数据首地址，len 字节内有效
 * @param len  字节数（可为 0，返回初值 0x00）
 * @return 8 位校验值
 * @note   自检：对 "123456789"（len=9）应得 0xF4
 */
uint8_t Crc8_Calc(const uint8_t *data, uint16_t len);

#endif /* CRC8_H */
