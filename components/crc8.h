/**
 * @file    crc8.h
 * @brief   CRC-8/ATM（多项式 0x07，初值 0x00），视觉/遥控/舵机帧共用。
 *
 * 三处协议的校验范围一致：从 VER 开始到数据段结尾，不含帧头 AA 55。
 */

#ifndef CRC8_H
#define CRC8_H

#include <stdint.h>

/**
 * @brief 计算 CRC-8/ATM
 * @param data 数据首地址
 * @param len  字节数
 * @return 8 位校验值
 * @note  自检：对字符串 "123456789" 计算应得 0xF4
 * @note  覆盖范围按各协议定义（视觉/遥控均为 VER~PAYLOAD，不含帧头 AA 55）
 */
uint8_t Crc8_Calc(const uint8_t *data, uint16_t len);

#endif /* CRC8_H */
