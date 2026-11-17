#ifndef BSP_GRAY_H
#define BSP_GRAY_H

#include <stdint.h>

/// @brief 读取五路灰度输入电平。
/// @param level 保存GRAY_1到GRAY_5的电平，数组至少有5项。
///              1表示高电平，0表示低电平，不代表数据已通过硬件验证。
/// @return 完成读取返回1；空指针返回0。
uint8_t BSP_Gray_Read(uint8_t level[5]);

#endif
