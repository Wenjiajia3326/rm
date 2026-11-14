// 负责编码器计数与轮速之间的换算。
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

// 保存单个电机测速时需要记住的数据。
typedef struct
{
    uint16_t previous_count;  ///< 上一次读取的编码器计数。
    uint8_t initialized;     ///< 0表示尚未建立起点，1表示已有上次计数。
} MotorData;

/// @brief 将编码器计数变化量换算成轮子角速度。
/// @param delta_count 本次采样期间的有符号计数变化量，已处理计数回绕。
/// @param counts_per_rev 轮子转一圈对应的计数，必须大于0。
/// @param dt_s 两次采样的时间间隔，单位秒，必须大于0。
/// @param speed 接收计算结果，单位rad/s，不能是空指针。
/// @return 成功返回1；参数无效返回0，并保持原结果不变。
/// @note 调用者保证浮点输入为有限值；计数正方向应与轮速正方向一致。
uint8_t Motor_Calc_Speed(int32_t delta_count,
                         uint32_t counts_per_rev,
                         float dt_s,
                         float *speed);


/// @brief 计算两次计数之间的有符号差值，处理计数回绕。
/// @param current 当前计数，必须小于count_range。
/// @param previous 上次计数，必须小于count_range。
/// @param count_range 计数范围，支持2～65536。
/// @param delta_count 接收修正后的计数差，不能是空指针。
/// @return 成功返回1；参数无效或恰好相差半个范围时返回0，且不修改原结果。
/// @note 两次采样之间，实际计数变化的绝对值必须小于计数范围的一半。
uint8_t Motor_Calc_Delta(uint16_t current,// 当前计数，必须小于count_range。
                        uint16_t previous,// 上次计数，必须小于count_range。
                        uint32_t count_range,// 计数范围，支持2～65536。
                        int32_t *delta_count);// 接收修正后的计数差，不能是空指针。

/// @brief 根据当前计数更新单个电机的轮速。
/// @param motor 单个电机的测速记录，首次使用前须清零。
/// @param current 当前编码器计数，必须小于count_range。
/// @param count_range 计数范围，支持2～65536。
/// @param counts_per_rev 轮子转一圈对应的计数，必须大于0。
/// @param dt_s 当前与上一次采样的间隔，单位秒，必须大于0。
/// @param speed 接收轮速，单位rad/s，不能是空指针。
/// @return 得到有效轮速返回1；首次建立起点或失败返回0，不修改speed。
/// @note 浮点输入须为有限值；实际计数变化须小于计数范围的一半。
/// @note 固定采样使用；漏采样或采样失败后须清零initialized，重新建立起点。
/// @note 同一记录只由一个执行上下文更新；更改计数配置前须重新建立起点。
uint8_t Motor_Update(MotorData *motor,
                     uint16_t current,
                     uint32_t count_range,
                     uint32_t counts_per_rev,
                     float dt_s,
                     float *speed);

#endif