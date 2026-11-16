// 负责将编码器计数变化量换算成轮子角速度。
#include "motor.h"
#include <stddef.h>

uint8_t Motor_Calc_Speed(int32_t delta_count,// 本次采样期间的有符号计数变化量，已处理计数回绕。
                         uint32_t counts_per_rev,// 轮子转一圈对应的计数，必须大于0。
                         float dt_s,// 两次采样的时间间隔，单位秒，必须大于0。
                         float *speed)// 接收计算结果，单位rad/s，不能是空指针。
{
    if (speed == NULL)
    {
        return 0;
    }

    if (counts_per_rev == 0 || dt_s <= 0.0f)
    {
        return 0;
    }

    // 先算转过的圈数，再换算成弧度，最后除以时间。
    *speed = ((float)delta_count / (float)counts_per_rev)
             * 6.2831853f / dt_s;

    return 1;
}
uint8_t Motor_Calc_Delta(uint16_t current,// 当前计数，必须小于count_range。
                        uint16_t previous,// 上次计数，必须小于count_range。
                        uint32_t count_range,// 计数范围，支持2～65536。
                        int32_t *delta_count)// 接收修正后的计数差，不能是空指针。
{
    int32_t difference;
    int32_t range;

    if (delta_count == NULL)
    {
        return 0;
    }

    // 范围必须有效，两个读数也必须在范围内。
    if (count_range < 2 || count_range > 65536 ||
        current >= count_range || previous >= count_range)
    {
        return 0;
    }

    range = (int32_t)count_range;
    difference = (int32_t)current - (int32_t)previous;

    // 恰好相差半个范围，无法判断方向。
    if (difference * 2 == range || difference * 2 == -range)
    {
        return 0;
    }

    if (difference * 2 > range)
    {
        // 例如2→97：95减去100，得到反转5格。
        difference -= range;
    }
    else if (difference * 2 < -range)
    {
        // 例如97→2：-95加上100，得到正转5格。
        difference += range;
    }

    *delta_count = difference;
    return 1;
}
uint8_t Motor_Update(MotorData *motor,
                     uint16_t current,
                     uint32_t count_range,
                     uint32_t counts_per_rev,
                     float dt_s,
                     float *speed)
{
    int32_t delta_count;

    if (motor == NULL)
    {
        return 0;
    }

    // 本次采样无效，取消旧起点，避免下次使用跨多个周期的计数差。
    if (speed == NULL ||
        count_range < 2 || count_range > 65536 ||
        current >= count_range ||
        counts_per_rev == 0 || dt_s <= 0.0f)
    {
        motor->initialized = 0;
        return 0;
    }

    // 首次只有一个读数，先建立起点，不生成轮速。
    if (motor->initialized == 0)
    {
        motor->previous_count = current;
        motor->initialized = 1;
        return 0;
    }

    // 先算计数差；只有成功时才换算轮速，任一步失败都重新建立起点。
    if (Motor_Calc_Delta(current,
                        motor->previous_count,
                        count_range,
                        &delta_count) == 0 ||
        Motor_Calc_Speed(delta_count,
                        counts_per_rev,
                        dt_s,
                        speed) == 0)
    {
        motor->initialized = 0;
        return 0;
    }

    motor->previous_count = current;
    return 1;
}