#include "app_chassis_wheels.h"
#include "wheel.hpp"

// 按底盘轮位顺序创建四个轮子对象，各自保存测速记录和 PI 状态。
static Wheel wheels[WHEEL_COUNT];

void Chassis_Wheel_ResetSpeed(uint8_t index)
{
    // index 是底盘内部轮位，调用时传对应的 WHEEL_*。
    wheels[index].ResetSpeed();
}

MotorSampleResult Chassis_Wheel_UpdateSpeed(uint8_t index, uint16_t count,
                                            uint32_t count_range,
                                            uint32_t counts_per_rev,
                                            float dt_s, float *speed)
{
    return wheels[index].UpdateSpeed(count, count_range, counts_per_rev, dt_s, speed);
}

int Chassis_Wheel_InitPI(uint8_t index, float kp, float ki,
                         float integral_max, float output_min, float output_max)
{
    return wheels[index].InitPI(kp, ki, integral_max, output_min, output_max);
}

float Chassis_Wheel_CalcOutput(uint8_t index, float target,
                               float feedback, float dt_s)
{
    return wheels[index].CalcOutput(target, feedback, dt_s);
}

void Chassis_Wheel_ResetPI(uint8_t index)
{
    wheels[index].ResetPI();
}
