#include "wheel.hpp"

void Wheel::ResetPI()
{
    // 只清这个轮子的积分，保留 PI 参数。
    Pid_Reset(&pid_);
}

void Wheel::ResetSpeed()
{
    // 标记旧计数不能再用，下次采样重新记起点。
    motor_.initialized = 0U;
}

MotorSampleResult Wheel::UpdateSpeed(uint16_t count,
                                    uint32_t count_range,
                                    uint32_t counts_per_rev,
                                    float dt_s,
                                    float *speed)
{
    // 把这个轮子的测速记录交给原来的测速函数。
    return Motor_Update(&motor_, count, count_range,
                        counts_per_rev, dt_s, speed);
}

int Wheel::InitPI(float kp, float ki, float integral_max,
                  float output_min, float output_max)
{
    // 设置这个轮子的 PI 参数，返回原函数的初始化结果。
    return Pid_Init(&pid_, kp, ki, integral_max, output_min, output_max);
}

float Wheel::CalcOutput(float target, float feedback, float dt_s)
{
    // 用这个轮子自己的积分，根据目标轮速和实际轮速算输出。
    return Pid_Calc(&pid_, target, feedback, dt_s);
}