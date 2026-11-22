#ifndef WHEEL_HPP
#define WHEEL_HPP

#include "motor.h"
#include "pid.h"

// 管理一个轮子的测速记录和速度 PI 状态。
class Wheel
{
public:
    // 更新测速。只有本次轮速有效时，才写入 speed。
    MotorSampleResult UpdateSpeed(uint16_t count,
                                  uint32_t count_range,
                                  uint32_t counts_per_rev,
                                  float dt_s,
                                  float *speed);

    // 设置这个轮子的 PI 参数，并清零积分。成功返回 1。
    int InitPI(float kp, float ki, float integral_max,
               float output_min, float output_max);

    // PI 已初始化、反馈有效时，计算这个轮子的控制输出。
    float CalcOutput(float target, float feedback, float dt_s);

    // 丢掉测速起点，下次采样重新建立起点。
    void ResetSpeed();

    // 只清 PI 积分，保留 PI 参数和测速记录。
    void ResetPI();

private:
    MotorData motor_{};  // 上一次编码器计数，以及是否已有采样起点。
    PidData pid_{};      // 当前轮子的 PI 参数和累计积分。
};                      // 类定义结束后要有分号。

#endif