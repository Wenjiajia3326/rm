// 目标斜坡计算：分别限制加减速，反向目标先减速到零。
#include "ramp.h"

float Ramp_Update(float current, float target,
                  float accel_limit, float decel_limit, float dt_s)
{
    float rate;
    float max_change;
    float difference;

    if (accel_limit <= 0.0f || decel_limit <= 0.0f || dt_s <= 0.0f)
    {
        return current;
    }

    // 反向时本次只向零靠近；到零后，下次再向反方向加速。
    // 不使用本周期减速后剩余的时间，避免跨零时混用两种限制。
    if ((current > 0.0f && target < 0.0f) ||
        (current < 0.0f && target > 0.0f))
    {
        target = 0.0f;
    }

    rate = decel_limit;
    if ((target > 0.0f && target > current) ||
        (target < 0.0f && target < current))
    {
        rate = accel_limit;
    }

    max_change = rate * dt_s;
    difference = target - current;

    if (difference > max_change)
    {
        return current + max_change;
    }
    if (difference < -max_change)
    {
        return current - max_change;
    }
    return target;
}