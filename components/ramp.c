// 目标斜坡计算：限制每次变化量，避免越过目标。
#include "ramp.h"

float Ramp_Update(float current, float target, float rate_limit, float dt_s)
{
    float max_change;
    float difference;

    if (rate_limit <= 0.0f || dt_s <= 0.0f)
    {
        return current;
    }

    max_change = rate_limit * dt_s;
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