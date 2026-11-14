// 麦轮解算实现：计算四轮目标角速度，并提供统一限速。
#include "mecanum.h"
#include <stddef.h>
#include <math.h>

int Mecanum_Calc(const MecanumConfig *config,
                 const ChassisSpeed *target,
                 WheelSpeed *wheel_speed)
{
    float turn_speed;

    // 先检查地址，避免通过空指针读取或写入数据。
    if (config == NULL || target == NULL || wheel_speed == NULL)
    {
        return 0;
    }

    // 几何尺寸必须大于零。
    if (config->wheel_radius <= 0.0f ||
        config->half_length <= 0.0f ||
        config->half_width <= 0.0f)
    {
        return 0;
    }

    // 45度滚子、顶部滚子轴线俯视呈X、以四轮中心矩形中心为原点。
    // 实车轮位与电机方向仍需上板核对。
    turn_speed = (config->half_length + config->half_width) * target->wz;

    wheel_speed->front_left =
        (target->vx - target->vy - turn_speed) / config->wheel_radius;
    wheel_speed->front_right =
        (target->vx + target->vy + turn_speed) / config->wheel_radius;
    wheel_speed->rear_left =
        (target->vx + target->vy - turn_speed) / config->wheel_radius;
    wheel_speed->rear_right =
        (target->vx - target->vy + turn_speed) / config->wheel_radius;

    return 1;
}

int Mecanum_Limit(WheelSpeed *wheel_speed, float max_speed)
{
    float largest;
    float scale;

    if (wheel_speed == NULL || max_speed <= 0.0f)
    {
        return 0;
    }

    // 用绝对值同时检查正转和反转的速度大小。
    largest = fabsf(wheel_speed->front_left);
    if (fabsf(wheel_speed->front_right) > largest)
    {
        largest = fabsf(wheel_speed->front_right);
    }
    if (fabsf(wheel_speed->rear_left) > largest)
    {
        largest = fabsf(wheel_speed->rear_left);
    }
    if (fabsf(wheel_speed->rear_right) > largest)
    {
        largest = fabsf(wheel_speed->rear_right);
    }

    // 未超限直接返回，也避免全零轮速时发生除零。
    if (largest <= max_speed)
    {
        return 1;
    }

    // 保留四轮速度比例，统一降低目标。
    scale = max_speed / largest;
    wheel_speed->front_left *= scale;
    wheel_speed->front_right *= scale;
    wheel_speed->rear_left *= scale;
    wheel_speed->rear_right *= scale;

    return 1;
}
