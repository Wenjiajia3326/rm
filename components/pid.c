// PI控制实现：管理积分状态并计算控制输出。
#include "pid.h"
#include <stddef.h>

void Pid_Reset(PidData *pid)//重置
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
}
float Pid_Calc(PidData *pid, float target, float feedback, float dt_s)//pid计算
{
    float error;
    float output;

    if (pid == NULL || dt_s <= 0.0f)
    {
        return 0.0f;
    }

    if (pid->output_min > pid->output_max)
    {
        return 0.0f;
    }

    error = target - feedback;

    // 积分按经过的时间累积，而不是只按调用次数累积。
    pid->integral += pid->ki * error * dt_s;

    // 限制累计的I输出，避免积分无限增长。
    if (pid->integral > pid->output_max)
    {
        pid->integral = pid->output_max;
    }
    else if (pid->integral < pid->output_min)
    {
        pid->integral = pid->output_min;
    }

    output = pid->kp * error + pid->integral;

    if (output > pid->output_max)
    {
        output = pid->output_max;
    }
    else if (output < pid->output_min)
    {
        output = pid->output_min;
    }

    return output;
}