// PI控制实现：管理积分状态并计算控制输出。
#include "pid.h"
#include <stddef.h>

int Pid_Init(PidData *pid, float kp, float ki, float integral_max,
             float output_min, float output_max)
{
    if (pid == NULL)
    {
        return 0;
    }
    // kp、ki 必须非负；integral_max 必须有意义；输出范围必须合法
    if (kp < 0.0f || ki < 0.0f || integral_max <= 0.0f || output_min >= output_max)
    {
        return 0;
    }

    pid->kp           = kp;
    pid->ki           = ki;
    pid->integral_max = integral_max;
    pid->output_min   = output_min;
    pid->output_max   = output_max;
    pid->integral     = 0.0f;

    return 1;
}

void Pid_Reset(PidData *pid)
{
    if (pid == NULL)
    {
        return;
    }
    pid->integral = 0.0f;
}

float Pid_Calc(PidData *pid, float target, float feedback, float dt_s)
{
    float error;
    float output;
    float integral_change;
    float integral_next;
    float output_candidate;

    if (pid == NULL || dt_s <= 0.0f)
    {
        return 0.0f;
    }

    error = target - feedback;

    integral_change = pid->ki * error * dt_s;
    integral_next = pid->integral + integral_change;

    // 积分自身的限幅独立于总输出范围。
    if (integral_next > pid->integral_max)
    {
        integral_next = pid->integral_max;
    }
    else if (integral_next < -pid->integral_max)
    {
        integral_next = -pid->integral_max;
    }

    output_candidate = pid->kp * error + integral_next;

    // 拒绝加重输出饱和的积分变化，允许反向积分帮助恢复。
    if (output_candidate > pid->output_max && integral_change > 0.0f)
    {
        integral_next = pid->integral;
    }
    else if (output_candidate < pid->output_min && integral_change < 0.0f)
    {
        integral_next = pid->integral;
    }

    pid->integral = integral_next;
    output = pid->kp * error + pid->integral;

    // 条件积分不代替最终输出限幅。
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
