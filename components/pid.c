// PI控制实现：管理积分状态并计算控制输出。
#include "pid.h"
#include <stddef.h>

int Pid_Init(PidData *pid, float kp, float ki,
             float output_min, float output_max)//初始化
{
    // 参数有效性由调用者保证，这里只检查空指针。
    if (pid == NULL)
    {
        return 0;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral = 0.0f;

    return 1;
}
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
    float integral_change;//本次积分预改变
    float integral_next;//新算的积分值
    float output_candidate;//用新积分算的新总输出

    if (pid == NULL || dt_s <= 0.0f)
    {
        return 0.0f;
    }

    if (pid->output_min > pid->output_max)
    {
        return 0.0f;
    }

    error = target - feedback;
    integral_change = pid->ki * error * dt_s;
    integral_next = pid->integral + integral_change;

    // 先限制试算积分，避免累计值超出允许范围。
    if (integral_next > pid->output_max)
    {
        integral_next = pid->output_max;
    }
    else if (integral_next < pid->output_min)
    {
        integral_next = pid->output_min;
    }

    output_candidate = pid->kp * error + integral_next;

    // 只阻止将输出进一步推向饱和的积分，允许反向积分帮助恢复。
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