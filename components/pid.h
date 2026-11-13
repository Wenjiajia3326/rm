// PI控制接口：根据目标值与反馈值计算控制输出。
#ifndef PID_H
#define PID_H

typedef struct
{
    float kp;
    float ki;
    float integral;
    float output_min;
    float output_max;
} PidData;

/// @brief 计算一次PI输出，并更新积分
/// @param pid 对应电机的PI数据，必须已经初始化
/// @param target 目标值
/// @param feedback 实际反馈，与target单位相同
/// @param dt_s 本次控制间隔，单位秒，必须大于0
/// @return 限制在output_min和output_max之间的控制输出
/// @note 每个电机使用独立的数据；同一份数据不能被并发修改。
///       函数不操作硬件，无阻塞操作。
float Pid_Calc(PidData *pid, float target, float feedback, float dt_s);

/// @brief 清除累计的积分，保留参数和输出限制
/// @param pid 需要复位的PI数据
void Pid_Reset(PidData *pid);

#endif