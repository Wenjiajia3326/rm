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

/// @brief 设置PI参数、输出范围，并清零积分
/// @param pid 需要初始化的PI数据
/// @param kp 比例系数，必须非负，单位为输出单位/误差单位
/// @param ki 积分系数，必须非负，单位为输出单位/(误差单位*秒)
/// @param output_min 输出下限，必须小于等于0
/// @param output_max 输出上限，必须大于等于0，与积分和返回输出单位一致
/// @return 1表示成功；0表示参数错误，原数据保持不变
/// @note 数值参数必须有限，下限必须小于上限。
///       初始化期间不能同时计算或修改同一份数据。
int Pid_Init(PidData *pid, float kp, float ki,
             float output_min, float output_max);
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