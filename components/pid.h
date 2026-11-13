// PI控制接口：根据目标值与反馈值计算控制输出。
#ifndef PID_H
#define PID_H

typedef struct
{
    float kp;
    float ki;
    float integral;
    float integral_max; ///< I输出的绝对值上限，单位与控制输出相同
    float output_min;
    float output_max;
} PidData;

/// @brief 设置PI参数、输出范围，并清零积分
/// @param pid 需要初始化的PI数据，不能为NULL
/// @param kp 比例系数，必须 >= 0
/// @param ki 积分系数，必须 >= 0
/// @param integral_max 积分绝对值上限，必须 > 0；积分限幅范围为[-integral_max, +integral_max]
/// @param output_min 输出下限，必须 < output_max
/// @param output_max 输出上限，必须 > output_min
/// @return 1表示成功；0表示参数非法（NULL、负系数、范围无效），原数据保持不变
/// @note 调用者保证所有数值参数有限；本函数只检查上述大小关系。
///       初始化期间不能同时在其他地方计算或修改同一份数据
int Pid_Init(PidData *pid, float kp, float ki, float integral_max,
             float output_min, float output_max);

/// @brief 计算一次PI输出，并更新积分
/// @param pid 对应电机的PI数据，必须已经初始化
/// @param target 目标值
/// @param feedback 实际反馈，与target单位相同
/// @param dt_s 本次控制间隔，单位秒，必须 > 0
/// @return 限制在[output_min, output_max]之间的控制输出
/// @note target、feedback、dt_s必须为有限数值；每个电机使用独立的PidData。
///       同一份数据不能并发修改；非阻塞，不操作硬件。
float Pid_Calc(PidData *pid, float target, float feedback, float dt_s);

/// @brief 清除累计的积分，保留参数和输出限制
/// @param pid 需要复位的PI数据，允许NULL（无操作）
void Pid_Reset(PidData *pid);

#endif
