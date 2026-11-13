// 麦轮解算接口：将底盘速度目标转换为四轮目标速度。
#ifndef MECANUM_H
#define MECANUM_H

/// @brief 车身坐标系中的速度目标
typedef struct
{
    float vx;  ///< 前后速度，向前为正，单位m/s
    float vy;  ///< 左右速度，向左为正，单位m/s
    float wz;  ///< 转动角速度，俯视逆时针为正，单位rad/s
} ChassisSpeed;
/// @brief 四轮目标角速度，正方向统一为推动整车前进的方向
typedef struct
{
    float front_left;   ///< 左前轮，单位rad/s
    float front_right;  ///< 右前轮，单位rad/s
    float rear_left;    ///< 左后轮，单位rad/s
    float rear_right;   ///< 右后轮，单位rad/s
} WheelSpeed;
/// @brief 四轮中心呈矩形分布、以矩形中心为原点时的几何参数
typedef struct
{
    float wheel_radius; ///< 轮半径，单位m
    float half_length;  ///< 前后轮中心距离的一半，单位m
    float half_width;   ///< 左右轮中心距离的一半，单位m
} MecanumConfig;
/// @brief 根据底盘速度目标计算四轮目标角速度
/// @note 适用于45度滚子、顶部滚子轴线俯视呈X的矩形底盘，绕矩形中心转动。
/// @param config 几何配置，只读；各尺寸必须大于0，单位m
/// @param target 底盘速度目标，只读；平移单位m/s，转动单位rad/s
/// @param wheel_speed 保存四轮计算结果，单位rad/s
/// @return 1表示成功；0表示指针为空或尺寸不大于0，输出保持不变
/// @note 所有输入数值必须有限，由调用者保证。
///       输出对象不能被并发访问；函数不操作硬件，无阻塞操作。
int Mecanum_Calc(const MecanumConfig *config,
                 const ChassisSpeed *target,
                 WheelSpeed *wheel_speed);

/// @brief 四轮目标超速时，按相同比例缩小
/// @param wheel_speed 四轮目标，直接修改，单位rad/s
/// @param max_speed 轮速绝对值上限，必须大于0，单位rad/s
/// @return 1表示成功，包括无需缩放；0表示参数无效，数据不变
/// @note 所有输入数值必须有限，由调用者保证；同一份数据不能并发访问。
int Mecanum_Limit(WheelSpeed *wheel_speed, float max_speed);

#endif