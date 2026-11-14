// 声明底盘目标接收接口。
#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include <stdint.h>
#include "mecanum.h"

/// @brief 底盘目标变化的加减速限制。
typedef struct
{
    float linear_accel;   ///< 前后、横移加速度限制，单位m/s²，必须大于0。
    float linear_decel;   ///< 前后、横移减速度限制，单位m/s²，必须大于0。
    float angular_accel;  ///< 旋转角加速度限制，单位rad/s²，必须大于0。
    float angular_decel;  ///< 旋转角减速度限制，单位rad/s²，必须大于0。
} ChassisRampConfig;
/// @brief 同一次测速采样取得的四轮编码器计数。
typedef struct
{
    uint16_t front_left;   ///< 左前轮计数。
    uint16_t front_right;  ///< 右前轮计数。
    uint16_t rear_left;    ///< 左后轮计数。
    uint16_t rear_right;   ///< 右后轮计数。
} ChassisEncoder;
/// @brief 四轮驱动请求，单位为驱动千分比，正方向与轮速目标一致。
typedef struct
{
    float front_left;
    float front_right;
    float rear_left;
    float rear_right;
} ChassisOutput;
/// @brief 一次底盘控制更新所需的配置。
typedef struct
{
    ChassisRampConfig ramp;    ///< 前后、横移和旋转的加减速限制。
    MecanumConfig mecanum;     ///< 轮半径和前后、左右轮中心距离的一半。
    float max_wheel_speed;    ///< 四轮角速度绝对值上限，单位rad/s。
    uint32_t count_range;     ///< 编码器计数范围，即ARR加1。
    uint32_t counts_per_rev;  ///< 轮子转一圈的最终计数，已计入必要的倍频与减速比。
} ChassisConfig;

/// @brief 将最终目标和平滑目标清零。
/// @note 开始接收目标前调用；不操作硬件，不代表电机已停止。
void App_Chassis_Init(void);

/// @brief 保存底盘速度目标。
/// @param target 车身速度；vx、vy单位m/s，wz单位rad/s。
/// @return 成功返回1；空指针返回0，原目标不变。
/// @note 输入数值须为有限值，正方向沿用ChassisSpeed定义。
///       复制目标内容，不保留传入指针；当前仅支持同一主循环顺序调用。
///       本函数不执行限速、控制计算或硬件输出。
uint8_t App_Chassis_Set_Target(const ChassisSpeed *target);

/// @brief 更新一次底盘平滑速度目标。
/// @param config 加减速限制，只读；函数不保留此指针。
/// @param dt_s 距上次更新经过的时间，单位s，必须大于0。
/// @return 成功返回1；参数无效返回0，平滑目标不变。
/// @note 所有输入数值必须有限；先调用初始化函数。
///       当前仅支持同一主循环顺序调用，不操作硬件。
///       前后与横移分别限制变化率，不限制合成平移加速度。
uint8_t App_Chassis_Update_Ramp(const ChassisRampConfig *config,
                               float dt_s);
/// @brief 将当前平滑目标解算为四轮目标，并统一限速。
/// @param config 底盘几何配置，只读；尺寸单位m，不保留指针。
/// @param max_speed 四轮角速度绝对值上限，单位rad/s，必须大于0。
/// @return 成功返回1；参数无效返回0，原四轮目标不变。
/// @note 所有输入数值必须有限；先初始化，再更新斜坡。
///       当前仅支持同一主循环顺序调用，不操作硬件。
uint8_t App_Chassis_Update_Wheel(const MecanumConfig *config,
                                float max_speed);

/// @brief 根据四轮当前计数更新实际轮速。
/// @param encoder 本次四轮计数，只读，不保留指针；NULL表示本次采样失败。
/// @param count_range 四轮共同的计数范围，即ARR加1，支持2～65536。
/// @param counts_per_rev 四轮共同的每轮一圈最终计数，已计入必要的倍频与减速比。
/// @param dt_s 两次采样间隔，单位s，必须大于0且有限。
/// @return 四轮反馈全部有效返回1；首次采样或任一失败返回0。
/// @note 先调用App_Chassis_Init；本接口仅用于四轮计数配置相同的情况。
///       失败时有效标志置0，保留旧轮速，旧值不可当成本次反馈使用。
///       固定周期顺序调用；漏采样时以NULL通知本函数，重新建立起点。
///       更改计数配置前须重新初始化；每次真实计数变化须小于计数范围的一半。
///       轮速符号沿用计数方向，接入PI前须统一为推动整车前进时为正。
///       仅由同一主循环顺序调用；不操作硬件，也不自动检测反馈超时。
uint8_t App_Chassis_Update_Feedback(const ChassisEncoder *encoder,
                                   uint32_t count_range,
                                   uint32_t counts_per_rev,
                                   float dt_s);

/// @brief 用同一组参数初始化四轮PI，四轮积分分别保存。
/// @param kp 比例系数，必须大于等于0；输入采用rad/s，输出采用驱动千分比。
/// @param ki 积分系数，必须大于等于0；与上述单位及秒制积分匹配。
/// @param integral_max 积分输出绝对值上限，单位为驱动千分比，必须大于0。
/// @param output_min 驱动请求下限，单位为驱动千分比，必须小于output_max。
/// @param output_max 驱动请求上限，单位为驱动千分比，须与后续BSP允许范围一致。
/// @return 四轮全部初始化成功返回1，否则返回0并禁止PI计算。
/// @note 所有参数必须有限；先调用App_Chassis_Init。
///       仅在停止控制计算时配置，成功配置会清空四轮积分。
///       本函数不操作硬件，不自动停止电机；调用者须先确保停止输出。
uint8_t App_Chassis_Init_PI(float kp,
                           float ki,
                           float integral_max,
                           float output_min,
                           float output_max);

/// @brief 根据四轮目标与反馈，计算一次四轮PI驱动请求。
/// @param dt_s 本次控制间隔，单位s，必须大于0且有限。
/// @param output 接收四轮驱动请求，不能为NULL。
/// @return 成功返回1；条件不满足返回0。
/// @note 先初始化PI，并确保本周期目标解算和测速成功。
///       反馈无效、PI未就绪或时间无效时，输出清零、清除四轮积分并取消反馈资格。
///       空输出指针直接返回0；每份有效反馈最多用于一次本函数计算。
///       数值须有限；仅在同一主循环顺序调用，不操作硬件。
uint8_t App_Chassis_Calc_Output(float dt_s,
                               ChassisOutput *output);

/// @brief 清除底盘运动目标、积分和驱动请求。
/// @param output 接收全零驱动请求；允许NULL，此时仅复位内部状态。
/// @note 保留PI参数和就绪状态，取消当前反馈的使用资格。
///       不操作硬件，不代表电机已停止；调用者须执行零输出。
///       停止期间须继续周期测速；漏采样须通知测速接口重新建立起点。
///       不锁存禁止状态；恢复前须由上层管理使能，防止旧命令被再次写入。
///       仅在同一主循环顺序调用。
void App_Chassis_Stop(ChassisOutput *output);

/// @brief 执行一次底盘测速、目标处理和四轮PI计算。
/// @param config 底盘配置，只读，不保留指针。
/// @param encoder 本次四轮计数；NULL表示采样失败。
/// @param enabled 0表示禁止运行，非0表示允许运行。
/// @param dt_s 采样及控制间隔，单位s，必须大于0且有限。
/// @param output 接收四轮驱动请求，不能为NULL。
/// @return 完成控制计算返回1；禁止运行或任一步失败返回0。
/// @note 先初始化底盘和PI；所有输入数值必须有限。
///       停止后须重新设置目标；首次采样只建立计数起点，返回0。
///       固定周期在同一主循环顺序调用，停止期间也须调用。
///       更改计数配置前须重新初始化。
///       调用者每次都须执行返回的驱动请求，包括失败时的零请求。
///       本函数不操作硬件，不检测命令超时。
uint8_t App_Chassis_Update(const ChassisConfig *config,
                           const ChassisEncoder *encoder,
                           uint8_t enabled,
                           float dt_s,
                           ChassisOutput *output);

#endif