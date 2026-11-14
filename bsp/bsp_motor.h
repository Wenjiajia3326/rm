// 声明底盘电机硬件接口，负责原始计数读取与驱动操作。
#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

/// @brief 电路板上的电机接口编号，不代表实际轮位。
typedef enum
{
    BSP_MOTOR_1 = 0,
    BSP_MOTOR_2,
    BSP_MOTOR_3,
    BSP_MOTOR_4,
    BSP_MOTOR_COUNT // 接口总数，不是有效电机编号。
} BSPMotorId;

/// @brief 启动四路编码器计数。
/// @return 全部启动成功返回1；任一路失败返回0。
/// @note CubeMX生成的四路编码器定时器初始化完成后调用。
///       在开始周期测速前调用一次，不启动电机PWM。
uint8_t BSP_Motor_Encoder_Init(void);

/// @brief 读取指定电机接口的编码器原始计数。
/// @param motor_id 电路板上的电机接口编号。
/// @param count 接收当前计数，不是转速；不能为NULL。
/// @return 成功返回1；编号或指针无效返回0，输出保持不变。
/// @note 编码器启动成功后，在普通任务中调用；不保留指针。
uint8_t BSP_Motor_Read_Count(BSPMotorId motor_id,
                            uint16_t *count);

/// @brief 设置指定电机接口的驱动请求。
/// @param motor_id 电路板上的电机接口编号。
/// @param duty 有符号驱动千分比，范围-1000～1000。
///             正负表示两个转动方向，0表示停止驱动请求。
/// @return 请求设置成功返回1；未初始化成功、编号或数值无效返回0。
/// @note 电机输出初始化成功后，在同一控制任务中调用。
///       不在中断中调用，其他任务或中断不得同时修改这些PWM通道。
///       临时屏蔽普通中断并直接更新CCR，可能改变本周期脉宽。
///       切换波形和电流须上板验证，不提供机械停稳或无毛刺保证。
///       设置成功不代表电机已经达到目标转速。
///       正方向与实际轮位的对应关系须上板确认。
///       零请求的电气状态须按驱动真值表实现，不代表瞬间停止。
uint8_t BSP_Motor_Set_Output(BSPMotorId motor_id,
                             int16_t duty);
/// @brief 从零输出启动四路电机PWM。
/// @return 全部启动成功返回1，任一路失败返回0。
/// @note GPIO、TIM1和TIM5初始化完成后，在普通任务中调用一次。
///       仅用于首次启动，不能作为运行期间的停止或重新初始化接口。
uint8_t BSP_Motor_Output_Init(void);

/// @brief 检查四路电机输出请求是否超时，超时通道设置为零输出。
/// @param timeout_ms 允许的最大更新间隔，单位ms，必须大于0。
/// @return 检查及归零操作成功返回1，包括发生超时；否则返回0。
/// @note 由同一控制任务周期调用，时间使用HAL_GetTick()。
///       达到超时即归零；新有效请求可恢复输出。
///       归零也刷新时间，持续无请求时每隔timeout_ms再次归零。
///       不能检测本任务卡死，也不代替遥控链路超时判断。
uint8_t BSP_Motor_Check_Timeout(uint32_t timeout_ms);

#endif
