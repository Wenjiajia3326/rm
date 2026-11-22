#ifndef APP_CHASSIS_WHEELS_H
#define APP_CHASSIS_WHEELS_H

#include <stdint.h>
#include "motor.h"

// 内部轮序固定为左前、右前、左后、右后，不等同于电路板接口编号。
typedef enum
{
    WHEEL_FRONT_LEFT = 0,
    WHEEL_FRONT_RIGHT,
    WHEEL_REAR_LEFT,
    WHEEL_REAR_RIGHT,
    WHEEL_COUNT
} WheelId;

#ifdef __cplusplus
extern "C" {
#endif

// index 必须是有效轮位（左前、右前、左后或右后）。
// 重置指定轮位的测速记录，下一次采样重新记起点。
void Chassis_Wheel_ResetSpeed(uint8_t index);

// 更新指定轮位的测速记录，仅在轮速有效时写入 speed（rad/s）。
MotorSampleResult Chassis_Wheel_UpdateSpeed(uint8_t index, uint16_t count,
                                            uint32_t count_range,
                                            uint32_t counts_per_rev,
                                            float dt_s, float *speed);

// 配置指定轮位的速度 PI 并清积分，成功返回 1。
int Chassis_Wheel_InitPI(uint8_t index, float kp, float ki,
                         float integral_max, float output_min, float output_max);

// 用该轮自己的积分，根据目标轮速和实际轮速计算输出。
float Chassis_Wheel_CalcOutput(uint8_t index, float target,
                               float feedback, float dt_s);

// 只清指定轮位的 PI 积分，保留参数和测速记录。
void Chassis_Wheel_ResetPI(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif
