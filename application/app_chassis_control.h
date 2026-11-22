#ifndef APP_CHASSIS_CONTROL_H
#define APP_CHASSIS_CONTROL_H

#include "app_chassis.h"

// 本拍的控制结果，由底盘状态机决定是否进入故障。
typedef enum
{
    CHASSIS_CONTROL_NO_OUTPUT = 0,  // 等待首拍、禁用或其他原因，本拍无驱动输出。
    CHASSIS_CONTROL_OK,             // 已算出本拍四轮输出。
    CHASSIS_CONTROL_ENCODER_ERROR   // 允许运动时测速异常，已清零输出。
} ChassisControlResult;

void Chassis_Control_Init(void);
void Chassis_Control_SetTarget(ChassisSpeed target);
ChassisControlResult Chassis_Control_Update(const ChassisConfig *config,
                                            const ChassisEncoder *encoder,
                                            uint8_t enabled, float dt_s,
                                            ChassisOutput *output);

// 调用者先确认本拍控制成功，再检查斜坡是否归零、四轮是否低于或等于停稳阈值。
uint8_t Chassis_Control_IsStopped(float max_wheel_speed);

#endif
