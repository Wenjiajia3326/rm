#ifndef APP_CHASSIS_IO_H
#define APP_CHASSIS_IO_H

#include "app_chassis.h"

// 输入为车身方向PI输出；本接口执行一次输出反号、硬件正向补偿和限幅。
// BSP只执行PWM互补换算，调用者不要提前反号或补偿；全部写入成功返回1。
uint8_t App_Chassis_Write_Output(const ChassisMotorMap *map,
                                 const ChassisOutput *output);

// 逐个尝试将电路板的四路输出归零。
void Chassis_IO_TryZeroAll(void);

#endif
