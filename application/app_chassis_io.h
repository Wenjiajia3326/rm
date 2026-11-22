#ifndef APP_CHASSIS_IO_H
#define APP_CHASSIS_IO_H

#include "app_chassis.h"

// 按轮位写入四路输出，全部写入成功才返回 1。
uint8_t App_Chassis_Write_Output(const ChassisMotorMap *map,
                                 const ChassisOutput *output);

// 逐个尝试将电路板的四路输出归零。
void Chassis_IO_TryZeroAll(void);

#endif
