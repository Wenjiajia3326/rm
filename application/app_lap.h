// 绕圈任务接口：启动只保存请求，Update每周期推进一次。
#ifndef APP_LAP_H
#define APP_LAP_H
#include "app_task.h"
// 清除绕圈进度、计时和出环记忆，并重置基础循迹。
void App_Lap_Reset(void);
// 按路线编号开始新的绕圈任务；只准备状态，后续周期函数负责推进。
void App_Lap_Start(uint8_t route);
// 每周期处理入环、绕行、出环和交接；写入速度目标并返回任务进度，时间间隔单位秒。
TaskResult App_Lap_Update(const TrackInput *track,
                          ChassisSpeed *target, float dt_s);
#endif
