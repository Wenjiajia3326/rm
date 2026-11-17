// 第一段路线与第二段断线的周期流程接口。
#ifndef APP_LINE_TASK_H
#define APP_LINE_TASK_H
#include "app_task.h"
// 清除循迹段、路线动作和断线记忆，并重置基础循迹。
void App_Line_Task_Reset(void);
// 开始一段循迹：段编号为一或二，路线编号为一至三；只保存请求，不驱动电机。
void App_Line_Task_Start(uint8_t segment, uint8_t route);
// 每周期推进一次循迹任务：读取现场结果，写入整车速度目标，返回任务进度；时间间隔单位秒。
TaskResult App_Line_Task_Update(const TrackInput *track,
                                ChassisSpeed *target, float dt_s);
#endif
