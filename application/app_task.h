// 各任务共用的流程返回值，与电机驱动输出无关。
// 这些返回值说明任务进度，不是电机占空比或驱动指令。
#ifndef APP_TASK_H
#define APP_TASK_H
#include "app_track.h"
typedef enum
{
    TASK_WAIT = 0,  // 正常等待，给零速度目标；已有运动许可时仍执行减速和速度闭环。
    TASK_MOVE,      // 本周期已有速度请求。
    TASK_BOUNDARY,  // 到达流程边界，仍继续运动等待交接。
    TASK_FINISHED,  // 到达停车点，由底盘确认停稳。
    TASK_UNSET,     // 判据或参数尚未配置；额外禁止运动输出，不能当作正常等待。
    TASK_FAILED     // 本次任务失败。
} TaskResult;

// 把基础循迹结果转换成任务结果；完成一次路口动作后仍继续行驶，不代表整个任务完成。
static inline TaskResult App_Task_FromLine(LineResult result)
{
    switch (result)
    {
        case LINE_TARGET_READY:
        case LINE_FEATURE_DONE:
            return TASK_MOVE;
        case LINE_FAILED:
            return TASK_FAILED;
        default:
            return TASK_WAIT;
    }
}
#endif
