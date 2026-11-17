// 隧道任务接口：接收灯色，保存完全进入的结果。
#ifndef APP_TUNNEL_H
#define APP_TUNNEL_H
#include "app_task.h"
// 清除隧道进度、入内记录、灯色有效标志和计时。
void App_Tunnel_Reset(void);
// 开始隧道任务；路线参数预留，当前仅重置进度。
void App_Tunnel_Start(uint8_t route);
// 保存本周期灯色及有效标志；颜色编号沿用协议，有效为一、无效为零。
void App_Tunnel_SetLight(uint8_t color, uint8_t valid);
// 查询本次任务是否曾确认整车完全入内；是返回一，否则返回零，不代表已经出隧道。
uint8_t App_Tunnel_IsInside(void);
// 每周期检查灯色和现场结果，推进隧道流程；写入速度目标并返回任务进度，时间间隔单位秒。
TaskResult App_Tunnel_Update(const TrackInput *track,
                             ChassisSpeed *target, float dt_s);
#endif
