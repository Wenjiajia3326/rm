#ifndef APP_LINE_H
#define APP_LINE_H

#include <stdint.h>
#include "mecanum.h"

// 循迹模块交给上层的处理结果，不是电机输出值。
typedef enum
{
    LINE_NO_TARGET = 0,  // 暂时没有可用目标：本周期给零请求，后续仍可继续处理。
    LINE_TARGET_READY,  // 已生成本周期的速度请求。
    LINE_FAILED,        // 恢复最终失败：交给底盘记录故障，不再继续本次任务。
    LINE_FEATURE_DONE   // 本次特征处理完成一次，同时给出有效循迹目标；不是整段任务完成。
} LineResult;

// 当前这一次特征的处理要求，不是整条路线的编号。
typedef enum
{
    LINE_ACTION_WAIT = 0,  // 遇到特征后等待明确动作，正常循迹仍可计算。
    LINE_ACTION_STRAIGHT,  // 沿主线直行。
    LINE_ACTION_LEFT,      // 在合适位置左转。
    LINE_ACTION_RIGHT,     // 在合适位置右转。
    LINE_ACTION_FOLLOW     // 连续沿线走，不将缓弯的外侧触发当成路口动作。
} LineAction;
/// @brief 清除循迹内部的连续计数、特征记忆和观察计时。
/// @note 初始化、新任务、禁用或离开循迹时调用；不操作电机。
void App_Line_Reset(void);

/// @brief 读取灰度、分析图案，并按正常、观察或转弯阶段生成速度请求。
/// @param target 保存速度请求；不能为NULL，失败时给出零请求。
/// @param dt_s 距离上次更新经过的时间，单位为秒，必须为有限正数。
/// @param action 当前这一次特征的处理要求；不是收到左转就立即转弯。
/// @return LINE_TARGET_READY表示有目标；LINE_NO_TARGET表示暂时无目标；LINE_FAILED表示最终失败。
///         LINE_FEATURE_DONE仅在本次特征处理完成时返回一次，同时给出有效速度目标，可继续行驶。
/// @note 必须按枚举值判断，不能用“非零就是成功”：LINE_FAILED也是非零值。
///       生成目标不代表任务完成；调用者负责运动许可。特殊路段流程尚未完成。
///       正常循迹连续全白进入有限时找线，超时保持LINE_FAILED直到Reset。
///       无方向记录时保持零请求等待；计划断线尚未接入，不按找线成功消费动作。
///       同一RobotTask周期调用，不保留指针，不直接操作电机。
LineResult App_Line_Update(ChassisSpeed *target,
                           float dt_s,
                           LineAction action);

#endif