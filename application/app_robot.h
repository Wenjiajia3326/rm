/**
 * @file    app_robot.h
 * @brief   整机调度入口：全局运行模式（比赛 / 机械臂调试 / 底盘调试）与后续阶段状态机。
 *
 * 上电默认停在 APP_MODE_INIT（选择态），需手柄 OK 确认后才进入某一模式。
 * 本模块只负责“模式与阶段”的决策，不直接操作硬件：
 * 底盘、舵机、串口等具体动作分别由 app_chassis / app_arm / app_vision 等模块执行。
 */

#ifndef APP_ROBOT_H
#define APP_ROBOT_H

#include <stdint.h>

/** 全局运行模式 */
typedef enum
{
    APP_MODE_INIT        = 0,  ///< 初始态（选择态）：未确认进入任何模式，执行器保持安全态
    APP_MODE_COMPETITION   = 1,  ///< 比赛模式：自动 60 s + 手动 240 s
    APP_MODE_ARM_DEBUG     = 2,  ///< 机械臂调试：机械B 与电控B 标定零位/限位用
    APP_MODE_CHASSIS_DEBUG = 3,  ///< 底盘调试：电控A 排查底盘与编码器用
    APP_MODE_NUM          = 4   ///< 模式数量（仅用于边界检查）
} AppMode_t;

/**
 * @brief 初始化整机调度模块，复位到初始选择态（APP_MODE_INIT）
 * @note  在调度器启动后、任务循环开始前调用一次
 */
void App_Robot_Init(void);

/**
 * @brief 周期调用（建议 10 ms），推进模式与后续阶段
 * @note  不阻塞；内部只做条件判断与状态迁移，不写硬件
 */
void App_Robot_Update(void);

/**
 * @brief 手柄 MODE 键：在 APP_MODE_INIT 态循环切换下一个模式
 * @note  已进入模式后忽略，避免比赛中误切
 */
void App_Robot_OnModeKey(void);

/**
 * @brief 手柄 OK 键：在 APP_MODE_INIT 态确认进入下一个模式
 * @return 1 = 已进入（或本来就已进入），0 = 当前安全条件不满足、未进入
 * @note  进入前会检查 App_Robot_IsSafeToSwitch()
 */
uint8_t App_Robot_OnOkKey(void);

/**
 * @brief 查询当前已进入的模式
 * @return 处于选择态时返回 APP_MODE_INIT
 */
AppMode_t App_Robot_GetMode(void);

/**
 * @brief 查询下一个模式（APP_MODE_INIT 态下由 MODE 键切换，OK 确认后进入）
 */
AppMode_t App_Robot_GetNextMode(void);

/**
 * @brief 查询是否已进入某个模式
 * @return 1 = 已进入，0 = 仍在选择态
 */
uint8_t App_Robot_IsEntered(void);

/**
 * @brief 取模式名，用于串口日志
 * @param mode 模式枚举
 * @return 指向静态字符串，调用者不需要释放
 */
const char *App_Robot_ModeName(AppMode_t mode);

#endif /* APP_ROBOT_H */
