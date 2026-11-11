/**
 * @file    app_robot.c
 * @brief   全局模式状态机：初始选择态（APP_MODE_INIT）+ 比赛 / 机械臂调试 / 底盘调试。
 *
 * 设计要点：
 * 1) 上电先进入 APP_MODE_INIT（选择态），不自动进比赛模式，避免上电即动作；
 * 2) 选择态里 MODE 只改候选、OK 才真正进入，进入前检查安全条件；
 * 3) 进入任一模式后不再接受模式切换，必须断电重启
 *    （比赛中误触 MODE 会直接毁掉整局，宁可要求重启）。
 */

#include "app_robot.h"

/** 下一个模式（OK 确认后进入）：初值比赛模式，在 APP_MODE_INIT 态由 MODE 键循环切换 */
static AppMode_t s_next_mode = APP_MODE_COMPETITION;

/** 当前已进入的模式；APP_MODE_INIT 表示还没进入 */
static AppMode_t s_mode = APP_MODE_INIT;

/**
 * @brief 判断当前是否允许进入/切换模式
 * @return 1 = 允许，0 = 不允许
 * @note  目前恒为允许。底盘与机构接入后，这里要补：
 *        - 底盘目标速度为 0，且实际速度接近 0
 *        - 机构处于收纳/安全姿态
 */
static uint8_t App_Robot_IsSafeToSwitch(void)
{
    return 1U;
}

void App_Robot_Init(void)
{
    s_mode = APP_MODE_INIT;
    s_next_mode = APP_MODE_COMPETITION;
}

void App_Robot_Update(void)
{
    switch (s_mode)
    {
        case APP_MODE_INIT:
            /* 初始选择态：不做任何动作，等待 MODE/OK；执行器应保持安全态 */
            break;

        case APP_MODE_COMPETITION:
            /* 比赛模式：后续在此推进 待命 -> 自动 -> 结算冻结 -> 手动 */
            break;

        case APP_MODE_ARM_DEBUG:
            /* 机械臂调试：后续在此处理单关节微调 */
            break;

        case APP_MODE_CHASSIS_DEBUG:
            /* 底盘调试：后续在此处理底盘单独调试 */
            break;

        default:
            /* 枚举异常时退回 APP_MODE_INIT，避免停在未定义分支 */
            s_mode = APP_MODE_INIT;
            break;
    }
}

void App_Robot_OnModeKey(void)
{
    if (s_mode != APP_MODE_INIT)
    {
        return;
    }

    /* APP_MODE_INIT 态内循环：比赛 -> 机械臂调试 -> 底盘调试 -> 比赛 */
    if (s_next_mode >= APP_MODE_CHASSIS_DEBUG)
    {
        s_next_mode = APP_MODE_COMPETITION;
    }
    else
    {
        s_next_mode = (AppMode_t)((uint32_t)s_next_mode + 1U);
    }
}

uint8_t App_Robot_OnOkKey(void)
{
    if (s_mode != APP_MODE_INIT)
    {
        /* 已进入模式：重复确认视为成功，避免按键连击产生歧义 */
        return 1U;
    }

    if (App_Robot_IsSafeToSwitch() == 0U)
    {
        return 0U;
    }

    s_mode = s_next_mode;
    return 1U;
}

AppMode_t App_Robot_GetMode(void)
{
    return s_mode;
}

AppMode_t App_Robot_GetNextMode(void)
{
    return s_next_mode;
}

uint8_t App_Robot_IsEntered(void)
{
    return (s_mode != APP_MODE_INIT) ? 1U : 0U;
}

const char *App_Robot_ModeName(AppMode_t mode)
{
    switch (mode)
    {
        case APP_MODE_INIT:
            return "INIT";
        case APP_MODE_COMPETITION:
            return "COMPETITION";
        case APP_MODE_ARM_DEBUG:
            return "ARM_DEBUG";
        case APP_MODE_CHASSIS_DEBUG:
            return "CHASSIS_DEBUG";
        default:
            return "UNKNOWN";
    }
}
