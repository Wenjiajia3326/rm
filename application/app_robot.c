/**
 * @file    app_robot.c
 * @brief   整机状态实现：帧值驱动的模式/档位状态 + 模式进入退出动作。
 * @owner   电控B
 */

#include "app_robot.h"
#include "app_config.h"
#include "bsp_log.h"

/** 当前模式（APP_MODE_*_V） */
static uint8_t s_mode;

/** 当前地形档（TERRAIN_*_V） */
static uint8_t s_terrain;

/** 当前放置高度档索引 */
static uint8_t s_level;

/** 当前载荷状态（LOAD_*_V） */
static uint8_t s_load;

/**
 * @brief 请求所有执行器进入安全状态
 * @param  无
 * @return 无
 * @note   接入执行器模块后在此实现：底盘斜坡停车、机构停止动作并保持当前位置
 * @note   切模式、链路丢失、急停都走这一个出口，避免多处写停车逻辑
 */
static void App_Robot_RequestSafeState(void)
{
    /* TODO: App_Chassis_RequestStop(); App_Arm_RequestHold(); */
}

/**
 * @brief 进入某个模式时执行的动作
 * @param mode 目标模式
 * @return 无
 */
static void App_Robot_EnterMode(uint8_t mode)
{
    switch (mode)
    {
        case APP_MODE_INIT_V:
            /* 初始态：不写任何执行器目标，保持安全 */
            break;

        case APP_MODE_COMP_AUTO_V:
            /* TODO: 自动流程复位到 A01（进入即开始） */
            break;

        case APP_MODE_COMP_MANUAL_V:
            /* 手动阶段：摇杆输入立即生效 */
            break;

        case APP_MODE_ARM_DEBUG_V:
            /* TODO: 只允许机构动作，底盘禁行 */
            break;

        case APP_MODE_CHASSIS_DEBUG_V:
            /* TODO: 只允许底盘动作，机构保持 */
            break;

        default:
            break;
    }

    LOG("mode -> %s (terrain=%u level=%u load=%u)\r\n",
        App_Robot_ModeName(mode), s_terrain, s_level, s_load);
}

/**
 * @brief 离开某个模式时执行的动作
 * @param mode 即将离开的模式
 * @return 无
 * @note  目前所有模式共用"请求安全态"；将来某模式有专属退出动作时再加 case
 */
static void App_Robot_ExitMode(uint8_t mode)
{
    (void)mode;

    App_Robot_RequestSafeState();
}

void App_Robot_Init(void)
{
    s_mode    = APP_MODE_INIT_V;
    s_terrain = TERRAIN_ROAD_V;
    s_level   = 0U;
    s_load    = LOAD_EMPTY_V;
}

void App_Robot_Update(void)
{
    switch (s_mode)
    {
        case APP_MODE_COMP_AUTO_V:
            /* TODO: 将来改为 App_Auto_Update();  自动流程 A01~A07 推进 */
            break;

        case APP_MODE_COMP_MANUAL_V:
            /* 手动阶段由按键事件驱动，暂不需要周期处理 */
            break;

        case APP_MODE_ARM_DEBUG_V:
            /* TODO: 将来改为 App_Arm_Debug_Update(); 单关节微调 */
            break;

        case APP_MODE_CHASSIS_DEBUG_V:
            /* 底盘调试由电控A 侧处理 */
            break;

        default:
            break;
    }
}

void App_Robot_ApplyMode(uint8_t mode)
{
    if (mode == s_mode)
    {
        return;
    }

    App_Robot_ExitMode(s_mode);
    s_mode = mode;
    App_Robot_EnterMode(s_mode);
}

void App_Robot_OnLinkLost(void)
{
    if (s_mode == APP_MODE_INIT_V)
    {
        return;
    }

    App_Robot_ExitMode(s_mode);
    s_mode = APP_MODE_INIT_V;

    LOG("link lost -> INIT\r\n");
}

uint8_t App_Robot_GetMode(void)
{
    return s_mode;
}

void App_Robot_SetTerrain(uint8_t terrain)
{
    s_terrain = terrain;
}

uint8_t App_Robot_GetTerrain(void)
{
    return s_terrain;
}

void App_Robot_SetLevel(uint8_t level)
{
    s_level = level;
}

uint8_t App_Robot_GetLevel(void)
{
    return s_level;
}

void App_Robot_SetLoad(uint8_t load)
{
    s_load = load;
}

uint8_t App_Robot_GetLoad(void)
{
    return s_load;
}

const char *App_Robot_ModeName(uint8_t mode)
{
    switch (mode)
    {
        case APP_MODE_INIT_V:            return "INIT";
        case APP_MODE_COMP_AUTO_V:       return "COMP_AUTO";
        case APP_MODE_COMP_MANUAL_V:     return "COMP_MANUAL";
        case APP_MODE_ARM_DEBUG_V:       return "ARM_DEBUG";
        case APP_MODE_CHASSIS_DEBUG_V:   return "CHASSIS_DEBUG";
        default:                         return "UNKNOWN";
    }
}
