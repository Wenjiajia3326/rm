/**
 * @file    app_auto.c
 * @brief   自动阶段状态机实现：锁路 → 循迹1 → 绕圈 → 等灯 → 过隧道 → 循迹2 → 建筑区放块
 * @owner   电控B
 *
 * 开局已携初始块并处于收纳态，一路带到建筑区才放，所以没有"搬运初始块"这一步。
 * 超时值待实车实测；规则自动阶段共 60 s。
 */

#include "app_auto.h"
#include "app_config.h"
#include "app_vision.h"
#include "app_chassis.h"
#include "app_arm.h"
#include "app_robot.h"
#include "main.h"

#define AUTO_TIMEOUT_LINE_MS        25000U
#define AUTO_TIMEOUT_LAP_MS         12000U
#define AUTO_TIMEOUT_LIGHT_MS       20000U
#define AUTO_TIMEOUT_TUNNEL_MS      10000U
#define AUTO_TIMEOUT_ARM_MS          8000U

/** 首见黄灯后这段时间内仍允许驶入（黄灯共 2 s）；数值待实测 */
#define AUTO_YELLOW_ENTER_MS         1000U

/** 当前步骤 */
static AutoStep_t s_step;

/** 进入当前步骤的时刻 */
static uint32_t s_step_tick;

/** 锁定的路线（1~3） */
static uint8_t s_route;

/** 上一周期的灯色，用于识别跳变 */
static uint8_t s_last_light;

/** 1 = 本次黄灯是从"非黄"跳过来的，计时起点可信 */
static uint8_t s_yellow_ok;

/** 首见黄灯的时刻 */
static uint32_t s_yellow_tick;

const char *App_Auto_Get_StepName(AutoStep_t step)
{
    switch (step)
    {
        case AUTO_STEP_INIT:   return "INIT";
        case AUTO_STEP_LINE1:  return "LINE1";
        case AUTO_STEP_LAP:    return "LAP";
        case AUTO_STEP_LIGHT:  return "LIGHT";
        case AUTO_STEP_TUNNEL: return "TUNNEL";
        case AUTO_STEP_LINE2:  return "LINE2";
        case AUTO_STEP_PLACE:  return "PLACE";
        case AUTO_STEP_DONE:   return "DONE";
        case AUTO_STEP_FAIL:   return "FAIL";
    }

    return "UNKNOWN";
}

/**
 * @brief 停掉当前步骤占用的底盘任务
 * @param  无
 * @return 无
 */
static void App_Auto_StopCurrent(void)
{
    switch (s_step)
    {
        case AUTO_STEP_LINE1:  App_Chassis_StopLine1();   break;
        case AUTO_STEP_LAP:    App_Chassis_StopLap();     break;
        case AUTO_STEP_TUNNEL: App_Chassis_RequestStop(); break;
        case AUTO_STEP_LINE2:  App_Chassis_StopLine2();   break;
        default: break;
    }
}

/**
 * @brief 流程失败：停车并停在 FAIL
 * @param why 失败原因（日志用）
 * @return 无
 */
static void App_Auto_Fail(const char *why)
{
    App_Auto_StopCurrent();
    App_Chassis_RequestStop();
    s_step = AUTO_STEP_FAIL;
    LOG("auto fail: %s\r\n", why);
}

/**
 * @brief 进入某个步骤，必要时发起对应的底盘任务
 * @param step 目标步骤
 * @return 无
 * @note   启动接口返回 0（状态不对、路线无效）时直接判失败，不干等超时
 */
static void App_Auto_EnterStep(AutoStep_t step)
{
    s_step = step;
    s_step_tick = HAL_GetTick();

    /* 按枚举顺序排列；INIT 是起始状态，不是"进入"出来的，所以没有 case */
    switch (step)
    {
        case AUTO_STEP_LINE1:
            if (App_Chassis_StartLine1(s_route) == 0U)
            {
                App_Auto_Fail("start line1");
                return;
            }
            break;

        case AUTO_STEP_LAP:
            if (App_Chassis_StartLap(s_route) == 0U)
            {
                App_Auto_Fail("start lap");
                return;
            }
            break;

        case AUTO_STEP_LIGHT:
            s_yellow_ok = 0U;
            break;

        case AUTO_STEP_TUNNEL:
            if (App_Chassis_PassTunnel(s_route) == 0U)
            {
                App_Auto_Fail("start tunnel");
                return;
            }
            break;

        case AUTO_STEP_LINE2:
            if (App_Chassis_StartLine2(s_route) == 0U)
            {
                App_Auto_Fail("start line2");
                return;
            }
            break;

        case AUTO_STEP_PLACE:
            App_Arm_Set_Action(ARM_ACT_PLACE);

            /* 动作没启动（载荷不对或臂正忙）就当场判失败，不干等超时 */
            if (App_Arm_IsBusy() == 0U)
            {
                App_Auto_Fail("place rejected");
                return;
            }
            break;

        default:
            break;
    }

    LOG("auto -> %s\r\n", App_Auto_Get_StepName(step));
}

/**
 * @brief 判断当前步骤是否超时
 * @param limit_ms 超时阈值
 * @return 1 = 已超时
 */
static uint8_t App_Auto_Timeout(uint32_t limit_ms)
{
    return ((HAL_GetTick() - s_step_tick) > limit_ms) ? 1U : 0U;
}

void App_Auto_Init(void)
{
    s_step = AUTO_STEP_INIT;
    s_step_tick = HAL_GetTick();
    s_route = VISION_ROUTE_INVALID;
    s_last_light = VISION_LIGHT_UNKNOWN;
    s_yellow_ok = 0U;
    s_yellow_tick = 0U;

    /* 出发时车上带着初始块，载荷状态由流程自己维护 */
    App_Robot_SetLoad(LOAD_CARRY_V);
}

void App_Auto_Stop(void)
{
    App_Auto_StopCurrent();
    App_Chassis_RequestStop();

    /* 只停任务，不复位状态：DONE/FAIL 保留到下次进入自动时再清 */
}

void App_Auto_Update(void)
{
    uint8_t light = App_Vision_GetLight();

    /* 灯色每周期提交：底盘只在当周期消费，不能只在变化时提交 */
    (void)App_Chassis_SetLight(light, (uint8_t)((App_Vision_IsOk() != 0U) ? 1U : 0U));

    /* 黄灯计时：只认"非黄 → 黄"的跳变，起点才可信 */
    if (light == VISION_LIGHT_YELLOW)
    {
        if (s_last_light != VISION_LIGHT_YELLOW)
        {
            s_yellow_tick = HAL_GetTick();
            s_yellow_ok = 1U;
        }
    }
    else
    {
        s_yellow_ok = 0U;
    }

    s_last_light = light;

    switch (s_step)
    {
        case AUTO_STEP_INIT:
            s_route = App_Vision_GetRoute();
            if (s_route != VISION_ROUTE_INVALID)
            {
                App_Auto_EnterStep(AUTO_STEP_LINE1);
            }
            break;

        case AUTO_STEP_LINE1:
            if (App_Chassis_IsArrived() != 0U)
            {
                App_Auto_EnterStep((APP_AUTO_LAP_ENABLE != 0) ? AUTO_STEP_LAP : AUTO_STEP_LIGHT);
            }
            else if (App_Auto_Timeout(AUTO_TIMEOUT_LINE_MS) != 0U)
            {
                App_Auto_Fail("line1");
            }
            break;

        case AUTO_STEP_LAP:
            if (App_Chassis_IsLapDone() != 0U)
            {
                App_Auto_EnterStep(AUTO_STEP_LIGHT);
            }
            else if (App_Auto_Timeout(AUTO_TIMEOUT_LAP_MS) != 0U)
            {
                App_Auto_Fail("lap");
            }
            break;

        case AUTO_STEP_LIGHT:
            if (light == VISION_LIGHT_GREEN)
            {
                App_Auto_EnterStep(AUTO_STEP_TUNNEL);
            }
            else if ((light == VISION_LIGHT_YELLOW) && (s_yellow_ok != 0U) &&
                     ((HAL_GetTick() - s_yellow_tick) <= AUTO_YELLOW_ENTER_MS))
            {
                App_Auto_EnterStep(AUTO_STEP_TUNNEL);
            }
            else if (App_Auto_Timeout(AUTO_TIMEOUT_LIGHT_MS) != 0U)
            {
                App_Auto_Fail("light");
            }
            break;

        case AUTO_STEP_TUNNEL:
            if (App_Chassis_IsArrived() != 0U)
            {
                App_Auto_EnterStep(AUTO_STEP_LINE2);
            }
            else if (App_Auto_Timeout(AUTO_TIMEOUT_TUNNEL_MS) != 0U)
            {
                App_Auto_Fail("tunnel");
            }
            break;

        case AUTO_STEP_LINE2:
            if (App_Chassis_IsArrived() != 0U)
            {
                App_Auto_EnterStep(AUTO_STEP_PLACE);
            }
            else if (App_Auto_Timeout(AUTO_TIMEOUT_LINE_MS) != 0U)
            {
                App_Auto_Fail("line2");
            }
            break;

        case AUTO_STEP_PLACE:
            if (App_Arm_IsBusy() == 0U)
            {
                App_Auto_EnterStep(AUTO_STEP_DONE);
            }
            else if (App_Auto_Timeout(AUTO_TIMEOUT_ARM_MS) != 0U)
            {
                App_Auto_Fail("place timeout");
            }
            break;

        case AUTO_STEP_DONE:
        case AUTO_STEP_FAIL:
        default:
            break;
    }
}
