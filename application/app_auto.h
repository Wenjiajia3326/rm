/**
 * @file    app_auto.h
 * @brief   自动阶段状态机：锁路 → 循迹1 → 绕圈 → 等灯 → 过隧道 → 循迹2 → 建筑区放块。
 * @owner   电控B
 *
 * 开局已携初始块并处于收纳态，一路带到建筑区才放，所以没有"搬运初始块"这一步。
 */

#ifndef APP_AUTO_H
#define APP_AUTO_H

#include <stdint.h>

/** 自动阶段步骤 */
typedef enum
{
    AUTO_STEP_INIT = 0,   /**< A01 等视觉锁定路线 */
    AUTO_STEP_LINE1,      /**< A02 第一段循迹 */
    AUTO_STEP_LAP,        /**< A02.1 绕圈（APP_AUTO_LAP_ENABLE 打开才做） */
    AUTO_STEP_LIGHT,      /**< A04 等绿灯 */
    AUTO_STEP_TUNNEL,     /**< A05 过隧道 */
    AUTO_STEP_LINE2,      /**< A06 第二段循迹 */
    AUTO_STEP_PLACE,      /**< A07 建筑区放初始块 */
    AUTO_STEP_DONE,       /**< 流程结束 */
    AUTO_STEP_FAIL        /**< 超时或启动被拒，已停车 */
} AutoStep_t;

/**
 * @brief 初始化自动状态机并置携块载荷
 * @param  无
 * @return 无
 * @note   进入自动模式时调用
 */
void App_Auto_Init(void);

/**
 * @brief 10 ms 主环：推进自动流程
 * @param  无
 * @return 无
 */
void App_Auto_Update(void);

/**
 * @brief 退出自动模式：停掉底盘任务
 * @param  无
 * @return 无
 * @note   不复位状态，DONE/FAIL 保留到下一次 App_Auto_Init()
 */
void App_Auto_Stop(void);

/**
 * @brief 取步骤名（日志用）
 * @param step 步骤
 * @return 名称字符串
 */
const char *App_Auto_Get_StepName(AutoStep_t step);

#endif /* APP_AUTO_H */
