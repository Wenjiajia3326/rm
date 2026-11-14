/**
 * @file    app_robot.h
 * @brief   整机状态：把遥控帧里的模式/地形/高度/载荷转成机器人当前运行状态。
 * @owner   电控B
 *
 * 职责边界：
 *   手持端负责"选哪个模式/哪档"，主控只接收"已生效值"，不做候选逻辑；
 *   模式变化时由本模块执行进入/退出动作（请求安全态、复位自动流程等）。
 *   具体模式的流程逻辑将来放到 app_auto / app_arm 等文件，本模块只做分派。
 */

#ifndef APP_ROBOT_H
#define APP_ROBOT_H

#include <stdint.h>

/**
 * @brief 初始化整机状态
 *
 * 模式回初始态，地形/高度/载荷取默认值。
 *
 * @param  无
 * @return 无
 * @note   上电调用一次，必须在 BSP_Uart_Init() 之后
 */
void App_Robot_Init(void);

/**
 * @brief 周期调用（10 ms），按当前模式分派流程
 *
 * 目前各模式分支为空占位；将来自动流程实现后，这里改成调用 App_Auto_Update()。
 *
 * @param  无
 * @return 无
 * @note   非阻塞；本函数不直接写执行器目标
 */
void App_Robot_Update(void);

/**
 * @brief 应用遥控帧里的模式值
 * @param mode 帧内 mode 字段（APP_MODE_*_V，协议层已校验范围）
 * @return 无
 * @note   模式变化时先执行旧模式退出动作，再执行新模式进入动作
 * @note   模式未变化时函数立即返回，不做任何事
 */
void App_Robot_ApplyMode(uint8_t mode);

/**
 * @brief 链路丢失时调用：强制回初始模式
 * @param  无
 * @return 无
 * @note   由 app_remote 在超过 RC_RX_LOST_MS 未收到有效帧时调用
 */
void App_Robot_OnLinkLost(void);

/**
 * @brief 取当前模式
 * @param  无
 * @return 当前模式（APP_MODE_*_V）
 */
uint8_t App_Robot_GetMode(void);

/**
 * @brief 设置地形档（来自帧内 terrain 字段）
 * @param terrain 地形档取值（TERRAIN_*_V）
 * @return 无
 */
void App_Robot_SetTerrain(uint8_t terrain);

/**
 * @brief 取当前地形档
 * @param  无
 * @return 地形档取值
 */
uint8_t App_Robot_GetTerrain(void);

/**
 * @brief 设置放置高度档（来自帧内 level 字段）
 * @param level 高度档索引
 * @return 无
 */
void App_Robot_SetLevel(uint8_t level);

/**
 * @brief 取当前放置高度档
 * @param  无
 * @return 高度档索引
 */
uint8_t App_Robot_GetLevel(void);

/**
 * @brief 设置载荷状态（来自帧内 load 字段）
 * @param load LOAD_EMPTY_V / LOAD_CARRY_V
 * @return 无
 */
void App_Robot_SetLoad(uint8_t load);

/**
 * @brief 取当前载荷状态
 * @param  无
 * @return LOAD_EMPTY_V / LOAD_CARRY_V
 */
uint8_t App_Robot_GetLoad(void);

/**
 * @brief 取模式名称字符串，用于日志
 * @param mode 模式取值
 * @return 静态字符串（"INIT"/"COMP_AUTO"/"COMP_MANUAL"/"ARM_DEBUG"/"CHASSIS_DEBUG"）
 */
const char *App_Robot_ModeName(uint8_t mode);

#endif /* APP_ROBOT_H */
