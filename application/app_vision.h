/**
 * @file    app_vision.h
 * @brief   视觉消息有效性：路线锁存、灯色时效、心跳在线判断。
 * @owner   电控B
 *
 * 规则来源：《萝卜杯_视觉串口协议 V1.0》+《启动与断线机制》
 *   1) 0x10 选路：1 Hz；收到第一条 conf >= 门限 且 route 1~3 的帧后**锁存**，之后不再改；
 *   2) 0x11 灯色：20 Hz；ttl 内有效，过期即视为"没看见"（返回 0）；
 *   3) 0x12 心跳：1 Hz；超过 VISION_HB_TIMEOUT_MS 没收到 → 视觉离线；
 *      state.bit0 = 0 表示相机异常，但视觉进程仍在（vision_ok 仍为 1）；
 *   4) 视觉重启后 seq 从 0 重来：vision_ok 由 0 变 1 时重设 seq 基准，不记丢帧。
 */

#ifndef APP_VISION_H
#define APP_VISION_H

#include <stdint.h>

/**
 * @brief 初始化视觉模块
 * @param  无
 * @return 无
 * @note   在 BSP_Uart_Init() 之后调用
 */
void App_Vision_Init(void);

/**
 * @brief 10 ms 主环：取完缓冲里所有视觉帧，更新路线锁存/灯色时效/在线状态
 * @param  无
 * @return 无
 * @note   非阻塞
 */
void App_Vision_Update(void);

/**
 * @brief 取已锁定的路线
 * @return 0 = 尚未锁定；1~3 = 已锁定的路线
 * @note   锁定后不会再变（即使之后收到不同 route 也忽略）
 */
uint8_t App_Vision_GetRoute(void);

/**
 * @brief 取当前有效灯色
 * @return 0 = 没有有效灯色（没看见或已过期）；1 红 / 2 绿 / 3 黄
 * @note   超过 ttl（VISION_MSG_TTL_MS）自动作废，返回 0
 */
uint8_t App_Vision_GetLight(void);

/**
 * @brief 查询视觉是否在线
 * @return 1 = 心跳新鲜（VISION_HB_TIMEOUT_MS 内收到过心跳）；0 = 离线
 * @note   在线不等于相机正常，相机状态看 App_Vision_IsCameraOk()
 */
uint8_t App_Vision_IsOk(void);

/**
 * @brief 查询相机是否正常
 * @return 1 = 心跳里 state.bit0 = 1（相机出图正常）；0 = 相机异常
 */
uint8_t App_Vision_IsCameraOk(void);

#endif /* APP_VISION_H */
