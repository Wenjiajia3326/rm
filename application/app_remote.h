/**
 * @file    app_remote.h
 * @brief   遥控处理：读帧 -> 应用模式/档位 -> 摇杆与按键 -> 链路超时判断。
 * @owner   电控B
 *
 * 本模块是 10 ms 主环的入口：App_Remote_Update() 每 10 ms 调用一次。
 * 输出：vx / vy / wz 目标量（供底盘读取）、按键事件（供 app_arm 读取）。
 *
 * 档位说明：
 *   精细档：摇杆原样映射，摇杆往哪推车往哪走（可任意方向）；
 *   修正档：主轴取绝对值大的轴；若另一轴偏差 <= RC_SNAP_RANGE_PCT，则吸附成纯前后/纯左右。
 */

#ifndef APP_REMOTE_H
#define APP_REMOTE_H

#include <stdint.h>

/**
 * @brief 初始化遥控模块
 * @param  无
 * @return 无
 * @note   在 BSP_Uart_Init()、App_Robot_Init() 之后调用
 */
void App_Remote_Init(void);

/**
 * @brief 10 ms 主环：取完缓冲里所有遥控帧，应用状态，处理摇杆/按键，判断链路
 * @param  无
 * @return 无
 * @note   非阻塞；帧处理完之后才做一次超时判断
 */
void App_Remote_Update(void);

/**
 * @brief 取当前前后目标量
 * @return -100~100，+ 为前
 */
int8_t App_Remote_GetVx(void);

/**
 * @brief 取当前平移目标量
 * @return -100~100，+ 为左
 */
int8_t App_Remote_GetVy(void);

/**
 * @brief 取当前旋转目标量
 * @return -100~100，+ 为左旋
 */
int8_t App_Remote_GetWz(void);

/**
 * @brief 查询精细/修正档
 * @return 1 = 精细档（摇杆原样映射）
 *         0 = 修正档（小偏差自动吸附到前后左右）
 */
uint8_t App_Remote_IsFineMode(void);

/**
 * @brief 查询遥控链路是否正常
 * @return 1 = 正常；0 = 超过 RC_RX_LOST_MS 没收到有效帧
 */
uint8_t App_Remote_IsLinkOk(void);

#endif /* APP_REMOTE_H */
