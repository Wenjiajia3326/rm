/**
 * @file    app_config.h
 * @brief   全工程集中配置：通信协议常量、模式档位取值、时间阈值、舵机 ID、动作时间与限速参数。
 * @owner   电控B（"底盘"分区由电控A 维护）
 *
 * 修改原则：
 * 1) 只放"有配置意义"的值，不放纯粹的局部临时数字；
 * 2) 改这里可能影响多个模块，改完必须重新编译并通知协作者；
 * 3) 帧头/类型/键位/档位取值改动等于改协议，必须同步视觉C、手柄固件与硬件；
 * 4) 本文件被 application 与 components 共同引用（组内约定：协议常量集中于此）。
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "bsp_log.h"    /* app -> bsp：应用层用日志函数，bsp 层不反向依赖 */

/* ===================== 〇、构建开关（比赛前按注释改） ===================== */
#define APP_LOG_ENABLE          1       ///< 日志总开关：1=开(开发) / 0=关(比赛)
#define APP_SELFTEST_ENABLE     1       ///< 上电自检：1=开(开发) / 0=关(比赛)
#define APP_SERIAL_CMD_ENABLE   1       ///< 串口调试命令：1=开(开发) / 0=关(比赛)
/* 比赛固件：三个都改成 0 */

/* ---- 日志宏：APP_LOG_ENABLE=0 时整段调用在编译期消失 ---- */
#if (APP_LOG_ENABLE != 0)
#define LOG(...)                BSP_Log_PrintFormat(__VA_ARGS__)
#else
#define LOG(...)                ((void)0)
#endif

/* ===================== 一、通用帧结构（owner：电控B） ===================== */
#define APP_FRAME_HEAD0                 0xAAU   ///< 帧头第 1 字节
#define APP_FRAME_HEAD1                 0x55U   ///< 帧头第 2 字节
#define APP_PROTOCOL_VER                0x01U   ///< 视觉协议版本（V1.0）
#define APP_PROTOCOL_VER_RC             0x02U   ///< 遥控协议版本（帧协议 V1.3 起）

/* 帧类型 */
#define APP_FRAME_TYPE_VISION_ROUTE     0x10U   ///< 视觉：号码牌选路
#define APP_FRAME_TYPE_VISION_LIGHT      0x11U   ///< 视觉：信号灯颜色
#define APP_FRAME_TYPE_VISION_HB        0x12U   ///< 视觉：心跳
#define APP_FRAME_TYPE_REMOTE           0x20U   ///< 遥控帧

/* 整帧长度（字节） */
#define APP_FRAME_LEN_VISION_ROUTE      10U     ///< 选路帧
#define APP_FRAME_LEN_VISION_LIGHT       10U     ///< 灯色帧
#define APP_FRAME_LEN_VISION_HB         8U      ///< 心跳帧（AA 55 01 12 02 state fps CRC）
#define APP_FRAME_LEN_REMOTE            15U     ///< 遥控帧（V1.3：14→15，加回 seq）
#define APP_FRAME_LEN_REMOTE_DATA       9U      ///< 数据段：vx/vy/btn_low/btn_high/mode/terrain/level/load/seq
#define APP_FRAME_RC_CRC_SPAN           12U     ///< 遥控 CRC 覆盖偏移 2~13 共 12 字节

/* 视觉数据字段取值 */
#define VISION_ROUTE_INVALID            0U      ///< route = 0 表示没看见
#define VISION_LIGHT_UNKNOWN             0U      ///< color = 0 表示没看见
#define VISION_LIGHT_RED                 1U      ///< 红灯
#define VISION_LIGHT_GREEN               2U      ///< 绿灯
#define VISION_LIGHT_YELLOW              3U      ///< 黄灯

/* ===================== 二、视觉消息参数（owner：电控B） ===================== */
#define VISION_CONF_MIN                 75U     ///< 置信度阈值（0-100，视觉C 确认）
#define VISION_MSG_TTL_MS               200U    ///< 消息有效期（ms）
#define VISION_HB_TIMEOUT_MS            3000U   ///< 心跳超时（ms）
#define VISION_ERROR_FRAME_MAX          10U     ///< 连续错误帧上限，超过判链路异常
#define VISION_BOOT_GRACE_MS            5000U   ///< 上电宽限：这段时间内不报 vision_ok=0

/* ===================== 三、遥控参数（owner：电控B） ===================== */
#define RC_FRAME_RATE_HZ                30U     ///< 手柄发送频率（协议 V1.3 约定）
#define RC_JOY_DEADZONE                 8       ///< 摇杆死区（百分比，±8）
#define RC_AXIS_MAX                     100     ///< 摇杆输出上限（百分比满速）
#define RC_SNAP_RANGE_PCT               15      ///< 修正档吸附范围（%）：偏小的轴 <=15 才吸附到主轴
#define RC_ROT_FAST_PCT                 100     ///< 快速旋转输出（百分比）
#define RC_ROT_SLOW_PCT                 40      ///< 精细旋转输出（百分比，数值待定）
#define RC_RX_WARN_MS                   150U    ///< 丢帧告警阈值（ms）
#define RC_RX_LOST_MS                   300U    ///< 判定链路丢失（ms）
#define RC_SEQ_MISS_WARN                5U      ///< 单次 seq 跳变超过该值即记一次链路抖动
#define RC_BTN_DEBOUNCE_MS              20U     ///< 按键消抖时间（ms）
#define RC_STOP_RAMP_MS                 200U    ///< 摇杆归零/停车斜坡（ms）

/* btn_low：动作键 + 旋转键（位定义） */
#define RC_BTN_ARM_READY                0x01U   ///< A 伸臂准备（回收纳位→伸出）
#define RC_BTN_ARM_PICK                 0x02U   ///< B 夹紧并抬起
#define RC_BTN_ARM_PLACE                0x04U   ///< C 下降+松开+后撤
#define RC_BTN_ARM_HOME                 0x08U   ///< D 收回
#define RC_BTN_ROT_FAST_LEFT            0x10U   ///< 快速左旋（按住）
#define RC_BTN_ROT_FAST_RIGHT           0x20U   ///< 快速右旋（按住）
#define RC_BTN_ROT_SLOW_LEFT            0x40U   ///< 精细左旋（按住）
#define RC_BTN_ROT_SLOW_RIGHT           0x80U   ///< 精细右旋（按住）

/* btn_high：仅 2 个功能键，其余 6 位预留 */
#define RC_BTN_FINE_MODE                0x01U   ///< 精细/修正切换（即时生效）
#define RC_BTN_ESTOP                    0x02U   ///< 急停（软件备份，硬件急停独立）

/* ===================== 四、模式与档位取值（owner：电控B，与协议 V1.3 字段一致） ===================== */
#define APP_MODE_INIT_V                 0U      ///< 初始 / 待命
#define APP_MODE_COMP_AUTO_V            1U      ///< 比赛-自动
#define APP_MODE_COMP_MANUAL_V          2U      ///< 比赛-手动
#define APP_MODE_ARM_DEBUG_V            3U      ///< 机械臂调试
#define APP_MODE_CHASSIS_DEBUG_V        4U      ///< 底盘调试

#define TERRAIN_ROAD_V                  0U      ///< 平路
#define TERRAIN_BUMP_V                  1U      ///< 颠簸

#define LOAD_EMPTY_V                    0U      ///< 空载
#define LOAD_CARRY_V                    1U      ///< 携块

/* ---- 数量：仅用于数组长度与边界检查，不对应任何取值 ---- */
#define APP_MODE_NUM                    5U      ///< 模式数量
#define TERRAIN_NUM                     2U      ///< 地形档数量（电控A 确认后调整）
#define LEVEL_NUM                       3U      ///< 高度档数量（机械B 标定后调整）

/* ===================== 五、舵机（owner：电控B，数值待机械B 标定） ===================== */
#define SERVO_ID_LIFT                   1U      ///< 大臂升降
#define SERVO_ID_WRIST                  2U      ///< 腕/对位
#define SERVO_ID_GRIP_A                 3U      ///< 夹爪 A
#define SERVO_ID_GRIP_B                 4U      ///< 夹爪 B（只用 3 只时该 ID 不使用）
#define SERVO_ACTION_TIMEOUT_PCT        140U    ///< 动作超时 = 规划时间 × 140%

/* ===================== 六、动作规划时间（owner：电控B，ms；数值待机械B 实测） ===================== */
#define ACT_TIME_READY_MS               900U    ///< A 伸臂准备（回收纳位 600 + 伸出 300）
#define ACT_TIME_PICK_MS                1200U   ///< B 夹紧+抬起
#define ACT_TIME_PLACE_MS               900U    ///< C 下降+松开+后撤（含退出接触）
#define ACT_TIME_HOME_MS                600U    ///< D 收回

/* ===================== 七、互锁限速（owner：电控B 起草，电控A 确认；百分比） ===================== */
#define CHASSIS_LIMIT_NORMAL_PCT        100U    ///< 收纳态行驶
#define CHASSIS_LIMIT_ARM_OUT_PCT       50U     ///< 臂伸出时行驶
#define CHASSIS_LIMIT_CARRY_PCT         70U     ///< 携块运输
#define CHASSIS_LIMIT_TUNNEL_PCT        40U     ///< 过隧道姿态
#define CHASSIS_LIMIT_ACTION_PCT        0U      ///< 抓放动作进行中：禁止行驶

/* ===================== 八、调度与任务（owner：电控B） ===================== */
#define APP_ROBOT_PERIOD_MS             10U     ///< 状态机/仲裁主环周期
#define APP_ROBOT_TASK_STACK_WORDS      256U    ///< 主任务栈（word）
#define APP_ROBOT_TASK_PRIORITY         3U      ///< 主任务优先级

#endif /* APP_CONFIG_H */
