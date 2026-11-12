/**
 * @file    app_config.h
 * @brief   全工程集中配置：通信协议常量、时间阈值、舵机 ID、动作时间与限速参数。
 * @owner   电控B（底盘专属参数由电控A 维护本文件对应分区）
 *
 * 修改原则：
 * 1) 只放"有配置意义"的值，不放纯粹的局部临时数字；
 * 2) 改这里可能影响多个模块，改完必须重新编译并通知协作者；
 * 3) 帧头/类型/位定义这类常量改动等于改协议，必须同步视觉C、手柄固件与硬件。
 * 4) 本文件被 application 与 components 共同引用（组内约定：协议常量集中于此）。
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ===================== 一、通用帧结构（视觉/遥控共用） ===================== */
#define APP_FRAME_HEAD0                 0xAAU   ///< 帧头第 1 字节
#define APP_FRAME_HEAD1                 0x55U   ///< 帧头第 2 字节
#define APP_PROTOCOL_VER                0x01U   ///< 协议版本，改格式必须 +1

/* 帧类型 */
#define APP_FRAME_TYPE_VISION_ROUTE     0x10U   ///< 视觉：号码牌选路
#define APP_FRAME_TYPE_VISION_LAMP      0x11U   ///< 视觉：信号灯颜色
#define APP_FRAME_TYPE_VISION_HB        0x12U   ///< 视觉：心跳
#define APP_FRAME_TYPE_REMOTE           0x20U   ///< 遥控帧

/* 整帧长度（字节） */
#define APP_FRAME_LEN_VISION_ROUTE      10U     ///< 选路帧长度
#define APP_FRAME_LEN_VISION_LAMP       10U     ///< 灯色帧长度
#define APP_FRAME_LEN_VISION_HB         7U      ///< 心跳帧长度
#define APP_FRAME_LEN_REMOTE            11U     ///< 遥控帧长度
#define APP_FRAME_LEN_REMOTE_DATA       5U      ///< 遥控数据段长度（vx/vy/btn_low/btn_high/seq）

/* 视觉数据字段取值 */
#define VISION_ROUTE_INVALID            0U      ///< route = 0 表示没看见
#define VISION_LAMP_UNKNOWN             0U      ///< color = 0 表示没看见
#define VISION_LAMP_RED                 1U      ///< 红灯
#define VISION_LAMP_GREEN               2U      ///< 绿灯
#define VISION_LAMP_YELLOW              3U      ///< 黄灯

/* ===================== 二、视觉消息参数 ===================== */
#define VISION_CONF_MIN                 75U     ///< 置信度阈值（0-100，视觉C 已确认）
#define VISION_MSG_TTL_MS               200U    ///< 消息有效期（ms）
#define VISION_HB_TIMEOUT_MS            3000U   ///< 心跳超时（ms）
#define VISION_ERROR_FRAME_MAX          10U     ///< 连续错误帧上限，超过判链路异常

/* ===================== 三、遥控参数 ===================== */
#define RC_JOY_DEADZONE                 8       ///< 摇杆死区（百分比，±8）
#define RC_AXIS_MAX                     100     ///< 摇杆输出上限（百分比满速）
#define RC_RX_WARN_MS                   150U    ///< 丢帧告警阈值（ms）
#define RC_RX_LOST_MS                   300U    ///< 判定链路丢失（ms）

/* 遥控按键位定义：btn_low（机械臂 4 键 + 旋转 4 键） */
#define RC_BTN_ARM_READY                0x01U   ///< A 伸臂准备
#define RC_BTN_ARM_PICK                 0x02U   ///< B 夹紧并抬起
#define RC_BTN_ARM_PLACE                0x04U   ///< C 下降并松开
#define RC_BTN_ARM_HOME                 0x08U   ///< D 收回机械臂
#define RC_BTN_ROT_FAST_LEFT            0x10U   ///< 快速左旋
#define RC_BTN_ROT_FAST_RIGHT           0x20U   ///< 快速右旋
#define RC_BTN_ROT_SLOW_LEFT            0x40U   ///< 精细左旋
#define RC_BTN_ROT_SLOW_RIGHT           0x80U   ///< 精细右旋

/* 遥控按键位定义：btn_high（全局键 + 预留） */
#define RC_BTN_MODE                     0x01U   ///< 模式键
#define RC_BTN_OK                       0x02U   ///< 确认键
#define RC_BTN_FINE_MODE                0x04U   ///< 精细/修正切换
#define RC_BTN_ESTOP                    0x08U   ///< 急停（软件备份，硬件急停独立）
#define RC_BTN_RESERVED0                0x10U   ///< 预留：全局
#define RC_BTN_RESERVED1                0x20U   ///< 预留：全局
#define RC_BTN_RESERVED2                0x40U   ///< 预留：机械臂
#define RC_BTN_RESERVED3                0x80U   ///< 预留：机械臂

/* ===================== 四、舵机 ===================== */
#define SERVO_ID_LIFT                   1U      ///< 大臂升降
#define SERVO_ID_WRIST                  2U      ///< 腕/对位
#define SERVO_ID_GRIP_A                 3U      ///< 夹爪 A
#define SERVO_ID_GRIP_B                 4U      ///< 夹爪 B（只用 3 只时该 ID 不使用）
#define SERVO_ACTION_TIMEOUT_PCT        140U    ///< 动作超时 = 规划时间 × 140%

/* ===================== 五、动作规划时间（ms） ===================== */
#define ACT_TIME_HOME_MS                600U    ///< 收纳
#define ACT_TIME_PICK_MS                1200U   ///< 抓取（含抬升）
#define ACT_TIME_PLACE_MS               900U    ///< 放置（先按 900，机械B 实测后调整）
#define ACT_TIME_RETREAT_MS             500U    ///< 撤离
#define ACT_TIME_READY_MS               300U    ///< 伸臂准备

/* ===================== 六、互锁限速（百分比，数值待实测后定稿） ===================== */
#define CHASSIS_LIMIT_NORMAL_PCT        100U    ///< 收纳态行驶
#define CHASSIS_LIMIT_ARM_OUT_PCT       50U     ///< 臂伸出时行驶
#define CHASSIS_LIMIT_CARRY_PCT         70U     ///< 携块运输
#define CHASSIS_LIMIT_TUNNEL_PCT        40U     ///< 过隧道姿态
#define CHASSIS_LIMIT_ACTION_PCT        0U      ///< 抓放动作进行中：禁止行驶

/* ===================== 七、调度与任务 ===================== */
#define APP_ROBOT_PERIOD_MS             10U     ///< 状态机/仲裁主环周期
#define APP_ROBOT_TASK_STACK_WORDS      256U    ///< 主任务栈（word）
#define APP_ROBOT_TASK_PRIORITY         3U      ///< 主任务优先级

#endif /* APP_CONFIG_H */
