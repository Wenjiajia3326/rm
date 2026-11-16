/**
 * @file    app_arm.h
 * @brief   机械臂动作：A 伸臂准备 / B 抓取 / C 放置 / D 收回。
 * @owner   电控B
 *
 * 实现方式：每个动作 = 若干"段"，每段一次性发出 4 个关节的目标脉宽与段时长，
 * 按段时长推进到下一段。开环控制，动作完成以时间为准（不读回角度）。
 */

#ifndef APP_ARM_H
#define APP_ARM_H

#include <stdint.h>

/** 关节编号（顺序与舵机 ID 一致：1~4） */
typedef enum
{
    ARM_JOINT_BASE = 0,     /**< 底座旋转 */
    ARM_JOINT_ARM1,         /**< 下臂 */
    ARM_JOINT_ARM2,         /**< 上臂 */
    ARM_JOINT_GRIP,         /**< 夹具 */
    ARM_JOINT_NUM
} ArmJoint_t;

/** 机构动作（对应手柄 A/B/C/D 键） */
typedef enum
{
    ARM_ACT_NONE = 0,       /**< 空闲 */

    /** A: 回收纳位+伸出（空载->空载） */
    ARM_ACT_READY,

    /** B: 下降+夹紧+抬起（空载->携块） */
    ARM_ACT_PICK,

    /** C: 下降+松开+后撤（携块->空载） */
    ARM_ACT_PLACE,

    /** D: 抬直+折叠+放到底盘（空载->空载） */
    ARM_ACT_HOME
} ArmAction_t;

/**
 * @brief 初始化机械臂模块
 * @param  无
 * @return 无
 * @note   上电先发开机释力，避免舵机自动冲向初始位置
 */
void App_Arm_Init(void);

/**
 * @brief 10 ms 主环：推进当前动作的分段
 * @param  无
 * @return 无
 * @note   空闲时直接返回；动作完成时自动更新载荷状态
 */
void App_Arm_Update(void);

/**
 * @brief 请求执行一个动作
 * @param action 动作（ARM_ACT_READY / PICK / PLACE / HOME）
 * @return 无
 * @note   执行中收到新请求会被忽略（打日志）；
 *         A/B/D 要求进入时为空载，C 要求进入时为携块，不满足则拒绝
 */
void App_Arm_Set_Action(ArmAction_t action);

/**
 * @brief 查询是否正在执行动作
 * @return 1 = 执行中；0 = 空闲
 */
uint8_t App_Arm_IsBusy(void);

/**
 * @brief 查询当前动作
 * @return 当前动作，空闲为 ARM_ACT_NONE
 */
ArmAction_t App_Arm_GetAction(void);

#endif /* APP_ARM_H */
