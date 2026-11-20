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
    ARM_ACT_FREE = 0,       /**< 空闲 */

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
 *         A/B 要求空载，C 要求携块，D 不限载荷，不满足则拒绝
 */
void App_Arm_Set_Action(ArmAction_t action);

/**
 * @brief 查询是否正在执行动作
 * @return 1 = 执行中；0 = 空闲
 */
uint8_t App_Arm_IsBusy(void);

/**
 * @brief 查询当前动作
 * @return 当前动作，空闲为 ARM_ACT_FREE
 */
ArmAction_t App_Arm_GetAction(void);

/**
 * @brief 立即停止机械臂：取消当前动作，4 个关节停在当前位置
 * @param  无
 * @return 无
 * @note   急停、失联、切模式时调用；已发出的段不再继续，剩余段不再下发
 */
void App_Arm_Stop(void);

/**
 * @brief 查询机械臂是否在收纳位
 * @return 1 = 收纳（可全速行驶）；0 = 伸出或位置不确定（限速）
 * @note   按"最后完成的动作"约定推出来的，不是实测角度；
 *         只用于互锁限速，不能当安全判据
 */
uint8_t App_Arm_IsStowed(void);


/**
 * @brief 调试模式下选中一个关节
 * @param joint 关节编号（ARM_JOINT_BASE / ARM1 / ARM2 / GRIP）
 * @return 无
 * @note   只在 ARM_DEBUG 模式下调用；重复选同一个关节是幂等的
 */
void App_Arm_Debug_Select(ArmJoint_t joint);

/**
 * @brief 调试模式 10 ms 推进：按摇杆推当前选中关节的脉宽
 * @param stick 摇杆前后量（-100~100，+ = 脉宽增）
 * @param fine  1 = 细调（100 脉宽/s），0 = 粗调（500 脉宽/s）
 * @return 无
 * @note   没选关节、或段表动作还在跑时直接返回；
 *         脉宽被夹在软限位内；本次上电第一推动作用长 T 缓动
 */
void App_Arm_Debug_Pulse_Update(int8_t stick, uint8_t fine);

#endif /* APP_ARM_H */
