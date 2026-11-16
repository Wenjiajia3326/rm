/**
 * @file    app_arm.c
 * @brief   机械臂动作实现：分段表 + 按时间推进。
 * @owner   电控B
 *
 * 注意：段表里的脉宽值目前全部是 0（未标定），0 表示"该关节本段不发指令"，
 *       等机械B 标定后填入真实脉宽（500~2500）。
 *       段时长取自《电控B-机械B 机构接口单》：A 900 / B 1200 / C 900 / D 600 ms。
 */

#include "app_arm.h"
#include "app_config.h"
#include "app_robot.h"
#include "servo.h"
#include "main.h"           /* HAL_GetTick */

/** 单个关节段的参数 */
typedef struct
{
    uint16_t pulse[ARM_JOINT_NUM];  /**< 4 个关节的目标脉宽（500~2500），0 = 本段不发 */
    uint16_t time_ms;               /**< 本段时长（ms） */
} ArmSegment_t;

/** 一个动作 = 若干段 */
typedef struct
{
    const ArmSegment_t *segments;
    uint8_t count;
} ArmPlan_t;

/** 关节 -> 舵机 ID 映射（按机械B 给的串联顺序） */
static const uint8_t s_servo_id[ARM_JOINT_NUM] =
{
    SERVO_ID_BASE, SERVO_ID_ARM1, SERVO_ID_ARM2, SERVO_ID_GRIP
};

/* ===================== 动作表（脉宽待机械B 标定，未标定为 0） ===================== */
/* 每个关节的顺序：{ 底座, 下臂, 上臂, 夹具 } */

/* A: 回收纳位+伸出（空载->空载） */
static const ArmSegment_t s_plan_ready[2] =
{
    { { 0U, 0U, 0U, 0U }, 600U },   /* 回收纳位 */
    { { 0U, 0U, 0U, 0U }, 300U },   /* 伸出 */
};

/* B: 下降+夹紧+抬起（空载->携块） */
static const ArmSegment_t s_plan_pick[3] =
{
    { { 0U, 0U, 0U, 0U }, 600U },   /* 下降到抓取位 */
    { { 0U, 0U, 0U, 0U }, 400U },   /* 夹紧 */
    { { 0U, 0U, 0U, 0U }, 200U },   /* 抬起 */
};

/* C: 下降+松开+后撤（携块->空载），一层 */
static const ArmSegment_t s_plan_place_l1[3] =
{
    { { 0U, 0U, 0U, 0U }, 400U },   /* 下降到一层 */
    { { 0U, 0U, 0U, 0U }, 200U },   /* 松开 */
    { { 0U, 0U, 0U, 0U }, 300U },   /* 后撤 */
};

/* C: 下降+松开+后撤（携块->空载），二层 */
static const ArmSegment_t s_plan_place_l2[3] =
{
    { { 0U, 0U, 0U, 0U }, 400U },   /* 下降到二层 */
    { { 0U, 0U, 0U, 0U }, 200U },   /* 松开 */
    { { 0U, 0U, 0U, 0U }, 300U },   /* 后撤 */
};

/* C: 下降+松开+后撤（携块->空载），三层 */
static const ArmSegment_t s_plan_place_l3[3] =
{
    { { 0U, 0U, 0U, 0U }, 400U },   /* 下降到三层 */
    { { 0U, 0U, 0U, 0U }, 200U },   /* 松开 */
    { { 0U, 0U, 0U, 0U }, 300U },   /* 后撤 */
};

/* D: 抬直+折叠+放到底盘（空载->空载） */
static const ArmSegment_t s_plan_home[3] =
{
    { { 0U, 0U, 0U, 0U }, 250U },   /* 抬直下臂 */
    { { 0U, 0U, 0U, 0U }, 200U },   /* 上臂与下臂折叠 */
    { { 0U, 0U, 0U, 0U }, 150U },   /* 放到底盘 */
};

/** 当前动作 */
static ArmAction_t s_action;

/** 当前动作的分段表 */
static const ArmPlan_t *s_plan;

/** 当前段序号 */
static uint8_t s_step;

/** 当前段开始时刻 */
static uint32_t s_step_tick;

/**
 * @brief 取动作名（日志用）
 * @param action 动作
 * @return 名称字符串
 */
static const char *App_Arm_ActionName(ArmAction_t action)
{
    switch (action)
    {
        case ARM_ACT_READY: return "READY";
        case ARM_ACT_PICK:  return "PICK";
        case ARM_ACT_PLACE: return "PLACE";
        case ARM_ACT_HOME:  return "HOME";
        default:            return "NONE";
    }
}

/**
 * @brief 取动作对应的分段表
 * @param action 动作
 * @return 分段表指针；未知动作返回 0
 * @note   C 动作按当前高度档（App_Robot_GetLevel）选一层/二层/三层表
 */
static const ArmPlan_t *App_Arm_GetPlan(ArmAction_t action)
{
    static const ArmPlan_t plan_ready   = { s_plan_ready,   2U };
    static const ArmPlan_t plan_pick    = { s_plan_pick,    3U };
    static const ArmPlan_t plan_place_l1 = { s_plan_place_l1, 3U };
    static const ArmPlan_t plan_place_l2 = { s_plan_place_l2, 3U };
    static const ArmPlan_t plan_place_l3 = { s_plan_place_l3, 3U };
    static const ArmPlan_t plan_home    = { s_plan_home,    3U };

    switch (action)
    {
        case ARM_ACT_READY: return &plan_ready;
        case ARM_ACT_PICK:  return &plan_pick;
        case ARM_ACT_HOME:  return &plan_home;

        case ARM_ACT_PLACE:
            if (App_Robot_GetLevel() == 1U)
            {
                return &plan_place_l2;
            }

            if (App_Robot_GetLevel() == 2U)
            {
                return &plan_place_l3;
            }

            return &plan_place_l1;

        default:
            return 0;
    }
}

/**
 * @brief 把一段参数发给 4 个舵机
 * @param segment 段参数
 * @return 无
 * @note   脉宽为 0 表示该关节本段未标定，跳过不发（否则会被夹到 500 乱动）
 * @note   4 条指令共约 5 ms 阻塞，只在换段时发一次
 */
static void App_Arm_SendSegment(const ArmSegment_t *segment)
{
    uint8_t joint;

    for (joint = 0U; joint < (uint8_t)ARM_JOINT_NUM; joint++)
    {
        if (segment->pulse[joint] == 0U)
        {
            continue;
        }

        Servo_Move(s_servo_id[joint], segment->pulse[joint], segment->time_ms);
    }
}

/**
 * @brief 动作结束：更新载荷状态并回到空闲
 * @param  无
 * @return 无
 * @note   B 抓取完成 -> 置携块；C 放置完成 -> 置空载
 */
static void App_Arm_Finish(void)
{
    if (s_action == ARM_ACT_PICK)
    {
        App_Robot_SetLoad(LOAD_CARRY_V);
    }
    else if (s_action == ARM_ACT_PLACE)
    {
        App_Robot_SetLoad(LOAD_EMPTY_V);
    }

    LOG("arm done: %s\r\n", App_Arm_ActionName(s_action));

    s_action = ARM_ACT_NONE;
    s_plan = 0;
    s_step = 0U;
}

void App_Arm_Init(void)
{
    uint8_t joint;

    s_action = ARM_ACT_NONE;
    s_plan = 0;
    s_step = 0U;
    s_step_tick = 0U;

    /* 开机释力：避免舵机上电自动冲向初始值 1500 */
    for (joint = 0U; joint < (uint8_t)ARM_JOINT_NUM; joint++)
    {
        Servo_SetBootTorque(s_servo_id[joint], 0U);
    }
}

void App_Arm_Update(void)
{
    const ArmSegment_t *segment;

    if (s_action == ARM_ACT_NONE)
    {
        return;
    }

    segment = &s_plan->segments[s_step];

    if ((HAL_GetTick() - s_step_tick) < segment->time_ms)
    {
        return;
    }

    s_step++;

    if (s_step >= s_plan->count)
    {
        App_Arm_Finish();
        return;
    }

    s_step_tick = HAL_GetTick();
    App_Arm_SendSegment(&s_plan->segments[s_step]);
}

void App_Arm_Motion(ArmAction_t action)
{
    uint8_t load;
    const ArmPlan_t *plan;

    if (s_action != ARM_ACT_NONE)
    {
        LOG("arm busy: ignore %s\r\n", App_Arm_ActionName(action));
        return;
    }

    plan = App_Arm_GetPlan(action);

    if (plan == 0)
    {
        return;
    }

    /* 守卫：A/B/D 进入时为空载；C 进入时为携块 */
    load = App_Robot_GetLoad();

    if (action == ARM_ACT_PLACE)
    {
        if (load != LOAD_CARRY_V)
        {
            LOG("arm: place needs block\r\n");
            return;
        }
    }
    else
    {
        if (load != LOAD_EMPTY_V)
        {
            LOG("arm: %s needs empty\r\n", App_Arm_ActionName(action));
            return;
        }
    }

    s_action = action;
    s_plan = plan;
    s_step = 0U;
    s_step_tick = HAL_GetTick();

    LOG("arm start: %s\r\n", App_Arm_ActionName(action));

    App_Arm_SendSegment(&s_plan->segments[0]);
}

uint8_t App_Arm_IsBusy(void)
{
    return (s_action != ARM_ACT_NONE) ? 1U : 0U;
}

ArmAction_t App_Arm_GetAction(void)
{
    return s_action;
}
