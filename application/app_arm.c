/**
 * @file    app_arm.c
 * @brief   机械臂动作实现：分段表 + 按时间推进。
 * @owner   电控B
 *
 * 注意：段表里的脉宽值目前全部是 0（未标定），等机械B 标定后填入真实脉宽 500~2500。
 *       未填之前不要按 A/B/C/D：0 会被夹到 500，关节会转到底。
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
    const ArmSegment_t *segments;   /**< 段数组 */
    uint8_t step_amount;            /**< 一共几段 */
} ArmPlan_t;

/** 关节 -> 舵机 ID 映射（按机械B 给的串联顺序） */
static const uint8_t s_servo_id[ARM_JOINT_NUM] =
{
    SERVO_ID_BASE, SERVO_ID_ARM1, SERVO_ID_ARM2, SERVO_ID_GRIP
};

/* ===================== 调试模式（ARM_DEBUG） ===================== */
/*
 * 旋转四键直选关节，摇杆上下推脉宽，精细键切粗/细。
 * 舵机只发不收，所以这里维护"记账脉宽"：开机假设 1500，之后每次下发都更新。
 */

/** 记账脉宽：主控认为 4 个关节当前的目标值 */
static uint16_t s_pulse[ARM_JOINT_NUM];

/** 最后一次真正下发的脉宽（判断有没有变化用） */
static uint16_t s_last_pulse[ARM_JOINT_NUM];

/** 当前选中关节，ARM_JOINT_NUM = 没选 */
static uint8_t s_debug_select_joint;

/** 不足 1 脉宽的部分（单位：脉宽的 1/100） */
static int16_t s_debug_acc;

/** 本次上电第一次动：用长 T 慢慢蹭 */
static uint8_t s_debug_first_flag;

/** 1 = 按约定在收纳位；上电先按"伸出"处理（保守） */
static uint8_t s_arm_stowed;


/** 软限位：机械B 回填《机构接口单》#5 后按关节收窄 */
#define ARM_DEBUG_PULSE_MIN     800U
#define ARM_DEBUG_PULSE_MAX     2200U

/** 推进速度：主环 10 ms 一次，满推每 tick 加多少脉宽（粗 500/s、细 100/s） */
#define ARM_DEBUG_STEP_COARSE   5
#define ARM_DEBUG_STEP_FINE     1

/** 指令 T 只影响平滑、不影响速度；给大了松手后还会多走一段 */
#define ARM_DEBUG_T_COARSE_MS   50U
#define ARM_DEBUG_T_FINE_MS     100U
#define ARM_DEBUG_T_FIRST_MS    3000U

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
        case ARM_ACT_FREE:  return "FREE";
    }
}

/**
 * @brief 取动作对应的分段表
 * @param action 动作
 * @return 分段表指针；ARM_ACT_FREE 返回 0
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

        case ARM_ACT_FREE:
            return 0;
    }
}

/**
 * @brief 把一段参数发给 4 个舵机
 * @param segment 段参数
 * @return 无
 * @note   4 条指令共约 5 ms 阻塞，只在换段时发一次
 */
static void App_Arm_SendSegment(const ArmSegment_t *segment)
{
    uint8_t joint;

    for (joint = 0U; joint < (uint8_t)ARM_JOINT_NUM; joint++)
    {
        if (segment->pulse[joint] == 0U)
        {
            continue;           /* 0 = 本段不发，别夹到 500 转到底 */
        }

        s_pulse[joint] = segment->pulse[joint];   /* 记账跟着动作走 */
        s_last_pulse[joint] = segment->pulse[joint];

        Servo_Move(s_servo_id[joint], segment->pulse[joint], segment->time_ms);
    }
}

/**
 * @brief 动作结束：更新载荷状态并回到空闲
 * @param  无
 * @return 无
 * @note   B 抓取完成 -> 置携块；C 放置完成 -> 置空载
 */
static void App_Arm_Renew_State(void)
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

    /* 约定：只有走完 D 才算回到收纳位 */
    s_arm_stowed = (s_action == ARM_ACT_HOME) ? 1U : 0U;

    s_action = ARM_ACT_FREE;
    s_plan = 0;
    s_step = 0U;
}

void App_Arm_Init(void)
{
    uint8_t joint;

    s_action = ARM_ACT_FREE;
    s_plan = 0;
    s_step = 0U;
    s_step_tick = 0U;

    s_arm_stowed = 0U;          /* 上电位置未知，按伸出处理 */
    s_debug_select_joint = (uint8_t)ARM_JOINT_NUM;
    s_debug_acc = 0;
    s_debug_first_flag = 1U;

    /* 开机释力：避免舵机上电自动冲向初始值 1500 */
    for (joint = 0U; joint < (uint8_t)ARM_JOINT_NUM; joint++)
    {
        s_pulse[joint] = 1500U;     /* 假设值：实际位置未知 */
        s_last_pulse[joint] = 1500U;

        Servo_SetBootTorque(s_servo_id[joint], 0U);
    }
}

void App_Arm_Update(void)
{
    const ArmSegment_t *segment;

    if (s_action == ARM_ACT_FREE)
    {
        return;
    }

    segment = &s_plan->segments[s_step];

    if ((HAL_GetTick() - s_step_tick) < segment->time_ms)
    {
        return;
    }

    s_step++;

    if (s_step >= s_plan->step_amount)
    {
        App_Arm_Renew_State();
        return;
    }

    s_step_tick = HAL_GetTick();
    App_Arm_SendSegment(&s_plan->segments[s_step]);
}

void App_Arm_Set_Action(ArmAction_t action)
{
    uint8_t load;
    const ArmPlan_t *plan;

    if (s_action != ARM_ACT_FREE)
    {
        LOG("arm busy: ignore %s\r\n", App_Arm_ActionName(action));
        return;
    }

    plan = App_Arm_GetPlan(action);

    if (plan == 0)
    {
        return;
    }

    /* 守卫：A/B 要求空载；C 要求携块；D 收回不限载荷（携块也要收进隧道包络） */
    load = App_Robot_GetLoad();

    if (action == ARM_ACT_PLACE)
    {
        if (load != LOAD_CARRY_V)
        {
            LOG("arm: place needs block\r\n");
            return;
        }
    }
    else if (action != ARM_ACT_HOME)
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
    return (s_action != ARM_ACT_FREE) ? 1U : 0U;
}

ArmAction_t App_Arm_GetAction(void)
{
    return s_action;
}

uint8_t App_Arm_IsStowed(void)
{
    return s_arm_stowed;
}


void App_Arm_Stop(void)
{
    uint8_t joint;

    if (s_action != ARM_ACT_FREE)
    {
        LOG("arm stop: %s\r\n", App_Arm_ActionName(s_action));
        s_arm_stowed = 0U;      /* 停在半路，按伸出处理 */
    }

    s_action = ARM_ACT_FREE;
    s_plan = 0;
    s_step = 0U;
    s_debug_acc = 0;
    s_debug_select_joint = (uint8_t)ARM_JOINT_NUM;   /* 停掉就取消选中，下次进调试要重新选 */

    /* 停在当前位置：不读回角度，靠舵机自身的 PDST 保持 */
    for (joint = 0U; joint < (uint8_t)ARM_JOINT_NUM; joint++)
    {
        Servo_Stop(s_servo_id[joint]);
    }
}

void App_Arm_Debug_Select(ArmJoint_t joint)
{
    s_debug_select_joint = (uint8_t)joint;
    s_debug_acc = 0;

    LOG("arm dbg: joint %u pulse=%u\r\n",
        (unsigned int)((uint8_t)joint + 1U), (unsigned int)s_pulse[joint]);
}

void App_Arm_Debug_Pulse_Update(int8_t stick, uint8_t fine)
{
    int16_t rate;
    int16_t delta;
    int16_t pulse;
    uint16_t time_ms;

    if ((s_debug_select_joint >= (uint8_t)ARM_JOINT_NUM) || (s_action != ARM_ACT_FREE))
    {
        return;             /* 没选关节；或段表动作还在跑，会打架 */
    }

    rate = (fine != 0U) ? ARM_DEBUG_STEP_FINE : ARM_DEBUG_STEP_COARSE;
    s_debug_acc += (int16_t)((int16_t)stick * rate);

    delta = (int16_t)(s_debug_acc / 100);
    s_debug_acc -= (int16_t)(delta * 100);

    if (delta == 0)
    {
        return;             /* 累加还没到一个脉宽 */
    }

    pulse = (int16_t)s_pulse[s_debug_select_joint] + delta;

    if (pulse > (int16_t)ARM_DEBUG_PULSE_MAX)
    {
        pulse = (int16_t)ARM_DEBUG_PULSE_MAX;
    }
    else if (pulse < (int16_t)ARM_DEBUG_PULSE_MIN)
    {
        pulse = (int16_t)ARM_DEBUG_PULSE_MIN;
    }

    if (pulse == (int16_t)s_last_pulse[s_debug_select_joint])
    {
        s_debug_acc = 0;    /* 顶到限位：清累加，反向时立刻响应 */
        return;
    }

    if (s_debug_first_flag != 0U)
    {
        time_ms = ARM_DEBUG_T_FIRST_MS;
        s_debug_first_flag = 0U;
    }
    else
    {
        time_ms = (fine != 0U) ? ARM_DEBUG_T_FINE_MS : ARM_DEBUG_T_COARSE_MS;
    }

    s_pulse[s_debug_select_joint] = (uint16_t)pulse;
    s_last_pulse[s_debug_select_joint] = (uint16_t)pulse;
    s_arm_stowed = 0U;                  /* 推过脉宽，不再算收纳 */

    Servo_Move(s_servo_id[s_debug_select_joint], (uint16_t)pulse, time_ms);
}
