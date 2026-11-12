/**
 * @file    app_robot.c
 * @brief   全局模式状态机：初始选择态（APP_MODE_INIT）+ 比赛 / 机械臂调试 / 底盘调试。
 * @owner   电控B
 *
 * 设计要点：
 * 1) 上电先进入 APP_MODE_INIT（选择态），不自动进比赛模式，避免上电即动作；
 * 2) 选择态里 MODE 只改候选、OK 才真正进入，进入前检查安全条件；
 * 3) 进入任一模式后不再接受模式切换，必须断电重启
 *    （比赛中误触 MODE 会直接毁掉整局，宁可要求重启）。
 */

#include "app_robot.h"

/** 下一个模式（OK 确认后进入）：初值比赛模式，在 APP_MODE_INIT 态由 MODE 键循环切换 */
static AppMode_t s_next_mode = APP_MODE_COMPETITION;

/** 当前已进入的模式；APP_MODE_INIT 表示还没进入 */
static AppMode_t s_mode = APP_MODE_INIT;

/**
 * @brief 判断当前是否允许进入/切换模式
 * @return 1 = 允许，0 = 不允许
 * @note  目前恒为允许。底盘与机构接入后，这里要补：
 *        - 底盘目标速度为 0，且实际速度接近 0
 *        - 机构处于收纳/安全姿态
 */
static uint8_t App_Robot_IsSafeToSwitch(void)
{
    return 1U;
}

/**
 * @brief 初始化整机调度模块，把模式复位为初始选择态
 *
 * 上电调用一次；同时把"下一个模式"复位为比赛模式，保证第一次按 OK 进入比赛。
 *
 * @param  无
 * @return 无
 *
 * @note   必须在调度器启动后、任务循环开始前调用；重复调用会把模式强制拉回 INIT
 * @note   不改任何硬件，执行器安全态由各驱动模块初始化时自行保证
 */
void App_Robot_Init(void)
{
    s_mode = APP_MODE_INIT;
    s_next_mode = APP_MODE_COMPETITION;
}

/**
 * @brief 周期推进模式状态机（建议 10 ms 调用一次）
 *
 * 按当前模式分派处理：INIT 态空转等待按键；比赛与两个调试模式目前为空实现，
 * 后续分别填入"待命->自动->结算冻结->手动"和"单关节微调"等逻辑。
 *
 * @param  无
 * @return 无
 *
 * @note   非阻塞；内部只做条件判断与状态迁移，不写执行器目标
 * @note   调用周期由 APP_ROBOT_PERIOD_MS 约定，周期改变会影响后续阶段计时
 */
void App_Robot_Update(void)
{
    switch (s_mode)
    {
        case APP_MODE_INIT:
            /* 初始选择态：不做任何动作，等待 MODE/OK；执行器应保持安全态 */
            break;

        case APP_MODE_COMPETITION:
            /* 比赛模式：后续在此推进 待命 -> 自动 -> 结算冻结 -> 手动 */
            break;

        case APP_MODE_ARM_DEBUG:
            /* 机械臂调试：后续在此处理单关节微调 */
            break;

        case APP_MODE_CHASSIS_DEBUG:
            /* 底盘调试：后续在此处理底盘单独调试 */
            break;

        default:
            /* 枚举异常时退回 APP_MODE_INIT，避免停在未定义分支 */
            s_mode = APP_MODE_INIT;
            break;
    }
}

/**
 * @brief 处理 MODE 键按下事件（边沿已由调用者判定）
 *
 * 仅在 APP_MODE_INIT 态有效：把"下一个模式"按 比赛 -> 机械臂调试 -> 底盘调试 -> 比赛 循环推进。
 *
 * @param  无（按键"刚按下"的判定由 app_remote 负责）
 * @return 无
 *
 * @note   已进入任一模式后直接返回，比赛中不会因误触 MODE 而切换
 * @note   本函数不提供反馈；需要提示时读 App_Robot_GetNextMode() 自行点灯/打印
 */
void App_Robot_OnModeKey(void)
{
    if (s_mode != APP_MODE_INIT)
    {
        return;
    }

    /* APP_MODE_INIT 态内循环：比赛 -> 机械臂调试 -> 底盘调试 -> 比赛 */
    if (s_next_mode >= APP_MODE_CHASSIS_DEBUG)
    {
        s_next_mode = APP_MODE_COMPETITION;
    }
    else
    {
        s_next_mode = (AppMode_t)((uint32_t)s_next_mode + 1U);
    }
}

/**
 * @brief 处理 OK 键按下事件（边沿已由调用者判定）
 *
 * INIT 态下检查安全条件，满足则把"下一个模式"确认为当前模式。
 *
 * @param  无
 * @return 1 = 已进入模式（本来就已进入时也返回 1，重复确认按成功处理）
 *         0 = 安全条件不满足，本次未进入，模式仍为 APP_MODE_INIT
 *
 * @note   安全条件由 App_Robot_IsSafeToSwitch() 判断，目前恒为允许
 * @note   进入后不再退回 INIT，要换模式只能断电重启
 */
uint8_t App_Robot_OnOkKey(void)
{
    if (s_mode != APP_MODE_INIT)
    {
        /* 已进入模式：重复确认视为成功，避免按键连击产生歧义 */
        return 1U;
    }

    if (App_Robot_IsSafeToSwitch() == 0U)
    {
        return 0U;
    }

    s_mode = s_next_mode;
    return 1U;
}

/**
 * @brief 查询当前已进入的模式
 *
 * @param  无
 * @return 当前模式枚举；尚未进入任何模式时返回 APP_MODE_INIT
 *
 * @note   只读，可在任意上下文调用
 */
AppMode_t App_Robot_GetMode(void)
{
    return s_mode;
}

/**
 * @brief 查询"下一个模式"（INIT 态下按 OK 会进入的那个）
 *
 * @param  无
 * @return 候选模式枚举；初值为 APP_MODE_COMPETITION
 *
 * @note   用于界面提示，例如 LED 闪烁次数、串口打印当前候选
 */
AppMode_t App_Robot_GetNextMode(void)
{
    return s_next_mode;
}

/**
 * @brief 判断是否已经进入某个模式
 *
 * @param  无
 * @return 1 = 已进入（当前模式不是 APP_MODE_INIT），0 = 仍在选择态
 *
 * @note   需要"确认已选定模式才允许动作"的流程，用本函数做门禁
 */
uint8_t App_Robot_IsEntered(void)
{
    return (s_mode != APP_MODE_INIT) ? 1U : 0U;
}

/**
 * @brief 取模式名称字符串，用于串口日志
 *
 * @param  mode 模式枚举值
 * @return 指向静态字符串的指针："INIT" / "COMPETITION" / "ARM_DEBUG" / "CHASSIS_DEBUG" / "UNKNOWN"
 *
 * @note   返回静态常量区字符串，调用者不要释放、不要修改
 * @note   传入越界枚举值时返回 "UNKNOWN"，不会越界访问
 */
const char *App_Robot_ModeName(AppMode_t mode)
{
    switch (mode)
    {
        case APP_MODE_INIT:
            return "INIT";
        case APP_MODE_COMPETITION:
            return "COMPETITION";
        case APP_MODE_ARM_DEBUG:
            return "ARM_DEBUG";
        case APP_MODE_CHASSIS_DEBUG:
            return "CHASSIS_DEBUG";
        default:
            return "UNKNOWN";
    }
}
