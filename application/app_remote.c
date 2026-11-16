/**
 * @file    app_remote.c
 * @brief   遥控处理实现：读帧、应用状态、摇杆/按键处理、链路超时判断。
 * @owner   电控B
 */

#include "app_remote.h"
#include "app_config.h"
#include "app_robot.h"
#include "remote_protocol.h"
#include "main.h"          /* HAL_GetTick */

/** 前后目标量（-100~100，+ 为前） */
static int8_t s_vx;

/** 平移目标量（-100~100，+ 为左） */
static int8_t s_vy;

/** 旋转目标量（-100~100，+ 为左旋） */
static int8_t s_wz;

/** 精细/修正档：1 = 精细（原样映射），0 = 修正（小偏差吸附） */
static uint8_t s_fine_mode;

/** 链路状态：1 = 正常 */
static uint8_t s_link_ok;

/** 最近一次收到有效帧的 tick */
static uint32_t s_last_rx_tick;

/** 上一次的按键状态，用于边沿检测 */
static uint8_t s_prev_btn_low;
static uint8_t s_prev_btn_high;

/**
 * @brief 求绝对值
 * @param value 输入值
 * @return 绝对值
 */
static int8_t App_Remote_Abs(int8_t value)
{
    return (value < 0) ? (int8_t)(-value) : value;
}

/**
 * @brief 停止运动：清掉所有目标量
 * @param  无
 * @return 无
 * @note   链路超时、急停、模式退出都调用它，避免多处重复清目标
 */
static void App_Remote_StopMotion(void)
{
    s_vx = 0;
    s_vy = 0;
    s_wz = 0;
}

/**
 * @brief 摇杆死区处理
 * @param raw 原始值（-100~100）
 * @return 死区内返回 0，否则返回原值
 * @note   精细/修正档都不改变摇杆比例，只影响是否吸附方向
 */
static int8_t App_Remote_ApplyDeadzone(int8_t raw)
{
    if ((raw > -RC_JOY_DEADZONE) && (raw < RC_JOY_DEADZONE))
    {
        return 0;
    }

    return raw;
}

/**
 * @brief 处理摇杆：死区 + 修正档"小偏差吸附"
 * @param vx_raw 原始前后值
 * @param vy_raw 原始平移值
 * @return 无
 * @note  精细档：摇杆原样映射（任意方向）；
 *        修正档：主轴取绝对值大的那个，另一轴偏差 <= RC_SNAP_RANGE_PCT 时清零（吸附到纯前后/纯左右）；
 *                偏差大于该值说明是有意斜推，保留原方向。
 */
static void App_Remote_ApplyStick(int8_t vx_raw, int8_t vy_raw)
{
    int8_t vx = App_Remote_ApplyDeadzone(vx_raw);
    int8_t vy = App_Remote_ApplyDeadzone(vy_raw);

    if (s_fine_mode == 0U)
    {
        if (App_Remote_Abs(vx) >= App_Remote_Abs(vy))
        {
            if (App_Remote_Abs(vy) <= RC_SNAP_RANGE_PCT)
            {
                vy = 0;
            }
        }
        else
        {
            if (App_Remote_Abs(vx) <= RC_SNAP_RANGE_PCT)
            {
                vx = 0;
            }
        }
    }

    s_vx = vx;
    s_vy = vy;
}

/**
 * @brief 处理旋转键（按住生效，松开为 0）
 * @param btn_low 低 8 键位图
 * @return 无
 */
static void App_Remote_ApplyRotate(uint8_t btn_low)
{
    if ((btn_low & RC_BTN_ROT_FAST_LEFT) != 0U)
    {
        s_wz = (int8_t)(-RC_ROT_FAST_PCT);
    }
    else if ((btn_low & RC_BTN_ROT_FAST_RIGHT) != 0U)
    {
        s_wz = (int8_t)RC_ROT_FAST_PCT;
    }
    else if ((btn_low & RC_BTN_ROT_SLOW_LEFT) != 0U)
    {
        s_wz = (int8_t)(-RC_ROT_SLOW_PCT);
    }
    else if ((btn_low & RC_BTN_ROT_SLOW_RIGHT) != 0U)
    {
        s_wz = (int8_t)RC_ROT_SLOW_PCT;
    }
    else
    {
        s_wz = 0;
    }
}

/**
 * @brief 处理精细/修正切换键（按下沿生效）
 * @param pressed 本帧新按下的高 8 键位图
 * @return 无
 */
static void App_Remote_HandleFineKey(uint8_t pressed)
{
    if ((pressed & RC_BTN_FINE_MODE) == 0U)
    {
        return;
    }

    s_fine_mode = (s_fine_mode == 0U) ? 1U : 0U;

    LOG("fine mode -> %s\r\n", (s_fine_mode != 0U) ? "FINE" : "SNAP");
}

/**
 * @brief 处理急停键（按下沿触发）
 * @param pressed 本帧新按下的高 8 键位图
 * @return 无
 * @note   这是软件备份：硬件急停是独立回路。收到后立刻停车并回初始模式。
 */
static void App_Remote_HandleEstop(uint8_t pressed)
{
    if ((pressed & RC_BTN_ESTOP) == 0U)
    {
        return;
    }

    App_Remote_StopMotion();
    App_Robot_ApplyMode(APP_MODE_INIT_V);
    LOG("ESTOP (software)\r\n");
}

/**
 * @brief 处理机械臂动作键（按下沿触发一次）
 * @param pressed 本帧新按下的低 8 键位图
 * @return 无
 * @note   目前只打日志；app_arm 写好后替换成动作请求
 */
static void App_Remote_HandleArmKeys(uint8_t pressed)
{
    if ((pressed & RC_BTN_ARM_READY) != 0U)
    {
        LOG("key A ready\r\n");         /* TODO: App_Arm_Request(ARM_ACT_READY) */
    }

    if ((pressed & RC_BTN_ARM_PICK) != 0U)
    {
        LOG("key B pick\r\n");          /* TODO: App_Arm_Request(ARM_ACT_PICK) */
    }

    if ((pressed & RC_BTN_ARM_PLACE) != 0U)
    {
        LOG("key C place\r\n");         /* TODO: App_Arm_Request(ARM_ACT_PLACE) */
    }

    if ((pressed & RC_BTN_ARM_HOME) != 0U)
    {
        LOG("key D home\r\n");          /* TODO: App_Arm_Request(ARM_ACT_HOME) */
    }
}

/**
 * @brief 处理一帧遥控数据
 * @param data 解析结果
 * @return 无
 */
static void App_Remote_HandleFrame(const RemoteData_t *data)
{
    uint8_t pressed_low;
    uint8_t pressed_high;

    /* 状态字段：直接应用（值不变也无害） */
    App_Robot_ApplyMode(data->mode);
    App_Robot_SetTerrain(data->terrain);
    App_Robot_SetLevel(data->level);
    App_Robot_SetLoad(data->load);

    /* 摇杆与旋转 */
    App_Remote_ApplyStick(data->vx, data->vy);
    App_Remote_ApplyRotate(data->btn_low);

    /* 按键边沿：本帧按下且上一帧没按 */
    pressed_low = (uint8_t)(data->btn_low & (uint8_t)(~s_prev_btn_low));
    pressed_high = (uint8_t)(data->btn_high & (uint8_t)(~s_prev_btn_high));

    App_Remote_HandleFineKey(pressed_high);
    App_Remote_HandleEstop(pressed_high);
    App_Remote_HandleArmKeys(pressed_low);

    s_prev_btn_low = data->btn_low;
    s_prev_btn_high = data->btn_high;
}

/**
 * @brief 链路超时判断
 * @param  无
 * @return 无
 * @note  超过 RC_RX_WARN_MS 打一次警告；超过 RC_RX_LOST_MS 判丢失、停车并回初始模式
 */
static void App_Remote_CheckLink(void)
{
    uint32_t elapsed;

    if (s_link_ok == 0U)
    {
        return;
    }

    elapsed = HAL_GetTick() - s_last_rx_tick;

    if (elapsed > RC_RX_LOST_MS)
    {
        s_link_ok = 0U;
        App_Remote_StopMotion();
        App_Robot_OnLinkLost();
        LOG("remote link lost (%lu ms)\r\n", (unsigned long)elapsed);
    }
    else if (elapsed > RC_RX_WARN_MS)
    {
        LOG("remote frame late (%lu ms)\r\n", (unsigned long)elapsed);
    }
}

void App_Remote_Init(void)
{
    RemoteData_t data;

    s_vx = 0;
    s_vy = 0;
    s_wz = 0;
    s_fine_mode = 0U;
    s_link_ok = 1U;
    s_last_rx_tick = HAL_GetTick();
    s_prev_btn_low = 0U;
    s_prev_btn_high = 0U;

    Remote_Init();

    /* 清掉上电时缓冲里可能残留的字节，避免把旧帧当有效数据 */
    while (Remote_GetFrame(&data) != 0U)
    {
    }
}

void App_Remote_Update(void)
{
    RemoteData_t data;

    while (Remote_GetFrame(&data) != 0U)
    {
        s_last_rx_tick = HAL_GetTick();
        s_link_ok = 1U;

        App_Remote_HandleFrame(&data);
    }

    App_Remote_CheckLink();
}

int8_t App_Remote_GetVx(void)
{
    return s_vx;
}

int8_t App_Remote_GetVy(void)
{
    return s_vy;
}

int8_t App_Remote_GetWz(void)
{
    return s_wz;
}

uint8_t App_Remote_IsFineMode(void)
{
    return s_fine_mode;
}

uint8_t App_Remote_IsLinkOk(void)
{
    return s_link_ok;
}
