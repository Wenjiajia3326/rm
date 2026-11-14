/**
 * @file    app_vision.c
 * @brief   视觉消息有效性实现：路线锁存、灯色时效、心跳在线判断。
 * @owner   电控B
 */

#include "app_vision.h"
#include "app_config.h"
#include "vision_protocol.h"
#include "main.h"           /* HAL_GetTick */

/** 已锁定的路线：0 = 未锁定，1~3 = 已锁定 */
static uint8_t s_route;

/** 当前有效灯色（0 = 无有效值） */
static uint8_t s_light;

/** 灯色最近一次更新时间 */
static uint32_t s_light_tick;

/** 心跳最近一次收到时间 */
static uint32_t s_heartbeat_tick;

/** 视觉在线：1 = 心跳新鲜 */
static uint8_t s_vision_ok;

/** 相机正常：心跳里的 state.bit0 */
static uint8_t s_camera_ok;

/** seq 诊断用：上一帧序号 / 是否已建立基准 */
static uint8_t s_last_seq;
static uint8_t s_have_seq;

/**
 * @brief 业务帧的 seq 连续性诊断
 * @param seq 本帧序号
 * @return 无
 * @note  只打日志，不影响功能；视觉重启由 App_Vision_Update 里的在线跳变处理
 */
static void App_Vision_CheckSeq(uint8_t seq)
{
    uint8_t expected;

    if (s_have_seq != 0U)
    {
        expected = (uint8_t)(s_last_seq + 1U);
        if (seq != expected)
        {
            LOG("vision seq jump: expect %u got %u\r\n", expected, seq);
        }
    }

    s_last_seq = seq;
    s_have_seq = 1U;
}

/**
 * @brief 处理选路帧（0x10）
 * @param frame 帧数据
 * @return 无
 * @note  只锁第一次：route 1~3 且 conf 达到门限才锁；锁定后不再改变
 */
static void App_Vision_HandleRoute(const VisionFrame_t *frame)
{
    if (s_route != 0U)
    {
        return;
    }

    if ((frame->vision_route == 0U) || (frame->vision_conf < VISION_CONF_MIN))
    {
        return;
    }

    s_route = frame->vision_route;

    LOG("vision route locked -> %u (conf=%u)\r\n", s_route, frame->vision_conf);
}

/**
 * @brief 处理灯色帧（0x11）
 * @param frame 帧数据
 * @param now   当前 tick
 * @return 无
 * @note  conf 不足视为"没看见"，不做更新；有效则刷新灯色与时间戳
 */
static void App_Vision_HandleLight(const VisionFrame_t *frame, uint32_t now)
{
    if (frame->vision_conf < VISION_CONF_MIN)
    {
        return;
    }

    s_light = frame->vision_light;
    s_light_tick = now;
}

/**
 * @brief 处理心跳帧（0x12）
 * @param frame 帧数据
 * @param now   当前 tick
 * @return 无
 * @note  心跳 1 Hz，不逐帧打印（日志阻塞约 7 ms，会占满串口）；
 *        只在"视觉上线/离线""相机有图/掉线"这几个跳变点打一次。
 */
static void App_Vision_HandleHeartbeat(const VisionFrame_t *frame, uint32_t now)
{
    uint8_t camera;

    s_heartbeat_tick = now;

    camera = (uint8_t)(frame->vision_state & 0x01U);

    if (s_vision_ok == 0U)
    {
        s_vision_ok = 1U;
        s_camera_ok = camera;

        Vision_ResetSeqExpectation();
        s_have_seq = 0U;

        LOG("vision online (camera=%u fps=%u)\r\n", s_camera_ok, frame->vision_fps);
        return;
    }

    if (camera != s_camera_ok)
    {
        s_camera_ok = camera;
        LOG("camera %s\r\n", (camera != 0U) ? "ok" : "lost");
    }
}

/**
 * @brief 更新在线状态与灯色时效
 * @param now 当前 tick
 * @return 无
 * @note  离线只在"在线 -> 离线"跳变时打印一次；上电时本来就是离线，不会误报
 */
static void App_Vision_CheckTimeouts(uint32_t now)
{
    if (s_vision_ok != 0U)
    {
        if ((now - s_heartbeat_tick) > VISION_HB_TIMEOUT_MS)
        {
            s_vision_ok = 0U;
            s_camera_ok = 0U;
            LOG("vision offline\r\n");
        }
    }

    /* 灯色时效：过期即视为没看见 */
    if (s_light != 0U)
    {
        if ((now - s_light_tick) > VISION_MSG_TTL_MS)
        {
            s_light = 0U;
        }
    }
}

void App_Vision_Init(void)
{
    VisionFrame_t frame;

    s_route = 0U;
    s_light = 0U;
    s_light_tick = 0U;
    s_heartbeat_tick = 0U;
    s_vision_ok = 0U;
    s_camera_ok = 0U;
    s_last_seq = 0U;
    s_have_seq = 0U;

    Vision_Init();

    /* 清掉上电时缓冲里可能残留的字节 */
    while (Vision_GetFrame(&frame) != 0U)
    {
    }
}

void App_Vision_Update(void)
{
    VisionFrame_t frame;
    uint32_t now;

    now = HAL_GetTick();

    while (Vision_GetFrame(&frame) != 0U)
    {
        if (frame.vision_type == APP_FRAME_TYPE_VISION_ROUTE)
        {
            App_Vision_CheckSeq(frame.vision_seq);
            App_Vision_HandleRoute(&frame);
        }
        else if (frame.vision_type == APP_FRAME_TYPE_VISION_LIGHT)
        {
            App_Vision_CheckSeq(frame.vision_seq);
            App_Vision_HandleLight(&frame, now);
        }
        else if (frame.vision_type == APP_FRAME_TYPE_VISION_HB)
        {
            App_Vision_HandleHeartbeat(&frame, now);
        }
    }

    App_Vision_CheckTimeouts(now);
}

uint8_t App_Vision_GetRoute(void)
{
    return s_route;
}

uint8_t App_Vision_GetLight(void)
{
    return s_light;
}

uint8_t App_Vision_IsOk(void)
{
    return s_vision_ok;
}

uint8_t App_Vision_IsCameraOk(void)
{
    return s_camera_ok;
}
