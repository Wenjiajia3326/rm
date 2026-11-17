/**
 * @file    app_selftest.c
 * @brief   上电自测实现：CRC、遥控帧、视觉帧、基础状态机。
 * @owner   电控B
 *
 * 数据来源：
 *   遥控样例帧：《萝卜杯_遥控帧协议 V1.3》第 5 节（11 条正例 + 1 条 CRC 反例）
 *   视觉测试向量：《萝卜杯_视觉串口协议 V1.0》第 5 节（8 条正例 + 1 条 CRC 反例）
 */

#include "app_selftest.h"
#include "app_config.h"
#include "app_robot.h"
#include "crc8.h"
#include "remote_protocol.h"
#include "vision_protocol.h"
#include "bsp_uart.h"

/** 遥控用例 */
typedef struct
{
    const uint8_t *bytes;
    uint8_t length;
    int8_t  vx;
    int8_t  vy;
    uint8_t btn_low;
    uint8_t btn_high;
    uint8_t mode;
    uint8_t terrain;
    uint8_t level;
    uint8_t load;
    uint8_t seq;
} SelfTestRemote_t;

/** 视觉用例 */
typedef struct
{
    const uint8_t *bytes;
    uint8_t length;
    uint8_t type;
    uint8_t value;      /* 0x10 的路线 或 0x11 的灯色 */
    uint8_t state;      /* 0x12 的心跳状态 */
    uint8_t fps;        /* 0x12 的自报帧率 */
    uint8_t conf;
    uint8_t ttl;
    uint8_t seq;
} SelfTestVision_t;

/* ---------------- 遥控样例帧（15 字节） ---------------- */

/** 初始态：静止无按键（mode=0） */
static const uint8_t s_rc_0[] = { 0xAA, 0x55, 0x02, 0x20, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x72 };

/** 比赛-手动，前进 50%（mode=2, vx=50） */
static const uint8_t s_rc_1[] = { 0xAA, 0x55, 0x02, 0x20, 0x09, 0x32, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0xCB };

/** 比赛-自动（mode=1） */
static const uint8_t s_rc_2[] = { 0xAA, 0x55, 0x02, 0x20, 0x09, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x1E };

/** 颠簸地形（terrain=1） */
static const uint8_t s_rc_3[] = { 0xAA, 0x55, 0x02, 0x20, 0x09, 0x00, 0x00, 0x00, 0x00, 0x02, 0x01, 0x00, 0x00, 0x03, 0xA9 };

/** 高度第 3 档（level=2） */
static const uint8_t s_rc_4[] = { 0xAA, 0x55, 0x02, 0x20, 0x09, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x04, 0x7C };

/** 携块（load=1） */
static const uint8_t s_rc_5[] = { 0xAA, 0x55, 0x02, 0x20, 0x09, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x05, 0xB8 };

/** A 伸臂准备按下（low.bit0） */
static const uint8_t s_rc_6[] = { 0xAA, 0x55, 0x02, 0x20, 0x09, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x06, 0x7B };

/** 快速左旋按住（low.bit4） */
static const uint8_t s_rc_7[] = { 0xAA, 0x55, 0x02, 0x20, 0x09, 0x00, 0x00, 0x10, 0x00, 0x02, 0x00, 0x00, 0x00, 0x07, 0x70 };

/** 精细/修正切换按下（high.bit0） */
static const uint8_t s_rc_8[] = { 0xAA, 0x55, 0x02, 0x20, 0x09, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x08, 0xA7 };

/** 急停（high.bit1） */
static const uint8_t s_rc_9[] = { 0xAA, 0x55, 0x02, 0x20, 0x09, 0x00, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0x09, 0xDB };

/** seq 回绕示例（seq=0xFF） */
static const uint8_t s_rc_10[] = { 0xAA, 0x55, 0x02, 0x20, 0x09, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0xFF, 0x45 };

/** 反例：CRC 坏，必须丢弃 */
static const uint8_t s_rc_bad[] = { 0xAA, 0x55, 0x02, 0x20, 0x09, 0x32, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0xEB };

/* ---------------- 视觉测试向量 ---------------- */

/** 数字：1 号路线，92% */
static const uint8_t s_vi_0[] = { 0xAA, 0x55, 0x01, 0x10, 0x04, 0x01, 0x5C, 0x04, 0x11, 0x25 };

/** 数字：2 号路线，88% */
static const uint8_t s_vi_1[] = { 0xAA, 0x55, 0x01, 0x10, 0x04, 0x02, 0x58, 0x04, 0x17, 0xA6 };

/** 数字：没看见（route=0） */
static const uint8_t s_vi_2[] = { 0xAA, 0x55, 0x01, 0x10, 0x04, 0x00, 0x00, 0x01, 0x18, 0x93 };

/** 灯色：绿，88% */
static const uint8_t s_vi_3[] = { 0xAA, 0x55, 0x01, 0x11, 0x04, 0x02, 0x58, 0x04, 0x31, 0x7D };

/** 灯色：红，90% */
static const uint8_t s_vi_4[] = { 0xAA, 0x55, 0x01, 0x11, 0x04, 0x01, 0x5A, 0x04, 0x2B, 0xD7 };

/** 灯色：黄，70% */
static const uint8_t s_vi_5[] = { 0xAA, 0x55, 0x01, 0x11, 0x04, 0x03, 0x46, 0x04, 0x32, 0xEC };

/** 灯色：没看见（color=0） */
static const uint8_t s_vi_6[] = { 0xAA, 0x55, 0x01, 0x11, 0x04, 0x00, 0x00, 0x01, 0x2C, 0x36 };

/** 心跳：在线，帧率 30 */
static const uint8_t s_vi_7[] = { 0xAA, 0x55, 0x01, 0x12, 0x02, 0x01, 0x1E, 0xB0 };

/** 反例：CRC 被改坏，必须丢弃 */
static const uint8_t s_vi_bad[] = { 0xAA, 0x55, 0x01, 0x10, 0x04, 0x02, 0x58, 0x99, 0x17, 0xA6 };

/** 遥控用例表 */
static const SelfTestRemote_t s_remote_cases[] =
{
    { s_rc_0,  15U,   0,   0, 0x00U, 0x00U, 0U, 0U, 0U, 0U, 0x00U },
    { s_rc_1,  15U,  50,   0, 0x00U, 0x00U, 2U, 0U, 0U, 0U, 0x01U },
    { s_rc_2,  15U,   0,   0, 0x00U, 0x00U, 1U, 0U, 0U, 0U, 0x02U },
    { s_rc_3,  15U,   0,   0, 0x00U, 0x00U, 2U, 1U, 0U, 0U, 0x03U },
    { s_rc_4,  15U,   0,   0, 0x00U, 0x00U, 2U, 0U, 2U, 0U, 0x04U },
    { s_rc_5,  15U,   0,   0, 0x00U, 0x00U, 2U, 0U, 0U, 1U, 0x05U },
    { s_rc_6,  15U,   0,   0, 0x01U, 0x00U, 2U, 0U, 0U, 0U, 0x06U },
    { s_rc_7,  15U,   0,   0, 0x10U, 0x00U, 2U, 0U, 0U, 0U, 0x07U },
    { s_rc_8,  15U,   0,   0, 0x00U, 0x01U, 2U, 0U, 0U, 0U, 0x08U },
    { s_rc_9,  15U,   0,   0, 0x00U, 0x02U, 2U, 0U, 0U, 0U, 0x09U },
    { s_rc_10, 15U,   0,   0, 0x00U, 0x00U, 2U, 0U, 0U, 0U, 0xFFU },
};

/** 视觉用例表 */
static const SelfTestVision_t s_vision_cases[] =
{
    { s_vi_0, 10U, 0x10U, 1U, 0U, 0U,  92U, 4U, 0x11U },
    { s_vi_1, 10U, 0x10U, 2U, 0U, 0U,  88U, 4U, 0x17U },
    { s_vi_2, 10U, 0x10U, 0U, 0U, 0U,   0U, 1U, 0x18U },
    { s_vi_3, 10U, 0x11U, 2U, 0U, 0U,  88U, 4U, 0x31U },
    { s_vi_4, 10U, 0x11U, 1U, 0U, 0U,  90U, 4U, 0x2BU },
    { s_vi_5, 10U, 0x11U, 3U, 0U, 0U,  70U, 4U, 0x32U },
    { s_vi_6, 10U, 0x11U, 0U, 0U, 0U,   0U, 1U, 0x2CU },
    { s_vi_7,  8U, 0x12U, 0U, 1U, 30U,  0U, 0U, 0U    },
};

/** 通过数 / 失败数 */
static uint8_t s_pass;
static uint8_t s_fail;

/**
 * @brief 记录一条用例结果
 * @param name 用例名
 * @param ok   1 = 通过
 * @return 无
 */
static void App_Selftest_Report(const char *name, uint8_t ok)
{
    if (ok != 0U)
    {
        s_pass++;
        LOG("[selftest] %-30s PASS\r\n", name);
    }
    else
    {
        s_fail++;
        LOG("[selftest] %-30s FAIL\r\n", name);
    }
}

/**
 * @brief 校验 CRC 算法
 * @param  无
 * @return 无
 * @note  标准向量 "123456789" 的 CRC-8/ATM 应为 0xF4
 */
static void App_Selftest_Crc(void)
{
    static const uint8_t data[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    uint8_t crc = Crc8_Calc(data, 9U);

    App_Selftest_Report("crc8 123456789", (crc == 0xF4U) ? 1U : 0U);

    if (crc != 0xF4U)
    {
        LOG("  expect 0x%02X got 0x%02X\r\n", 0xF4U, crc);
    }
}

/**
 * @brief 逐条跑遥控样例帧
 * @param  无
 * @return 无
 */
static void App_Selftest_Remote(void)
{
    RemoteData_t data;
    uint8_t i;
    uint8_t ok;

    Remote_Init();

    for (i = 0U; i < (uint8_t)(sizeof(s_remote_cases) / sizeof(s_remote_cases[0])); i++)
    {
        const SelfTestRemote_t *c = &s_remote_cases[i];

        BSP_Uart_TestInject(BSP_UART_REMOTE, c->bytes, c->length);

        ok = Remote_GetFrame(&data);

        if (ok != 0U)
        {
            ok = (uint8_t)((data.vx       == c->vx)       && (data.vy      == c->vy)      &&
                           (data.btn_low  == c->btn_low)  && (data.btn_high == c->btn_high) &&
                           (data.mode     == c->mode)     && (data.terrain  == c->terrain) &&
                           (data.level    == c->level)    && (data.load     == c->load)    &&
                           (data.seq      == c->seq));
        }

        App_Selftest_Report("remote sample", ok);

        if (ok == 0U)
        {
            LOG("  case %u: expect mode=%u vx=%d seq=0x%02X\r\n",
                i, c->mode, (int)c->vx, c->seq);
            LOG("           got    mode=%u vx=%d seq=0x%02X\r\n",
                data.mode, (int)data.vx, data.seq);
        }
    }

    /* 反例：CRC 坏必须被丢弃 */
    BSP_Uart_TestInject(BSP_UART_REMOTE, s_rc_bad, sizeof(s_rc_bad));
    App_Selftest_Report("remote bad crc rejected", (Remote_GetFrame(&data) == 0U) ? 1U : 0U);

    /* 结束自测，恢复该通道的正常计数 */
    BSP_Uart_TestInject(BSP_UART_REMOTE, 0, 0U);
}

/**
 * @brief 逐条跑视觉测试向量
 * @param  无
 * @return 无
 */
static void App_Selftest_Vision(void)
{
    VisionFrame_t frame;
    uint8_t i;
    uint8_t ok;
    uint8_t value;

    Vision_Init();

    for (i = 0U; i < (uint8_t)(sizeof(s_vision_cases) / sizeof(s_vision_cases[0])); i++)
    {
        const SelfTestVision_t *c = &s_vision_cases[i];

        BSP_Uart_TestInject(BSP_UART_VISION, c->bytes, c->length);

        ok = Vision_GetFrame(&frame);

        if (ok != 0U)
        {
            ok = (uint8_t)((frame.vision_type == c->type) && (frame.vision_conf == c->conf) &&
                           (frame.vision_ttl_50ms == c->ttl) && (frame.vision_seq == c->seq));
        }

        if ((ok != 0U) && (c->type == APP_FRAME_TYPE_VISION_ROUTE))
        {
            ok = (uint8_t)(frame.vision_route == c->value);
        }

        if ((ok != 0U) && (c->type == APP_FRAME_TYPE_VISION_LIGHT))
        {
            ok = (uint8_t)(frame.vision_light == c->value);
        }

        if ((ok != 0U) && (c->type == APP_FRAME_TYPE_VISION_HB))
        {
            ok = (uint8_t)((frame.vision_state == c->state) && (frame.vision_fps == c->fps));
        }

        App_Selftest_Report("vision vector", ok);

        if (ok == 0U)
        {
            value = (c->type == APP_FRAME_TYPE_VISION_LIGHT) ? frame.vision_light : frame.vision_route;
            LOG("  case %u: expect type=0x%02X value=%u\r\n", i, c->type, c->value);
            LOG("           got    type=0x%02X value=%u\r\n", frame.vision_type, value);
        }
    }

    /* 反例：CRC 坏必须被丢弃 */
    BSP_Uart_TestInject(BSP_UART_VISION, s_vi_bad, sizeof(s_vi_bad));
    App_Selftest_Report("vision bad crc rejected", (Vision_GetFrame(&frame) == 0U) ? 1U : 0U);

    /* 结束自测，恢复该通道的正常计数 */
    BSP_Uart_TestInject(BSP_UART_VISION, 0, 0U);
}

/**
 * @brief 校验基础状态机（模式名、载荷）
 * @param  无
 * @return 无
 */
static void App_Selftest_Robot(void)
{
    uint8_t ok;

    App_Robot_SetLoad(LOAD_CARRY_V);
    ok = (uint8_t)(App_Robot_GetLoad() == LOAD_CARRY_V);
    App_Robot_SetLoad(LOAD_EMPTY_V);
    ok = (uint8_t)(ok && (App_Robot_GetLoad() == LOAD_EMPTY_V));
    App_Selftest_Report("robot load state", ok);

    ok = (uint8_t)((App_Robot_GetModeName(APP_MODE_INIT_V) != 0) &&
                   (App_Robot_GetModeName(APP_MODE_COMP_AUTO_V) != 0));
    App_Selftest_Report("robot mode name", ok);
}

uint8_t App_Selftest_Run(void)
{
    s_pass = 0U;
    s_fail = 0U;

    LOG("---- selftest begin ----\r\n");

    App_Selftest_Crc();
    App_Selftest_Remote();
    App_Selftest_Vision();
    App_Selftest_Robot();

    LOG("---- selftest: %u pass, %u fail ----\r\n", s_pass, s_fail);

    return s_fail;
}
