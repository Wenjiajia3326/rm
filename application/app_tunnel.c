// 隧道流程：入口前等灯，进入、穿过、完全驶出。
#include "app_tunnel.h"
#include "app_config.h"
typedef enum
{
    TUNNEL_APPROACH = 0,  // 接近入口参考位置。
    TUNNEL_WAIT,          // 在入口等待有效绿灯。
    TUNNEL_ENTER,         // 正在进入，尚未确认整车入内。
    TUNNEL_PASS,          // 已确认完全进入。
    TUNNEL_EXIT,          // 确认整车完全驶出。
    TUNNEL_END,           // 已确认完全驶出，交给底盘停车。
    TUNNEL_ERROR          // 通行超时或流程异常。
} TunnelState;
static TunnelState tunnel_state;  // 当前隧道任务阶段，跨周期保留。
static uint8_t tunnel_inside;  // 本次任务是否曾确认整车完全进入；确认后保持为一，直到任务重置。
static uint8_t tunnel_light;  // 最近提交的灯色；必须结合有效标志使用。
static uint8_t tunnel_light_valid;  // 提交灯色是否对本周期有效；使用后清零。
static float tunnel_time;  // 通过灯色和通行许可检查后累计的周期时间，单位秒；暂停等待时不累加。

void App_Tunnel_Reset(void)
{
    tunnel_state = TUNNEL_APPROACH;
    tunnel_inside = 0U;
    tunnel_light = VISION_LIGHT_UNKNOWN;
    tunnel_light_valid = 0U;
    tunnel_time = 0.0f;
    App_Line_Reset();
}
void App_Tunnel_Start(uint8_t route)
{
    (void)route;  // 路线参数暂不参与计算；各路线的隧道判断留待场地测试后补充。
    App_Tunnel_Reset();
}
void App_Tunnel_SetLight(uint8_t color, uint8_t valid)
{
    tunnel_light = color;
    tunnel_light_valid = valid;
}
uint8_t App_Tunnel_IsInside(void)
{
    return tunnel_inside;
}
TaskResult App_Tunnel_Update(const TrackInput *track,
                             ChassisSpeed *target, float dt_s)
{
    // 只有本周期提交有效且灯色为绿，入口前才允许继续接近或驶入。
    uint8_t green = tunnel_light_valid != 0U &&
                    tunnel_light == VISION_LIGHT_GREEN;
    LineResult result;
    tunnel_light_valid = 0U;  // 本周期使用后立即清除有效标记，下一周期必须重新提交，防止旧绿灯一直放行。
    *target = (ChassisSpeed){0};
    if (tunnel_state == TUNNEL_ERROR)
    {
        return TASK_FAILED;
    }
    if (tunnel_state == TUNNEL_END)
    {
        return TASK_FINISHED;
    }
    // 时间和速度未标定时，流程明确等待配置。
    if (CHASSIS_TUNNEL_TIMEOUT_S <= 0.0f || CHASSIS_TUNNEL_SPEED_MPS <= 0.0f)
    {
        return TASK_UNSET;
    }
    if (tunnel_state == TUNNEL_ENTER || tunnel_state == TUNNEL_PASS ||
        tunnel_state == TUNNEL_EXIT)
    {
        // 入内后灯色变化怎么办尚未确定，不在这里猜测策略。
        if (track->tunnel_pass_allowed == TRACK_UNSET)
        {
            return TASK_UNSET;
        }
        if (track->tunnel_pass_allowed != TRACK_YES)
        {
            return TASK_WAIT;
        }
    }
    else if (green == 0U)
    {
        return TASK_WAIT;  // 入口前仅允许本周期有效绿灯放行。
    }
    tunnel_time += dt_s;
    if (tunnel_time >= CHASSIS_TUNNEL_TIMEOUT_S)
    {
        tunnel_state = TUNNEL_ERROR;
        return TASK_FAILED;
    }
    switch (tunnel_state)
    {
        case TUNNEL_APPROACH:
            if (track->tunnel_entry == TRACK_UNSET)
            {
                return TASK_UNSET;
            }
            if (track->tunnel_entry == TRACK_YES)
            {
                tunnel_state = TUNNEL_WAIT;
                App_Line_Reset();
                return TASK_WAIT;
            }
            break;
        case TUNNEL_WAIT:
            if (track->tunnel_pass_allowed == TRACK_UNSET)
            {
                return TASK_UNSET;  // 入内策略未配置，不能先驶入再等待。
            }
            if (track->tunnel_pass_allowed != TRACK_YES)
            {
                return TASK_WAIT;
            }
            tunnel_state = TUNNEL_ENTER;
            break;
        case TUNNEL_ENTER:
            if (track->tunnel_inside == TRACK_UNSET)
            {
                return TASK_UNSET;
            }
            if (track->tunnel_inside == TRACK_YES)
            {
                tunnel_inside = 1U;
                tunnel_state = TUNNEL_PASS;
            }
            break;
        case TUNNEL_PASS:
            tunnel_state = TUNNEL_EXIT;
            break;
        case TUNNEL_EXIT:
            if (track->tunnel_outside == TRACK_UNSET)
            {
                return TASK_UNSET;
            }
            if (track->tunnel_outside == TRACK_YES)
            {
                tunnel_state = TUNNEL_END;
                return TASK_FINISHED;
            }
            break;
        default:
            return TASK_FAILED;
    }
    result = App_Line_Update(target, dt_s, LINE_ACTION_FOLLOW);
    if (result == LINE_TARGET_READY)
    {
        target->vx = CHASSIS_TUNNEL_SPEED_MPS;
    }
    return App_Task_FromLine(result);
}
