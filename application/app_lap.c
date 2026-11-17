// 绕圈流程：接近入口、入环、绕行、出环、连续交接。
#include "app_lap.h"
#include "app_config.h"
typedef enum
{
    LAP_APPROACH = 0,  // 汇合后接近圆环。
    LAP_ENTER,         // 执行已配置的入环动作。
    LAP_FOLLOW,        // 沿圆环走，等待独立满圈证据。
    LAP_EXIT,          // 满圈后选择出口。
    LAP_HANDOFF,       // 出环后保持循迹等待下一任务。
    LAP_ERROR          // 超时或循迹失败，等待重新启动。
} LapState;
static LapState lap_state;  // 当前绕圈阶段，跨周期保留。
static uint8_t lap_route;  // 本次任务使用的路线编号，供入环、出环动作查询。
static uint8_t lap_exit_aligned;  // 出环动作是否已经完成；完成后等待驶出确认。
static float lap_time;  // 本次绕圈任务累计时间，含接近入口和等待交接，单位秒。
static float lap_handoff_time;  // 出环后等待下一阶段接管的累计时间，单位秒。

void App_Lap_Reset(void)
{
    lap_state = LAP_APPROACH;
    lap_route = 0U;
    lap_exit_aligned = 0U;
    lap_time = 0.0f;
    lap_handoff_time = 0.0f;
    App_Line_Reset();
}
void App_Lap_Start(uint8_t route)
{
    App_Lap_Reset();
    lap_route = route;
}
TaskResult App_Lap_Update(const TrackInput *track,
                          ChassisSpeed *target, float dt_s)
{
    LineAction action = LINE_ACTION_FOLLOW;
    LineResult result;
    *target = (ChassisSpeed){0};
    if (lap_state == LAP_ERROR)
    {
        return TASK_FAILED;
    }
    if (CHASSIS_LAP_TIMEOUT_S <= 0.0f)
    {
        return TASK_UNSET;
    }
    lap_time += dt_s;
    if (lap_time >= CHASSIS_LAP_TIMEOUT_S)
    {
        lap_state = LAP_ERROR;
        return TASK_FAILED;
    }
    switch (lap_state)
    {
        case LAP_APPROACH:
            if (track->lap_entry == TRACK_UNSET)
            {
                return TASK_UNSET;
            }
            if (track->lap_entry == TRACK_YES)
            {
                lap_state = LAP_ENTER;
                App_Line_Reset();
            }
            else
            {
                break;
            }
            // 本周期继续处理入环动作，避免仅切换状态却没有生成速度请求。
            /* 本分支不退出，继续执行紧接着的下一分支。 */
            /* fall through */
        case LAP_ENTER:
            if (App_Track_GetAction(TRACK_LAP_ENTER_ACTION, lap_route,
                                    0U, &action) != TRACK_YES)
            {
                return TASK_UNSET;
            }
            result = App_Line_Update(target, dt_s, action);
            if (result == LINE_FEATURE_DONE)
            {
                lap_state = LAP_FOLLOW;
            }
            return App_Task_FromLine(result);
        case LAP_FOLLOW:
            if (track->lap_full == TRACK_UNSET)
            {
                return TASK_UNSET;
            }
            if (track->lap_full == TRACK_YES)
            {
                lap_state = LAP_EXIT;
                App_Line_Reset();
            }
            else
            {
                break;
            }
            /* 本分支不退出，继续执行紧接着的下一分支。 */
            /* fall through */
        case LAP_EXIT:
            if (track->lap_exit == TRACK_UNSET)
            {
                return TASK_UNSET;
            }
            if (lap_exit_aligned == 0U)
            {
                if (App_Track_GetAction(TRACK_LAP_EXIT_ACTION, lap_route,
                                        0U, &action) != TRACK_YES)
                {
                    return TASK_UNSET;
                }
                result = App_Line_Update(target, dt_s, action);
                if (result == LINE_FEATURE_DONE)
                {
                    lap_exit_aligned = 1U;
                }
                return App_Task_FromLine(result);
            }
            if (track->lap_exit == TRACK_YES)
            {
                if (CHASSIS_HANDOFF_TIMEOUT_S <= 0.0f)
                {
                    return TASK_UNSET;
                }
                lap_state = LAP_HANDOFF;
            }
            break;
        case LAP_HANDOFF:
            if (CHASSIS_HANDOFF_TIMEOUT_S <= 0.0f)
            {
                return TASK_UNSET;
            }
            lap_handoff_time += dt_s;
            if (lap_handoff_time >= CHASSIS_HANDOFF_TIMEOUT_S)
            {
                lap_state = LAP_ERROR;
                return TASK_FAILED;
            }
            break;
        default:
            return TASK_FAILED;
    }
    result = App_Line_Update(target, dt_s, LINE_ACTION_FOLLOW);
    if (result == LINE_FAILED)
    {
        lap_state = LAP_ERROR;
        return TASK_FAILED;
    }
    return lap_state == LAP_HANDOFF ? TASK_BOUNDARY : App_Task_FromLine(result);
}
