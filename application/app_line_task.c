// 循迹任务：保存路线进度，组织断线跨越，复用已有基础循迹。
#include "app_line_task.h"
#include "app_config.h"

typedef enum
{
    LINE_TASK_FOLLOW = 0,  // 正常跟线与路线动作。
    LINE_TASK_CROSS_GAP,   // 计划断线窗口内跨越。
    LINE_TASK_FIND_LINE,   // 暂停跨越，连续确认候选线。
    LINE_TASK_MERGED,      // 已到汇合处，继续跟线等待交接。
    LINE_TASK_END,         // 到达第二段停车点，等待底盘确认停稳。
    LINE_TASK_ERROR        // 本次任务失败，需重新启动后才能继续。
} LineTaskState;
static LineTaskState line_task_state;  // 当前循迹任务所处阶段，跨周期保留。
static uint8_t line_segment;  // 循迹段编号：一为第一段，二为第二段。
static uint8_t line_route;  // 电控b选择的路线编号，取值一至三。
static uint8_t line_step;  // 当前路线动作序号；零表示入口选路，完成一个特征动作后递增。
static uint8_t line_gap_used;  // 当前断线窗口是否已经跨越成功；防止同一窗口重复触发。
static float line_gap_time;  // 本次跨越断线及重新见线确认的累计时间，单位秒。
static uint8_t line_gap_found;  // 连续检测到候选新线的周期数；中途未检测到就清零。
static float line_handoff_time;  // 到达汇合点后等待下一阶段接管的累计时间，单位秒。
// 三条路线的入口动作依次为左转、直行、右转；只读表不会随任务进度改变。
static const LineAction line_entry[3] =
{
    LINE_ACTION_LEFT, LINE_ACTION_STRAIGHT, LINE_ACTION_RIGHT
};

void App_Line_Task_Reset(void)
{
    line_task_state = LINE_TASK_FOLLOW;
    line_segment = 0U;
    line_route = 0U;
    line_step = 0U;
    line_gap_used = 0U;
    line_gap_time = 0.0f;
    line_gap_found = 0U;
    line_handoff_time = 0.0f;
    App_Line_Reset();
}
void App_Line_Task_Start(uint8_t segment, uint8_t route)
{
    App_Line_Task_Reset();
    line_segment = segment;
    line_route = route;
}

// 根据路线和动作序号选择本次动作；通过指针写出动作，返回动作是否已配置。
static TrackCheck App_Line_Task_GetAction(LineAction *action)
{
    if (line_step == 0U)
    {
        *action = line_entry[line_route - 1U];
        return TRACK_YES;
    }
    if (line_route == 1U || line_route == 3U)
    {
        *action = LINE_ACTION_FOLLOW;
        return TRACK_YES;
    }
    return App_Track_GetAction(TRACK_LINE1_ACTION, line_route,
                               line_step, action);
}

// 处理一次计划断线的跨越与重新见线确认；写出速度请求，连续确认成功后恢复普通循迹。
// 后续实车可评估编码器里程方案：开始跨越时记录里程起点，以累计距离限制跨越范围。
// 里程受打滑影响，是否采用及距离阈值待实测；当前尚未实现里程判断，仍按配置速度直行。
// 跨越成功以探头连续确认新线为依据，不以走够距离或时间作为成功依据；时间保留作超时保护。
static TaskResult App_Line_Task_Gap(const TrackInput *track,
                                    ChassisSpeed *target, float dt_s)
{
    // 跨越断线和确认重新见线共用总计时；反复见线、丢线不能重新获得完整超时时间。
    if (track->line_found == TRACK_UNSET ||
        CHASSIS_GAP_TIMEOUT_S <= 0.0f || CHASSIS_GAP_SPEED_MPS <= 0.0f)
    {
        return TASK_UNSET;
    }
    line_gap_time += dt_s;
    if (line_gap_time >= CHASSIS_GAP_TIMEOUT_S)
    {
        line_task_state = LINE_TASK_ERROR;
        return TASK_FAILED;
    }
    if (track->line_found == TRACK_YES)
    {
        line_task_state = LINE_TASK_FIND_LINE;
        if (line_gap_found < CHASSIS_LINE_CONFIRM_COUNT)
        {
            line_gap_found++;
        }
        if (line_gap_found < CHASSIS_LINE_CONFIRM_COUNT)
        {
            return TASK_WAIT;
        }
        line_task_state = LINE_TASK_FOLLOW;
        line_gap_used = 1U;
        App_Line_Reset();
        return App_Task_FromLine(App_Line_Update(target, dt_s,
                                                  LINE_ACTION_FOLLOW));
    }
    line_gap_found = 0U;
    line_task_state = LINE_TASK_CROSS_GAP;
    // 尚未重新见线时，以配置速度直行跨越，并受上面的总超时限制；不推测转向角速度。
    target->vx = CHASSIS_GAP_SPEED_MPS;
    return TASK_MOVE;
}

TaskResult App_Line_Task_Update(const TrackInput *track,
                                ChassisSpeed *target, float dt_s)
{
    LineAction action;
    LineResult result;
    *target = (ChassisSpeed){0};
    switch (line_task_state)
    {
        case LINE_TASK_ERROR:
            return TASK_FAILED;
        case LINE_TASK_END:
            return TASK_FINISHED;
        case LINE_TASK_MERGED:
            if (CHASSIS_HANDOFF_TIMEOUT_S <= 0.0f)
            {
                return TASK_UNSET;
            }
            line_handoff_time += dt_s;
            if (line_handoff_time >= CHASSIS_HANDOFF_TIMEOUT_S)
            {
                line_task_state = LINE_TASK_ERROR;
                return TASK_FAILED;
            }
            result = App_Line_Update(target, dt_s, LINE_ACTION_FOLLOW);
            return result == LINE_FAILED ? TASK_FAILED : TASK_BOUNDARY;
        case LINE_TASK_CROSS_GAP:
        case LINE_TASK_FIND_LINE:
            return App_Line_Task_Gap(track, target, dt_s);
        default:
            break;
    }

    if (line_segment == 1U)
    {
        // 入口选路已确定，可先执行；入口动作完成后，继续行驶前必须有汇合点判断。
        if (line_step > 0U && track->merge == TRACK_UNSET)
        {
            return TASK_UNSET;
        }
        if (line_step > 0U && track->merge == TRACK_YES)
        {
            if (CHASSIS_HANDOFF_TIMEOUT_S <= 0.0f)
            {
                return TASK_UNSET;
            }
            line_task_state = LINE_TASK_MERGED;
            App_Line_Reset();
            result = App_Line_Update(target, dt_s, LINE_ACTION_FOLLOW);
            return result == LINE_FAILED ? TASK_FAILED : TASK_BOUNDARY;
        }
        if (App_Line_Task_GetAction(&action) != TRACK_YES)
        {
            return TASK_UNSET;
        }
        result = App_Line_Update(target, dt_s, action);
        if (result == LINE_FEATURE_DONE && action != LINE_ACTION_FOLLOW &&
            action != LINE_ACTION_WAIT)
        {
            line_step++;
        }
        return App_Task_FromLine(result);
    }

    // 计划断线窗口和终点判断尚未配置时等待；普通丢线不能直接算作计划断线。
    if (track->line2_end == TRACK_UNSET || track->gap_window == TRACK_UNSET)
    {
        return TASK_UNSET;
    }
    if (track->line2_end == TRACK_YES)
    {
        line_task_state = LINE_TASK_END;
        return TASK_FINISHED;
    }
    if (track->gap_window == TRACK_NO)
    {
        line_gap_used = 0U;  // 离开上一个断线窗口后，才允许触发下一个断线跨越。
    }
    if (track->gap_window == TRACK_YES && line_gap_used == 0U)
    {
        if (track->gap_start == TRACK_UNSET)
        {
            return TASK_UNSET;
        }
        if (track->gap_start == TRACK_YES)
        {
            line_task_state = LINE_TASK_CROSS_GAP;
            line_gap_time = 0.0f;
            line_gap_found = 0U;
            App_Line_Reset();
            return App_Line_Task_Gap(track, target, dt_s);
        }
    }
    return App_Task_FromLine(App_Line_Update(target, dt_s, LINE_ACTION_FOLLOW));
}
