// 循迹应用：读取灰度、管理特征，向底盘交回速度请求。
#include "app_line.h"
#include "app_config.h"
#include "line.h"
#include "bsp_gray.h"
#include <stddef.h>
#include <math.h>

// 外侧连续黑色计数跨周期保留，达到确认阈值后不再增加。
static uint8_t line_left_count = 0U;
static uint8_t line_right_count = 0U;

// 五路探头连续全白的次数，只表示没有看到黑线，不判断断线或丢线。
static uint8_t line_white_count = 0U;

// 本次特征处理过程中，是否已确认看到过这一侧黑线。
// 变白只清除连续计数；确认标记保留到本次处理结束或被中断。
static uint8_t line_left_seen = 0U;
static uint8_t line_right_seen = 0U;

// 循迹内部的工作阶段。
typedef enum
{
    LINE_FOLLOW = 0,  // 正常循迹。
    LINE_OBSERVE,     // 已确认外侧特征，继续观察。
    LINE_TURN,        // 已确认需要转弯，正在执行。
    LINE_RECOVER,     // 普通循迹丢线，有限时找线。
    LINE_ERROR        // 本次循迹处理失败，等待任务复位。
} LineState;

// 最后一次有效偏差所指向的方向，不代表路口的选路指令。
typedef enum
{
    LINE_DIR_NONE = 0,  // 尚未得到明确方向。
    LINE_DIR_LEFT,      // 黑线偏左。
    LINE_DIR_RIGHT      // 黑线偏右。
} LineDirection;

// 保留到下个周期，供后续丢线恢复参考。
static LineDirection line_last_direction = LINE_DIR_NONE;

// 保存当前阶段，下一个周期继续使用。
static LineState line_state = LINE_FOLLOW;

// 已进入观察阶段的时间，单位为秒，每个周期累加实际间隔。
static float line_observe_time = 0.0f;

// 正常循迹持续无法辨清线路的时间，全白由独立找线流程处理。
static float line_unclear_time = 0.0f;

// 观察阶段中间三路连续无黑的次数；与五路全白计数分开。
static uint8_t line_observe_lost_count = 0U;

// 要求直行时，连续恢复清晰中间线的次数。
static uint8_t line_observe_clear_count = 0U;

// 本次已经确定的转弯方向，转弯过程中保持不变。
static LineDirection line_turn_direction = LINE_DIR_NONE;

// 本次转弯已经经过的时间，单位秒，供后续超时保护使用。
static float line_turn_time = 0.0f;

// 转弯后连续看到清晰线路的次数，供后续确认转弯完成。
static uint8_t line_turn_found_count = 0U;

// 直角初版的过程标记：本次转弯中，中间三路是否曾全部无黑。
// 不代表所有弯道都必须出现全白，也不能单凭它判断选路正确。
static uint8_t line_turn_lost_middle = 0U;

// 找线计时与连续找回次数，和转弯的计数互不混用。
static float line_recover_time = 0.0f;
static uint8_t line_recover_found_count = 0U;

void App_Line_Reset(void)
{
    line_left_count = 0U;
    line_right_count = 0U;
    line_white_count = 0U;
    line_left_seen = 0U;
    line_right_seen = 0U;
    line_state = LINE_FOLLOW;
    line_recover_time = 0.0f;
    line_recover_found_count = 0U;
    line_observe_time = 0.0f;
    line_unclear_time = 0.0f;
    line_observe_lost_count = 0U;
    line_observe_clear_count = 0U;
    line_last_direction = LINE_DIR_NONE;
    line_turn_direction = LINE_DIR_NONE;
    line_turn_time = 0.0f;
    line_turn_found_count = 0U;
    line_turn_lost_middle = 0U;
}

// 将五路原始电平整理成循迹算法使用的黑白结果。
// 暂按GRAY_1到GRAY_5对应左到右、高电平表示黑色，实物到位后核对。
static uint8_t App_Line_Read(LineInput *input)
{
    uint8_t level[LINE_SENSOR_COUNT];  // 暂存五路原始电平。
    uint8_t sensor_index;              // 当前处理的探头编号，依次为0到4。

    if (input == NULL)
    {
        return 0;
    }

    // 先标记本次数据尚未准备好。
    input->sample_valid = 0U;

    if (BSP_Gray_Read(level) == 0)
    {
        return 0;
    }

    for (sensor_index = 0U;
         sensor_index < LINE_SENSOR_COUNT;
         sensor_index++)
    {
        // 把当前探头的电平保存到对应的黑白结果中。
        input->black[sensor_index] = level[sensor_index];
    }

    // 表示完成了软件读取，不代表已检测到黑线或硬件连接正常。
    input->sample_valid = 1U;
    return 1;
}

// 普通循迹时更新速度目标。
// 调用前由任务逻辑排除停线、路口等特殊图案。
// 返回1表示成功生成目标；返回0表示本次不能循迹。
static uint8_t App_Line_Calc_Target(const LineInput *input, ChassisSpeed *target)
{
    float offset_mm;  // 本次检测到的线路偏移，左正右负。

    // 先清除旧目标，避免本次计算失败后继续沿用旧命令。
    target->vx = 0.0f;
    target->vy = 0.0f;
    target->wz = 0.0f;

    if (Line_Calc_Offset(input, &offset_mm) == 0)
    {
        return 0;
    }

    // 当前偏差约定：左正右负；居中时保留之前的方向线索。
    if (offset_mm > 0.0f)
    {
        line_last_direction = LINE_DIR_LEFT;
    }
    else if (offset_mm < 0.0f)
    {
        line_last_direction = LINE_DIR_RIGHT;
    }

    // 一边前进，一边根据线路偏移转向。
    target->vx = CHASSIS_LINE_FORWARD_MPS;
    target->wz = Line_Calc_Turn(
        offset_mm,
        CHASSIS_LINE_KP,
        CHASSIS_LINE_MAX_TURN_RPS);

    return 1;
}

// 更新两侧探头的连续黑色计数，并记住已确认的特征。
// 调用者保证pattern来自本周期成功的图案分析。
static void App_Line_Update_Feature(const LinePattern *pattern)
{
    if (pattern->left_black == 0U)
    {
        line_left_count = 0U;
    }
    else if (line_left_count < CHASSIS_LINE_CONFIRM_COUNT)
    {
        line_left_count++;
    }

    if (pattern->right_black == 0U)
    {
        line_right_count = 0U;
    }
    else if (line_right_count < CHASSIS_LINE_CONFIRM_COUNT)
    {
        line_right_count++;
    }

    // 连续检测达到阈值后记住，不因下一周期变白而清除。
    if (line_left_count >= CHASSIS_LINE_CONFIRM_COUNT)
    {
        line_left_seen = 1U;
    }

    if (line_right_count >= CHASSIS_LINE_CONFIRM_COUNT)
    {
        line_right_seen = 1U;
    }

}

// 只负责开始一次已经确认的转弯，不负责判断是不是转角。
// direction由观察阶段结合特征和路线要求确定。
static uint8_t App_Line_Begin_Turn(LineDirection direction)
{
    // 方向没有确定，就不进入转弯状态。
    if (direction != LINE_DIR_LEFT &&
        direction != LINE_DIR_RIGHT)
    {
        return 0U;
    }

    // 保存本次方向，避免后续普通偏差改变转弯决定。
    line_turn_direction = direction;

    // 每次开始新的转弯，都重新计时、重新确认新线。
    line_turn_time = 0.0f;
    line_turn_found_count = 0U;
    line_turn_lost_middle = 0U;

    line_state = LINE_TURN;
    return 1U;
}

// 执行观察阶段的一个周期。
// 路线要求提供方向；外侧特征和中间无黑确认共同决定直角转弯入口。
static LineResult App_Line_Observe(const LineInput *input,
                                  const LinePattern *pattern,
                                  ChassisSpeed *target,
                                  float dt_s,
                                  LineAction action)
{
    LineDirection direction = LINE_DIR_NONE;  // 本周期允许开始的转向。

    line_observe_time += dt_s;

    // 尚未设置时限，或观察超过时限，都结束本次处理。
    // Update已将目标清零；失败保持到任务显式调用Reset。
    if (CHASSIS_LINE_OBSERVE_TIMEOUT_S <= 0.0f ||
        line_observe_time >= CHASSIS_LINE_OBSERVE_TIMEOUT_S)
    {
        line_state = LINE_ERROR;
        return LINE_FAILED;
    }

    // 中间三路连续无黑才确认已离开中间线；任一路恢复黑色就重新计数。
    if (pattern->middle_count != 0U)
    {
        line_observe_lost_count = 0U;
    }
    else if (line_observe_lost_count < CHASSIS_LINE_CONFIRM_COUNT)
    {
        line_observe_lost_count++;
    }

    // 只有要求直行且图案清晰才累计；图案中断或动作变化都重新确认。
    if (action != LINE_ACTION_STRAIGHT || Line_Is_Clear(pattern) == 0U)
    {
        line_observe_clear_count = 0U;
    }
    else if (line_observe_clear_count < CHASSIS_LINE_CONFIRM_COUNT)
    {
        line_observe_clear_count++;
    }

    // 动作来自路线安排，探头负责确认时机，不靠外侧探头自行选路。
    switch (action)
    {
        case LINE_ACTION_WAIT:
        {
            return LINE_NO_TARGET;
        }

        case LINE_ACTION_LEFT:
        {
            if (line_left_seen != 0U)
            {
                direction = LINE_DIR_LEFT;
            }
            break;
        }

        case LINE_ACTION_RIGHT:
        {
            if (line_right_seen != 0U)
            {
                direction = LINE_DIR_RIGHT;
            }
            break;
        }

        case LINE_ACTION_STRAIGHT:
        {
            break;
        }

        default:
        {
            return LINE_FAILED;
        }
    }

    if (direction != LINE_DIR_NONE &&
        line_observe_lost_count >= CHASSIS_LINE_CONFIRM_COUNT)
    {
        App_Line_Begin_Turn(direction);
        // 入口已经确认中间无黑，交给转弯阶段继续记住，避免遗漏这段过程。
        line_turn_lost_middle = 1U;
        // 本周期只切换阶段，下一周期由App_Line_Turn生成旋转请求。
        return LINE_NO_TARGET;
    }

    if (pattern->middle_count == 0U ||
        pattern->middle_count == 3U)
    {
        return LINE_NO_TARGET;
    }

    // 中间的101不是一段连续黑线，暂不给纠偏请求。
    if (input->black[1] != 0U &&
        input->black[2] == 0U &&
        input->black[3] != 0U)
    {
        return LINE_NO_TARGET;
    }

    if (line_observe_clear_count >= CHASSIS_LINE_CONFIRM_COUNT)
    {
        // 完成一处特征，继续按当前线路生成速度，不插入零请求。
        App_Line_Reset();
        if (App_Line_Calc_Target(input, target) == 0U)
        {
            return LINE_NO_TARGET;
        }
        return LINE_FEATURE_DONE;
    }

    if (App_Line_Calc_Target(input, target) == 0U)
    {
        return LINE_NO_TARGET;
    }

    // 确认期间保持观察速度；恢复正常阶段后使用Calc_Target给出的正常速度。
    if (line_state == LINE_OBSERVE)
    {
        target->vx = CHASSIS_LINE_OBSERVE_MPS;
    }
    return LINE_TARGET_READY;
}

// 执行已确认转弯的一个周期。
// 调用前保证采样、dt_s有效，且target已经清零。
static LineResult App_Line_Turn(const LineInput *input,
                               const LinePattern *pattern,
                               ChassisSpeed *target,
                               float dt_s)
{
    if (CHASSIS_LINE_TURN_RPS <= 0.0f ||
        CHASSIS_LINE_TURN_TIMEOUT_S <= 0.0f)
    {
        return LINE_FAILED;
    }

    line_turn_time += dt_s;

    if (line_turn_time >= CHASSIS_LINE_TURN_TIMEOUT_S)
    {
        return LINE_FAILED;
    }

    // 中间三路无黑，记住已离开原来的中间线，不要求外侧也全白。
    if (pattern->middle_count == 0U)
    {
        line_turn_lost_middle = 1U;
    }

    // 使用现有的black[2]读取正中探头，不增加虚构的图案字段。
    if (line_turn_lost_middle != 0U &&
        Line_Is_Clear(pattern) != 0U &&
        input->black[2] != 0U)
    {
        if (line_turn_found_count < CHASSIS_LINE_CONFIRM_COUNT)
        {
            line_turn_found_count++;
        }
    }
    else
    {
        line_turn_found_count = 0U;
    }

    // 看到候选线路后先给零请求，连续确认，避免继续旋转越过线路。
    // 这是零速度目标，实际减速仍受上层斜坡和电机控制影响。
    if (line_turn_found_count != 0U)
    {
        if (line_turn_found_count >= CHASSIS_LINE_CONFIRM_COUNT)
        {
            // 转弯对线已连续确认；完成反馈同时携带当前循迹目标。
            App_Line_Reset();
            if (App_Line_Calc_Target(input, target) == 0U)
            {
                return LINE_NO_TARGET;
            }
            return LINE_FEATURE_DONE;
        }

        return LINE_NO_TARGET;
    }

    switch (line_turn_direction)
    {
        case LINE_DIR_LEFT:
        {
            target->wz = CHASSIS_LINE_TURN_RPS;
            break;
        }

        case LINE_DIR_RIGHT:
        {
            target->wz = -CHASSIS_LINE_TURN_RPS;
            break;
        }

        default:
        {
            return LINE_FAILED;
        }
    }

    return LINE_TARGET_READY;
}
// 普通丢线只尝试沿最后有效偏差方向原地找线，不猜测断线或路口。
static LineResult App_Line_Recover(const LineInput *input,
                                   const LinePattern *pattern,
                                   ChassisSpeed *target,
                                   float dt_s)
{
    line_recover_time += dt_s;
    if (CHASSIS_LINE_RECOVER_TIMEOUT_S <= 0.0f ||
        line_recover_time >= CHASSIS_LINE_RECOVER_TIMEOUT_S)
    {
        line_state = LINE_ERROR;
        return LINE_FAILED;
    }

    // 先暂停旋转确认候选线，单次闪烁不能退出恢复。
    if (Line_Is_Clear(pattern) != 0U && input->black[2] != 0U)
    {
        line_recover_found_count++;
        if (line_recover_found_count >= CHASSIS_LINE_CONFIRM_COUNT)
        {
            App_Line_Reset();
            App_Line_Calc_Target(input, target);
            return LINE_TARGET_READY;  // 找回线路，不消费路线动作。
        }
        return LINE_NO_TARGET;
    }
    line_recover_found_count = 0U;

    if (line_last_direction == LINE_DIR_NONE)
    {
        return LINE_NO_TARGET;  // 无方向证据时保持零请求，等待找回或超时。
    }
    if (CHASSIS_LINE_RECOVER_RPS <= 0.0f)
    {
        line_state = LINE_ERROR;
        return LINE_FAILED;
    }

    target->wz = line_last_direction == LINE_DIR_LEFT ?
                 CHASSIS_LINE_RECOVER_RPS : -CHASSIS_LINE_RECOVER_RPS;
    return LINE_TARGET_READY;
}

LineResult App_Line_Update(ChassisSpeed *target,
                           float dt_s,
                           LineAction action)
{
    LineInput input = {0};      // 本周期的五路检测结果。
    LinePattern pattern = {0};  // 黑色数量、分组和外侧读数。
    uint8_t observe_entered = 0U;  // 本周期是否刚进入观察。

    if (target == NULL)
    {
        return LINE_NO_TARGET;
    }

    // 每周期先给出零请求，防止失败后沿用旧目标。
    *target = (ChassisSpeed){0};

    // 循迹处理失败后不因下一次采样自动重新启动。
    if (line_state == LINE_ERROR)
    {
        return LINE_FAILED;
    }

    // 时间间隔必须是有效的正数，否则清除本次循迹记录。
    if (!isfinite(dt_s) || dt_s <= 0.0f)
    {
        App_Line_Reset();
        return LINE_NO_TARGET;
    }

    if (App_Line_Read(&input) == 0U ||
        Line_Analyze(&input, &pattern) == 0U)
    {
        App_Line_Reset();
        return LINE_NO_TARGET;
    }

    // 连续循迹时不锁存外侧路口标记；正在执行的转弯仍按原流程完成。
    if (action == LINE_ACTION_FOLLOW && line_state == LINE_FOLLOW)
    {
        line_left_count = 0U;
        line_right_count = 0U;
        line_left_seen = 0U;
        line_right_seen = 0U;
    }
    else
    {
        App_Line_Update_Feature(&pattern);
    }

    // 全白连续确认；任意探头看到黑线，就重新开始计数。
    if (pattern.black_count == 0U)
    {
        if (line_white_count < CHASSIS_LINE_CONFIRM_COUNT)
        {
            line_white_count++;
        }
    }
    else
    {
        line_white_count = 0U;
    }
    // 任意一侧确认出现特征，进入观察阶段。
    if (line_state == LINE_FOLLOW &&
        action != LINE_ACTION_FOLLOW &&
        (line_left_seen != 0U || line_right_seen != 0U))
    {
        line_state = LINE_OBSERVE;
        line_observe_time = 0.0f;  // 刚进入观察，从零开始计时。
        line_unclear_time = 0.0f;
        line_observe_lost_count = 0U;
        line_observe_clear_count = 0U;
        observe_entered = 1U;
    }
    // 按当前阶段生成速度请求。
    switch (line_state)
    {
        case LINE_FOLLOW:
        {
            // 只在正常循迹中将连续全白视为普通丢线。
            // OBSERVE和TURN有各自的图案处理，不在这里抢占。
            if (line_white_count >= CHASSIS_LINE_CONFIRM_COUNT)
            {
                line_unclear_time = 0.0f;
                line_state = LINE_RECOVER;
                line_recover_time = 0.0f;
                line_recover_found_count = 0U;
                return App_Line_Recover(&input, &pattern, target, 0.0f);
            }

            uint8_t clear;  // 本周期是否有可用于计算偏差的线路。

            // 全白先等待连续确认，之后由上面的RECOVER入口处理。
            if (pattern.black_count == 0U)
            {
                line_unclear_time = 0.0f;
                return LINE_NO_TARGET;
            }

            if (action == LINE_ACTION_FOLLOW)
            {
                // 缓弯允许外侧同时触发；中间须有1或2个连续黑色。
                clear = Line_Has_Middle(&input);
            }
            else
            {
                clear = Line_Is_Clear(&pattern);
            }

            if (clear != 0U && App_Line_Calc_Target(&input, target) != 0U)
            {
                line_unclear_time = 0.0f;
                return LINE_TARGET_READY;
            }

            // 模糊图案之间切换也算持续不清楚，不反复重置等待时间。
            line_unclear_time += dt_s;
            if (CHASSIS_LINE_UNCLEAR_TIMEOUT_S <= 0.0f ||
                line_unclear_time >= CHASSIS_LINE_UNCLEAR_TIMEOUT_S)
            {
                line_state = LINE_ERROR;
                return LINE_FAILED;
            }
            return LINE_NO_TARGET;
        }

        case LINE_OBSERVE:
        {
            // 刚进入观察时从零计时，后续周期再累加实际间隔。
            return App_Line_Observe(
                &input, &pattern, target,
                observe_entered != 0U ? 0.0f : dt_s, action);
        }

        case LINE_RECOVER:
        {
            return App_Line_Recover(&input, &pattern, target, dt_s);
        }

        case LINE_TURN:
        {
            return App_Line_Turn(&input, &pattern, target, dt_s);
        }

        default:
        {
            return LINE_NO_TARGET;
        }
    }
}