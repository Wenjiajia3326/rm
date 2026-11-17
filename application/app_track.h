// 注意：本接口当前全部为占位实现，实际检测与动作表待场地实测后补充。
// 现场检测结果接口：只声明要观察什么，不实现怎么检测。
// 具体检测方法留待场地测试后填写，目前只提供流程所需的结果接口。
#ifndef APP_TRACK_H
#define APP_TRACK_H
#include <stdint.h>
#include "app_line.h"
typedef enum
{
    TRACK_UNSET = 0,  // 尚未实现判断，与没有检测到不同。
    TRACK_NO,         // 判断已实现，本周期不满足。
    TRACK_YES         // 本周期确认满足。
} TrackCheck;

typedef struct
{
    TrackCheck merge;           // 汇合点是否确认。
    TrackCheck gap_window;      // 是否处于允许计划断线的窗口。
    TrackCheck gap_start;       // 是否开始跨越断线。
    TrackCheck line_found;      // 是否看到候选新线，流程再连续确认。
    TrackCheck line2_end;       // 第二段真正停车点。
    TrackCheck lap_entry;  // 是否到达圆环入口。
    TrackCheck lap_full;        // 独立证据是否确认完整一圈。
    TrackCheck lap_exit;  // 是否确认沿出口离开圆环。
    TrackCheck tunnel_entry;  // 是否到达隧道入口参考位置。
    TrackCheck tunnel_inside;  // 是否确认整车完全进入隧道。
    TrackCheck tunnel_outside;  // 是否确认整车完全驶出隧道。
    TrackCheck tunnel_pass_allowed;  // 入隧道后的通行策略，尚待规则和B确认。
} TrackInput;

typedef enum
{
    TRACK_LINE1_ACTION = 0,  // 查询第一段循迹当前步骤的动作。
    TRACK_LAP_ENTER_ACTION,  // 查询入环动作。
    TRACK_LAP_EXIT_ACTION  // 查询出环动作。
} TrackAction;

// 每个自动控制周期调用一次，写入本周期观察结果；尚未实现判断与本次未检测到必须区分。
void App_Track_Read(TrackInput *input);
// 查询指定路线、指定步骤的动作，写入动作指针；返回尚未配置、未满足或已确认三种结果。
TrackCheck App_Track_GetAction(TrackAction part, uint8_t route,
                               uint8_t step, LineAction *action);
#endif
