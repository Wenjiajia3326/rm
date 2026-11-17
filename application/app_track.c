// 现场判断集中预留：目前不读探头、不推测图案、不报告检测成功。
#include "app_track.h"

void App_Track_Read(TrackInput *input)
{
    // 待补充：根据场地实测填写判断；目前全部置为尚未配置，不把全黑直接当成进入隧道。
    *input = (TrackInput){0};
}

TrackCheck App_Track_GetAction(TrackAction part, uint8_t route,
                               uint8_t step, LineAction *action)
{
    // 待补充：路线二后续动作及入环、出环方向；下面暂不使用这些预留参数，只返回等待动作。
    (void)part;
    (void)route;
    (void)step;
    *action = LINE_ACTION_WAIT;
    return TRACK_UNSET;
}
