// 电脑端流程模拟：用模拟结果代替现场判断和基础循迹，不连接真实电机硬件。
#include <assert.h>
#include <stdio.h>
#include "app_config.h"
#undef CHASSIS_HANDOFF_TIMEOUT_S
#define CHASSIS_HANDOFF_TIMEOUT_S 0.5f
#undef CHASSIS_GAP_TIMEOUT_S
#define CHASSIS_GAP_TIMEOUT_S 0.3f
#undef CHASSIS_GAP_SPEED_MPS
#define CHASSIS_GAP_SPEED_MPS 0.1f
#undef CHASSIS_LAP_TIMEOUT_S
#define CHASSIS_LAP_TIMEOUT_S 2.0f
#undef CHASSIS_TUNNEL_TIMEOUT_S
#define CHASSIS_TUNNEL_TIMEOUT_S 1.0f
#undef CHASSIS_TUNNEL_SPEED_MPS
#define CHASSIS_TUNNEL_SPEED_MPS 0.1f
#undef CHASSIS_STOP_CONFIRM_S
#define CHASSIS_STOP_CONFIRM_S 0.03f
#undef CHASSIS_STOP_WHEEL_RPS
#define CHASSIS_STOP_WHEEL_RPS 0.1f
#include "app_line_task.c"
#include "app_lap.c"
#include "app_tunnel.c"
#include "app_chassis.c"
static TrackInput observations;
static LineResult line_result = LINE_TARGET_READY;
static LineAction last_action;
static TrackCheck action_ready = TRACK_YES;
void App_Line_Reset(void) {}
LineResult App_Line_Update(ChassisSpeed *target, float dt, LineAction action)
{
    (void)dt;
    last_action = action;
    *target = (ChassisSpeed){0};
    if (line_result == LINE_TARGET_READY || line_result == LINE_FEATURE_DONE)
        target->vx = 0.2f;
    return line_result;
}
void App_Track_Read(TrackInput *input) { *input = observations; }
TrackCheck App_Track_GetAction(TrackAction part, uint8_t route,
                               uint8_t step, LineAction *action)
{
    (void)part; (void)route; (void)step;
    *action = LINE_ACTION_LEFT;
    return action_ready;
}
static unsigned nonzero_outputs;
static uint8_t output_fail;
uint8_t BSP_Motor_Read_Count(BSPMotorId id, uint16_t *count)
{ (void)id; *count = 0U; return 1U; }
uint8_t BSP_Motor_Set_Output(BSPMotorId id, int16_t duty)
{ (void)id; if (duty != 0) nonzero_outputs++; return !output_fail; }
uint8_t BSP_Motor_Output_Init(void) { return 1U; }
uint8_t BSP_Motor_Encoder_Init(void) { return 1U; }
uint8_t BSP_Motor_Get_Encoder_Range(uint32_t *range)
{ *range = 65536U; return 1U; }
static void prepare(void)
{
    App_Chassis_Init();
    chassis_status = CHASSIS_STATUS_IDLE;
    chassis_config = (ChassisConfig){{1,1,1,1},{0.05f,0.1f,0.1f},100,65536U,1000U};
    assert(App_Chassis_Init_PI(1,1,100,-1000,1000));
    observations = (TrackInput){0};
    line_result = LINE_TARGET_READY;
    action_ready = TRACK_YES;
}
static void test_wheel_control(void)
{
    ChassisEncoder count = {100U, 200U, 300U, 400U};
    ChassisOutput output;
    const float dt_s = 0.125f;

    App_Chassis_Init();
    assert(App_Chassis_Init_PI(0.0f, 2.0f, 100.0f, -1000.0f, 1000.0f));
    assert(App_Chassis_Update_Feedback(&count, 65536U, 1000U, dt_s) == MOTOR_SAMPLE_READY);
    assert(App_Chassis_Update_Feedback(&count, 65536U, 1000U, dt_s) == MOTOR_SAMPLE_VALID);

    // 只让左前轮积累误差，其他三轮的积分应保持为零。
    wheel_target.front_left = 4.0f;
    assert(App_Chassis_Calc_Output(dt_s, &output));
    assert(output.front_left == 1.0f);
    assert(output.front_right == 0.0f && output.rear_left == 0.0f && output.rear_right == 0.0f);
    assert(App_Chassis_Update_Feedback(&count, 65536U, 1000U, dt_s) == MOTOR_SAMPLE_VALID);
    assert(App_Chassis_Calc_Output(dt_s, &output) && output.front_left == 2.0f);
    App_Chassis_Stop(&output);
    assert(output.front_left == 0.0f);
    wheel_target.front_left = 4.0f;
    assert(App_Chassis_Update_Feedback(&count, 65536U, 1000U, dt_s) == MOTOR_SAMPLE_VALID);
    assert(App_Chassis_Calc_Output(dt_s, &output) && output.front_left == 1.0f);

    // 漏采样时清除旧起点；恢复后的第一拍只记数，第二拍才能算轮速。
    assert(App_Chassis_Update_Feedback(NULL, 65536U, 1000U, dt_s) == MOTOR_SAMPLE_ERROR);
    count.front_left += 20U;
    assert(App_Chassis_Update_Feedback(&count, 65536U, 1000U, dt_s) == MOTOR_SAMPLE_READY);
    assert(App_Chassis_Update_Feedback(&count, 65536U, 1000U, dt_s) == MOTOR_SAMPLE_VALID);
    assert(wheel_feedback.front_left == 0.0f);
}

int main(void)
{
    test_wheel_control();
    ChassisSpeed target;
    TrackInput t = {0};
    // 路线测试：当前只填写已确认的入口选路动作。
    for (uint8_t route = 1; route <= 3; route++)
    {
        App_Line_Task_Start(1,route);
        line_result = LINE_FEATURE_DONE;
        assert(App_Line_Task_Update(&t,&target,0.01f) == TASK_MOVE);
        assert(last_action == (route == 1 ? LINE_ACTION_LEFT :
                              route == 2 ? LINE_ACTION_STRAIGHT : LINE_ACTION_RIGHT));
        assert(line_step == 1);
        line_result = LINE_TARGET_READY;
        assert(App_Line_Task_Update(&t,&target,0.01f) == TASK_UNSET);
        t.merge = TRACK_NO;
        assert(App_Line_Task_Update(&t,&target,0.01f) == TASK_MOVE);
        t.merge = TRACK_YES;
        assert(App_Line_Task_Update(&t,&target,0.01f) == TASK_BOUNDARY);
        assert(target.vx == 0.2f);
        assert(App_Line_Task_Update(&t,&target,0.51f) == TASK_FAILED);
        t = (TrackInput){0};
    }
    App_Line_Task_Start(2,1);
    assert(App_Line_Task_Update(&t,&target,0.01f) == TASK_UNSET);
    t.line2_end = TRACK_NO; t.gap_window = TRACK_YES;
    t.gap_start = TRACK_YES; t.line_found = TRACK_NO;
    assert(App_Line_Task_Update(&t,&target,0.01f) == TASK_MOVE && target.vx == 0.1f);
    t.line_found = TRACK_YES;
    assert(App_Line_Task_Update(&t,&target,0.01f) == TASK_WAIT);
    t.line_found = TRACK_NO;
    assert(App_Line_Task_Update(&t,&target,0.01f) == TASK_MOVE);
    t.line_found = TRACK_YES;
    for (unsigned i = 1; i < CHASSIS_LINE_CONFIRM_COUNT; i++)
        assert(App_Line_Task_Update(&t,&target,0.01f) == TASK_WAIT);
    assert(App_Line_Task_Update(&t,&target,0.01f) == TASK_MOVE);
    assert(line_gap_used == 1);
    assert(App_Line_Task_Update(&t,&target,0.01f) == TASK_MOVE && target.vx == 0.2f);
    t.line2_end = TRACK_YES;
    assert(App_Line_Task_Update(&t,&target,0.01f) == TASK_FINISHED);
    App_Line_Task_Start(2,1); t.line2_end = TRACK_NO; t.line_found = TRACK_NO;
    App_Line_Task_Update(&t,&target,0.01f);
    assert(App_Line_Task_Update(&t,&target,0.31f) == TASK_FAILED);
    // 绕圈测试：依次检查入环、独立满圈证据、出环和继续行驶交接。
    App_Lap_Start(1); t = (TrackInput){0};
    assert(App_Lap_Update(&t,&target,0.01f) == TASK_UNSET);
    t.lap_entry = TRACK_YES; line_result = LINE_FEATURE_DONE;
    assert(App_Lap_Update(&t,&target,0.01f) == TASK_MOVE);
    assert(lap_state == LAP_FOLLOW && target.vx == 0.2f);
    line_result = LINE_TARGET_READY; t.lap_full = TRACK_NO;
    t.lap_exit = TRACK_YES;
    assert(App_Lap_Update(&t,&target,0.01f) == TASK_MOVE && lap_state == LAP_FOLLOW);
    t.lap_full = TRACK_YES; line_result = LINE_FEATURE_DONE;
    assert(App_Lap_Update(&t,&target,0.01f) == TASK_MOVE && lap_exit_aligned);
    line_result = LINE_TARGET_READY;
    assert(App_Lap_Update(&t,&target,0.01f) == TASK_BOUNDARY && target.vx == 0.2f);
    // 隧道测试：本次有效绿灯、曾经完全入内、已经完全驶出是三个不同结果。
    App_Tunnel_Start(1); t = (TrackInput){0};
    App_Tunnel_SetLight(VISION_LIGHT_GREEN,1);
    assert(App_Tunnel_Update(&t,&target,0.01f) == TASK_UNSET);
    t.tunnel_entry = TRACK_YES;
    App_Tunnel_SetLight(VISION_LIGHT_RED,1);
    assert(App_Tunnel_Update(&t,&target,0.01f) == TASK_WAIT);
    App_Tunnel_SetLight(VISION_LIGHT_GREEN,1);
    assert(App_Tunnel_Update(&t,&target,0.01f) == TASK_WAIT);
    assert(App_Tunnel_Update(&t,&target,0.01f) == TASK_WAIT);
    App_Tunnel_SetLight(VISION_LIGHT_GREEN,1);
    assert(App_Tunnel_Update(&t,&target,0.01f) == TASK_UNSET);
    t.tunnel_pass_allowed = TRACK_YES;
    App_Tunnel_SetLight(VISION_LIGHT_GREEN,1);
    assert(App_Tunnel_Update(&t,&target,0.01f) == TASK_MOVE);
    t.tunnel_pass_allowed = TRACK_UNSET;
    // 入隧道后的通行许可由测试显式提供，正式固件暂未实现该判断。
    assert(App_Tunnel_Update(&t,&target,0.01f) == TASK_UNSET);
    t.tunnel_pass_allowed = TRACK_YES;
    t.tunnel_inside = TRACK_YES;
    App_Tunnel_SetLight(VISION_LIGHT_GREEN,1);
    App_Tunnel_Update(&t,&target,0.01f);
    assert(App_Tunnel_IsInside() && tunnel_state == TUNNEL_PASS);
    for (unsigned i = 0; i < 2; i++)
    {
        App_Tunnel_SetLight(VISION_LIGHT_GREEN,1);
        if (i == 0) assert(App_Tunnel_Update(&t,&target,0.01f) == TASK_MOVE);
        else assert(App_Tunnel_Update(&t,&target,0.01f) == TASK_UNSET);
    }
    t.tunnel_outside = TRACK_YES;
    App_Tunnel_SetLight(VISION_LIGHT_GREEN,1);
    assert(App_Tunnel_Update(&t,&target,0.01f) == TASK_FINISHED);
    App_Tunnel_Reset(); assert(!App_Tunnel_IsInside());
    // 底盘接口测试：拒绝启动时保留原任务，允许连续交接时保留速度目标。
    prepare(); assert(App_Chassis_StartLine1(1));
    assert(!App_Chassis_StartLap(1)); assert(!App_Chassis_StartLine2(0));
    chassis_boundary = 1; chassis_task_done = 1; chassis_target.vx = 0.2f;
    assert(App_Chassis_IsArrived());
    assert(App_Chassis_StartLap(1));
    assert(chassis_target.vx == 0.2f && !App_Chassis_IsArrived());
    App_Chassis_StopLine1(); assert(chassis_task == CHASSIS_TASK_LAP);
    App_Chassis_RequestStop(); chassis_stop_time = 0.01f;
    App_Chassis_RequestStop(); assert(chassis_stop_time == 0.01f);
    assert(chassis_status == CHASSIS_STATUS_STOPPING && !chassis_task_done);
    wheel_feedback = (WheelSpeed){0}; chassis_ramp = (ChassisSpeed){0};
    App_Chassis_Update_Stop(0.04f,0); assert(chassis_status == CHASSIS_STATUS_STOPPING);
    App_Chassis_Update_Stop(0.04f,1); assert(chassis_status == CHASSIS_STATUS_IDLE);
    assert(App_Chassis_StartLine2(1));
    chassis_finish_pending = 1; chassis_status = CHASSIS_STATUS_STOPPING;
    App_Chassis_Update_Stop(0.04f,1);
    assert(chassis_status == CHASSIS_STATUS_DONE && App_Chassis_IsArrived());
    ChassisMotorMap map = {BSP_MOTOR_1,BSP_MOTOR_2,BSP_MOTOR_3,BSP_MOTOR_4};
    App_Chassis_Run(&map,0,0.01f);
    assert(chassis_task == CHASSIS_TASK_NONE && !App_Chassis_IsArrived());
    chassis_status = CHASSIS_STATUS_FAULT; App_Chassis_RequestStop();
    assert(chassis_status == CHASSIS_STATUS_FAULT);
    App_Chassis_Run(&map,0,0.01f); assert(chassis_status == CHASSIS_STATUS_FAULT);
    // 流程不会自动解锁运动；当前执行其他任务时，也会消费电控乙提交的灯色有效标志。
    prepare(); assert(App_Chassis_StartLine1(1));
    assert(App_Chassis_SetLight(2,1));
    App_Chassis_Run(&map,1,0.01f);
    assert(!chassis_light_valid && nonzero_outputs == 0);
    output_fail = 1; App_Chassis_Run(&map,1,0.01f);
    assert(chassis_status == CHASSIS_STATUS_FAULT);
    puts("PASS: wheel control, routes, gaps, lap, tunnel, handoff, cancellation, disable, stop and light consumption");
    return 0;
}
