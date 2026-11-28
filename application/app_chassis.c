// 底盘状态机：管理任务、交接和停车，调度运动控制与硬件读写。
#include "app_chassis.h"
#include "app_chassis_io.h"
#include "app_chassis_control.h"
#include <stddef.h>
#include "app_config.h"
#include <math.h>
#include "app_line.h"
#include "app_line_task.h"
#include "app_lap.h"
#include "app_tunnel.h"

/// @brief 底盘内部的自动任务类型。
typedef enum
{
    CHASSIS_TASK_NONE = 0, ///< 没有自动任务。
    CHASSIS_TASK_LINE1,    ///< 第一段循迹。
    CHASSIS_TASK_LAP,      ///< 绕圈。
    CHASSIS_TASK_TUNNEL,   ///< 通过隧道。
    CHASSIS_TASK_LINE2     ///< 第二段循迹。
} ChassisTask;
// 记住当前自动任务，下一控制周期继续使用。
static ChassisTask chassis_task = CHASSIS_TASK_NONE;
// 到达交接点后仍继续行驶，只允许切换到约定的后续任务。
static uint8_t chassis_boundary = 0U;  // 是否到达允许连续交接的位置；为一不等于已经停稳。
static uint8_t chassis_finish_pending = 0U;  // 是否等待正常停车完成；取消任务时清零，避免误报正常完成。
static float chassis_stop_time = 0.0f;  // 连续满足停稳条件的累计时间，单位秒；条件中断就清零。
static uint8_t chassis_stop_auto = 0U;  // 本次停车是否源于自动任务；用于继续检查自动运动开关。
static uint8_t chassis_light = 0U;  // 电控乙最近提交的灯色编号。
static uint8_t chassis_light_valid = 0U;  // 灯色是否对下一次周期执行有效；每次周期执行都会清零。

// 1表示本次任务正常完成；0表示尚未完成，取消任务不能置1。
static uint8_t chassis_task_done = 0U;

// 当前底盘状态，由本模块管理，其他模块通过查询函数读取。
static ChassisStatus chassis_status = CHASSIS_STATUS_NOT_READY;

// 底盘控制配置：具体参数来自app_config.h。
// 计数范围由后续硬件初始化流程填写，目前0表示尚未准备好。
static ChassisConfig chassis_config =
{
    .ramp =
    {
        .linear_accel = CHASSIS_LINEAR_ACCEL,
        .linear_decel = CHASSIS_LINEAR_DECEL,
        .angular_accel = CHASSIS_ANGULAR_ACCEL,
        .angular_decel = CHASSIS_ANGULAR_DECEL
    },

    .mecanum =
    {
        .wheel_radius = CHASSIS_WHEEL_RADIUS_M,
        .half_length = CHASSIS_HALF_LENGTH_M,
        .half_width = CHASSIS_HALF_WIDTH_M
    },

    .max_wheel_speed = CHASSIS_MAX_WHEEL_SPEED_RAD_S,
    .count_range = 0U,
    .counts_per_rev = CHASSIS_COUNTS_PER_REV
};
ChassisStatus App_Chassis_GetStatus(void)
{
    return chassis_status;
}
void App_Chassis_Init(void)
{
    // 软件状态被重新初始化后，须重新完成启动流程。
    chassis_status = CHASSIS_STATUS_NOT_READY;

    // 重新初始化后，不继续执行之前的自动任务。
    chassis_task = CHASSIS_TASK_NONE;
    chassis_boundary = 0U;
    chassis_finish_pending = 0U;
    chassis_stop_time = 0.0f;
    chassis_stop_auto = 0U;
    chassis_light = 0U;
    chassis_light_valid = 0U;
    App_Line_Task_Reset();
    App_Lap_Reset();
    App_Tunnel_Reset();
    chassis_task_done = 0U;

    // 初始化或接收新任务时，清除上一次的计数与确认标记。
    App_Line_Reset();

    Chassis_Control_Init();
}

uint8_t App_Chassis_SetTarget(const ChassisSpeed *target)
{
    // 空闲或自动任务正常完成后，可接收手动目标；保留完成结果。
    if (chassis_status != CHASSIS_STATUS_IDLE &&
        chassis_status != CHASSIS_STATUS_DONE)
    {
        return 0;
    }

    if (target == NULL)
    {
        return 0;
    }

    // 拒绝NaN和无穷值，避免无效目标进入斜坡和PI计算。
    if (!isfinite(target->vx) ||
        !isfinite(target->vy) ||
        !isfinite(target->wz))
    {
        return 0;
    }

    // 复制三个速度值，不保存调用者的指针。
    Chassis_Control_SetTarget(*target);
    return 1;
}
// 清除各任务进度和交接记录；四轮比例积分控制与编码器采样仍由底盘控制流程管理。
static void App_Chassis_Reset_Task(void)
{
    App_Line_Task_Reset();
    App_Lap_Reset();
    App_Tunnel_Reset();
    chassis_boundary = 0U;
    chassis_task_done = 0U;
    chassis_finish_pending = 0U;
    chassis_stop_time = 0.0f;
    chassis_light_valid = 0U;
}

// 统一接收阶段启动请求：检查路线和当前状态；接受返回一，拒绝返回零，允许的连续交接保留速度。
static uint8_t App_Chassis_Start_Task(ChassisTask task, uint8_t route)
{
    // 是否允许从当前任务连续交接到指定任务；为一时保留当前速度目标。
    uint8_t handoff = chassis_boundary != 0U &&
        chassis_status == CHASSIS_STATUS_RUNNING &&
        ((chassis_task == CHASSIS_TASK_LINE1 && task == CHASSIS_TASK_LAP) ||
         (chassis_task == CHASSIS_TASK_LAP &&
          (task == CHASSIS_TASK_LINE2 || task == CHASSIS_TASK_TUNNEL)));
    if (route < 1U || route > 3U ||
        (chassis_status != CHASSIS_STATUS_IDLE &&
         chassis_status != CHASSIS_STATUS_DONE && handoff == 0U) ||
        chassis_status == CHASSIS_STATUS_FAULT ||
        chassis_status == CHASSIS_STATUS_NOT_READY)
    {
        return 0U;
    }
    App_Chassis_Reset_Task();
    if (handoff == 0U)
    {
        Chassis_Control_SetTarget((ChassisSpeed){0});
    }
    chassis_task = task;
    switch (task)
    {
        case CHASSIS_TASK_LINE1: App_Line_Task_Start(1U, route); break;
        case CHASSIS_TASK_LINE2: App_Line_Task_Start(2U, route); break;
        case CHASSIS_TASK_LAP: App_Lap_Start(route); break;
        case CHASSIS_TASK_TUNNEL: App_Tunnel_Start(route); break;
        default: return 0U;
    }
    chassis_status = CHASSIS_STATUS_RUNNING;
    return 1U;
}
uint8_t App_Chassis_StartLine1(uint8_t route)
{
    return App_Chassis_Start_Task(CHASSIS_TASK_LINE1, route);
}
uint8_t App_Chassis_StartLine2(uint8_t route)
{
    return App_Chassis_Start_Task(CHASSIS_TASK_LINE2, route);
}
uint8_t App_Chassis_StartLap(uint8_t route)
{
    return App_Chassis_Start_Task(CHASSIS_TASK_LAP, route);
}
uint8_t App_Chassis_PassTunnel(uint8_t route)
{
    return App_Chassis_Start_Task(CHASSIS_TASK_TUNNEL, route);
}
void App_Chassis_RequestStop(void)
{
    if (chassis_status == CHASSIS_STATUS_STOPPING)
    {
        // 重复取消不重置停车计时，但仍清掉正常完成及隧道进入记录。
        App_Line_Task_Reset();
        App_Lap_Reset();
        App_Tunnel_Reset();
        chassis_finish_pending = 0U;
        chassis_task_done = 0U;
        chassis_boundary = 0U;
        chassis_task = CHASSIS_TASK_NONE;
        return;
    }
    chassis_stop_auto = chassis_task != CHASSIS_TASK_NONE;
    App_Chassis_Reset_Task();
    chassis_task = CHASSIS_TASK_NONE;
    Chassis_Control_SetTarget((ChassisSpeed){0});
    if (chassis_status != CHASSIS_STATUS_FAULT &&
        chassis_status != CHASSIS_STATUS_NOT_READY &&
        chassis_status != CHASSIS_STATUS_STOPPING)
    {
        chassis_status = CHASSIS_STATUS_STOPPING;
    }
}
void App_Chassis_StopLine1(void)
{
    if (chassis_task == CHASSIS_TASK_LINE1)
    {
        App_Chassis_RequestStop();
    }
}
void App_Chassis_StopLine2(void)
{
    if (chassis_task == CHASSIS_TASK_LINE2)
    {
        App_Chassis_RequestStop();
    }
}
void App_Chassis_StopLap(void)
{
    if (chassis_task == CHASSIS_TASK_LAP)
    {
        App_Chassis_RequestStop();
    }
}
uint8_t App_Chassis_IsArrived(void)
{
    return chassis_task_done != 0U &&
        (chassis_task == CHASSIS_TASK_LINE1 ||
         chassis_task == CHASSIS_TASK_LINE2 ||
         chassis_task == CHASSIS_TASK_TUNNEL);
}
uint8_t App_Chassis_IsLapDone(void)
{
    return chassis_task_done != 0U && chassis_task == CHASSIS_TASK_LAP;
}
uint8_t App_Chassis_IsTunnelInside(void)
{
    return chassis_task == CHASSIS_TASK_TUNNEL && App_Tunnel_IsInside() != 0U;
}
uint8_t App_Chassis_SetLight(uint8_t color, uint8_t valid)
{
    if (color > VISION_LIGHT_YELLOW || valid > 1U)
    {
        chassis_light_valid = 0U;
        return 0U;
    }
    chassis_light = color;
    chassis_light_valid = valid;
    return 1U;
}

uint8_t App_Chassis_Update(const ChassisConfig *config,
                           const ChassisEncoder *encoder,
                           uint8_t enabled,
                           float dt_s,
                           ChassisOutput *output)
{
    ChassisControlResult result = Chassis_Control_Update(config, encoder,
                                                       enabled, dt_s, output);
    if (result == CHASSIS_CONTROL_ENCODER_ERROR && chassis_task != CHASSIS_TASK_NONE)
    {
        chassis_status = CHASSIS_STATUS_FAULT;
    }
    return result == CHASSIS_CONTROL_OK;
}

uint8_t App_Chassis_Control(const ChassisConfig *config,
                           const ChassisMotorMap *map,
                           uint8_t enabled,
                           float dt_s)
{
    ChassisEncoder encoder;
    ChassisOutput output = {0};
    uint8_t control_valid;

    // 读取失败时传NULL，通知测速逻辑取消旧计数起点。
    if (App_Chassis_Read_Encoder(map, &encoder) == 0)
    {
        control_valid = App_Chassis_Update(
            config, NULL, enabled, dt_s, &output);
    }
    else
    {
        control_valid = App_Chassis_Update(
            config, &encoder, enabled, dt_s, &output);
    }

    // 即使禁止运行或计算失败，也要执行Update给出的零输出。
    if (App_Chassis_Write_Output(map, &output) == 0)
    {
        if (chassis_status != CHASSIS_STATUS_NOT_READY)
        {
            chassis_status = CHASSIS_STATUS_FAULT;
        }
        App_Chassis_Stop(&output);

        // 轮位映射可能出错，直接按电路板接口尝试全部归零。
        Chassis_IO_TryZeroAll();

        return 0;
    }

    return control_valid;
}
uint8_t App_Chassis_Init_Hardware(void)
{
    uint8_t output_ready;
    uint8_t encoder_ready;

    // 清除旧范围，获取成功后再填入有效值。
    chassis_config.count_range = 0U;

    // 配置不符合当前测速要求时，不继续启动硬件。
    if (BSP_Motor_Get_Encoder_Range(
            &chassis_config.count_range) == 0)
    {
        return 0;
    }

    // PWM从零输出启动。
    output_ready = BSP_Motor_Output_Init();

    // 启动编码器，之后才能周期读取计数。
    encoder_ready = BSP_Motor_Encoder_Init();

    return output_ready && encoder_ready;
}
// 启动前集中检查固定配置，周期控制不重复检查这些固定数值。
// 编码器计数范围稍后从BSP获取；循迹参数留到自动任务入口检查。
static uint8_t App_Chassis_Check_Config(void)
{
    // 几何尺寸必须为有限正数，单位为米。
    if (!isfinite(chassis_config.mecanum.wheel_radius) ||
        !isfinite(chassis_config.mecanum.half_length) ||
        !isfinite(chassis_config.mecanum.half_width) ||
        chassis_config.mecanum.wheel_radius <= 0.0f ||
        chassis_config.mecanum.half_length <= 0.0f ||
        chassis_config.mecanum.half_width <= 0.0f)
    {
        return 0;
    }

    // 斜坡和轮速上限不能使用尚未确认的零值。
    if (!isfinite(chassis_config.ramp.linear_accel) ||
        !isfinite(chassis_config.ramp.linear_decel) ||
        !isfinite(chassis_config.ramp.angular_accel) ||
        !isfinite(chassis_config.ramp.angular_decel) ||
        !isfinite(chassis_config.max_wheel_speed) ||
        chassis_config.ramp.linear_accel <= 0.0f ||
        chassis_config.ramp.linear_decel <= 0.0f ||
        chassis_config.ramp.angular_accel <= 0.0f ||
        chassis_config.ramp.angular_decel <= 0.0f ||
        chassis_config.max_wheel_speed <= 0.0f ||
        chassis_config.counts_per_rev == 0U)
    {
        return 0;
    }

    // 增益允许为0；PI的大小关系由Pid_Init检查，此处只补充有限值及驱动范围。
    if (!isfinite(CHASSIS_SPEED_KP) ||
        !isfinite(CHASSIS_SPEED_KI) ||
        !isfinite(CHASSIS_SPEED_INTEGRAL_MAX) ||
        !isfinite(CHASSIS_SPEED_OUTPUT_MIN) ||
        !isfinite(CHASSIS_SPEED_OUTPUT_MAX) ||
        CHASSIS_SPEED_OUTPUT_MIN < -1000.0f ||
        CHASSIS_SPEED_OUTPUT_MIN > 0.0f ||
        CHASSIS_SPEED_OUTPUT_MAX < 0.0f ||
        CHASSIS_SPEED_OUTPUT_MAX > 1000.0f)
    {
        return 0;
    }

    // 硬件正向补偿必须有效，硬件范围必须包含零且不超过BSP千分比能力。
    if (!isfinite(CHASSIS_POSITIVE_DUTY_SCALE) ||
        CHASSIS_POSITIVE_DUTY_SCALE <= 0.0f ||
        !isfinite(CHASSIS_HARDWARE_DUTY_MIN) ||
        !isfinite(CHASSIS_HARDWARE_DUTY_MAX) ||
        CHASSIS_HARDWARE_DUTY_MIN < -1000.0f ||
        CHASSIS_HARDWARE_DUTY_MIN >= 0.0f ||
        CHASSIS_HARDWARE_DUTY_MAX <= 0.0f ||
        CHASSIS_HARDWARE_DUTY_MAX > 1000.0f)
    {
        return 0;
    }

    return 1;
}
uint8_t App_Chassis_Start(void)
{
    // 清除旧目标、测速历史和PI状态。
    App_Chassis_Init();

    // 检查底盘配置。
    if (App_Chassis_Check_Config() == 0)
    {
        return 0;
    }

    // 配置四轮PI，各轮积分独立。
    if (App_Chassis_Init_PI(
            CHASSIS_SPEED_KP,
            CHASSIS_SPEED_KI,
            CHASSIS_SPEED_INTEGRAL_MAX,
            CHASSIS_SPEED_OUTPUT_MIN,
            CHASSIS_SPEED_OUTPUT_MAX) == 0)
    {
        return 0;
    }

    // 启动PWM和编码器。
    if (App_Chassis_Init_Hardware() == 0)
    {
        App_Chassis_Init();
        return 0;
    }

    // 配置、PI和硬件均已准备成功，目前没有自动任务占用。
    chassis_status = CHASSIS_STATUS_IDLE;

    return 1;
}
// 本周期控制完成后检查停稳；依据本次控制结果和实际轮速，不重复消费比例积分控制用过的反馈标志。
static void App_Chassis_Update_Stop(float dt_s, uint8_t control_valid)
{
    if (control_valid == 0U || CHASSIS_STOP_CONFIRM_S <= 0.0f)
    {
        chassis_stop_time = 0.0f;
        return;
    }
    if (Chassis_Control_IsStopped(CHASSIS_STOP_WHEEL_RPS) == 0U)
    {
        chassis_stop_time = 0.0f;
        return;
    }
    chassis_stop_time += dt_s;
    if (chassis_stop_time >= CHASSIS_STOP_CONFIRM_S)
    {
        chassis_task_done = chassis_finish_pending;
        chassis_status = chassis_finish_pending != 0U ?
                         CHASSIS_STATUS_DONE : CHASSIS_STATUS_IDLE;
        chassis_finish_pending = 0U;
    }
}

// 按当前任务调用对应的周期函数；写入速度请求，并返回本周期任务结果。
// 灯色有效标志只传给隧道任务，现场判断仍由统一接口提供。
static TaskResult App_Chassis_Dispatch_Task(ChassisSpeed *target,
                                            float dt_s, uint8_t light_valid)
{
    TrackInput track;  // 本周期现场结果；占位实现会返回尚未配置。
    App_Track_Read(&track);
    switch (chassis_task)
    {
        case CHASSIS_TASK_LINE1:
        case CHASSIS_TASK_LINE2:
            return App_Line_Task_Update(&track, target, dt_s);
        case CHASSIS_TASK_LAP:
            return App_Lap_Update(&track, target, dt_s);
        case CHASSIS_TASK_TUNNEL:
            App_Tunnel_SetLight(chassis_light, light_valid);
            return App_Tunnel_Update(&track, target, dt_s);
        default:
            return TASK_FAILED;
    }
}

// 将任务结果转换为底盘状态；交接继续行驶，正常结束则进入停稳确认。
// 这里只处理目标和流程标志，电机输出仍由周期总入口统一执行。
static void App_Chassis_Handle_Result(TaskResult result,
                                      const ChassisSpeed *target)
{
    Chassis_Control_SetTarget(*target);
    switch (result)
    {
        case TASK_MOVE:
            chassis_status = CHASSIS_STATUS_RUNNING;
            break;
        case TASK_BOUNDARY:
            // 到达交接点仍保持运行，等待电控乙调用约定的下一阶段启动接口。
            chassis_boundary = 1U;
            chassis_task_done = 1U;
            chassis_status = CHASSIS_STATUS_RUNNING;
            break;
        case TASK_FINISHED:
            Chassis_Control_SetTarget((ChassisSpeed){0});
            chassis_finish_pending = 1U;
            chassis_stop_auto = 1U;
            chassis_stop_time = 0.0f;
            chassis_status = CHASSIS_STATUS_STOPPING;
            break;
        case TASK_FAILED:
            chassis_status = CHASSIS_STATUS_FAULT;
            chassis_task_done = 0U;
            chassis_boundary = 0U;
            break;
        case TASK_UNSET:
        case TASK_WAIT:
            Chassis_Control_SetTarget((ChassisSpeed){0});
            chassis_status = CHASSIS_STATUS_WAITING;
            break;
    }
}

// 周期总入口：推进当前任务，生成速度目标，再执行测速、斜坡、解算、四轮控制和硬件输出。
uint8_t App_Chassis_Run(const ChassisMotorMap *map,
                       uint8_t enabled, float dt_s)
{
    uint8_t motion_allowed = enabled;  // 本周期最终是否允许电机运动，后面还要检查任务配置和故障。
    uint8_t control_valid;  // 本次硬件控制是否成功；不能拿旧反馈来确认停稳。
    ChassisSpeed target = {0};  // 任务本周期生成的整车速度请求，初始为零。
    TaskResult result = TASK_WAIT;  // 任务本周期返回的进度，决定底盘继续、等待、交接或停车。
    uint8_t light_valid = chassis_light_valid;  // 先保存本周期灯色有效性，再清除内部标志，避免重复使用。
    chassis_light_valid = 0U;  // 每次周期执行都会消费电控乙提交的灯色有效标志，不能沿用上一周期的有效性。

    if (enabled == 0U)
    {
        App_Chassis_RequestStop();
        if (chassis_status != CHASSIS_STATUS_FAULT &&
            chassis_status != CHASSIS_STATUS_NOT_READY)
        {
            chassis_status = CHASSIS_STATUS_IDLE;
        }
    }
    else if (!isfinite(dt_s) || dt_s <= 0.0f)
    {
        chassis_status = CHASSIS_STATUS_FAULT;
    }
    else if (chassis_status == CHASSIS_STATUS_STOPPING)
    {
        Chassis_Control_SetTarget((ChassisSpeed){0});
    }
    else if (chassis_task != CHASSIS_TASK_NONE &&
             chassis_status != CHASSIS_STATUS_DONE &&
             chassis_status != CHASSIS_STATUS_FAULT &&
             chassis_status != CHASSIS_STATUS_NOT_READY)
    {
        result = App_Chassis_Dispatch_Task(&target, dt_s, light_valid);
        App_Chassis_Handle_Result(result, &target);
        // 流程接好后仍须检查自动运动开关；仅添加流程不会自动放开实车电机输出。
        motion_allowed = enabled && CHASSIS_AUTO_MOTION_READY != 0U;
        if (result == TASK_UNSET)
        {
            motion_allowed = 0U;
        }
    }
    if (chassis_status == CHASSIS_STATUS_STOPPING && chassis_stop_auto != 0U)
    {
        motion_allowed = enabled && CHASSIS_AUTO_MOTION_READY != 0U;
    }
    if (chassis_status == CHASSIS_STATUS_FAULT ||
        chassis_status == CHASSIS_STATUS_NOT_READY)
    {
        Chassis_Control_SetTarget((ChassisSpeed){0});
        motion_allowed = 0U;
    }
    control_valid = App_Chassis_Control(&chassis_config, map,
                                        motion_allowed, dt_s);
    if (chassis_status == CHASSIS_STATUS_FAULT)
    {
        chassis_task_done = 0U;
        chassis_boundary = 0U;
    }
    if (chassis_status == CHASSIS_STATUS_STOPPING)
    {
        App_Chassis_Update_Stop(dt_s, control_valid);
    }
    return control_valid;
}
