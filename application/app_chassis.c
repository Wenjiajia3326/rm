// 保存底盘目标，后续在本模块组织周期控制。
#include "app_chassis.h"
#include "ramp.h"
#include <stddef.h>
#include "motor.h"
#include "pid.h"

// 本模块保存的整车速度目标，只允许通过接口修改。
static ChassisSpeed chassis_target = {0};

// 上次斜坡处理后的目标，下一次更新从这里继续。
static ChassisSpeed chassis_ramp = {0};
// 保存解算并限速后的四轮目标，供后续四轮速度PI使用。
static WheelSpeed wheel_target = {0};

// 四轮分别保存自己的上次计数，不能共用测速记录。
static MotorData motor_front_left = {0};
static MotorData motor_front_right = {0};
static MotorData motor_rear_left = {0};
static MotorData motor_rear_right = {0};

// 保存四轮实际角速度，单位rad/s，正方向与四轮目标一致。
static WheelSpeed wheel_feedback = {0};

// 1表示有一份尚未用于PI计算的有效四轮反馈；0表示不可用于计算。
static uint8_t wheel_feedback_valid = 0;
// 四轮参数可以相同，但各自的积分必须独立保存。
static PidData pid_front_left = {0};
static PidData pid_front_right = {0};
static PidData pid_rear_left = {0};
static PidData pid_rear_right = {0};

// 四轮PI全部配置成功后才置1；后续控制计算须先检查此标志。
static uint8_t chassis_pi_ready = 0;


void App_Chassis_Init(void)
{
    chassis_target.vx = 0.0f;
    chassis_target.vy = 0.0f;
    chassis_target.wz = 0.0f;

    chassis_ramp.vx = 0.0f;
    chassis_ramp.vy = 0.0f;
    chassis_ramp.wz = 0.0f;

    wheel_target.front_left = 0.0f;
    wheel_target.front_right = 0.0f;
    wheel_target.rear_left = 0.0f;
    wheel_target.rear_right = 0.0f;
    // 重新初始化后，四轮都需要重新建立计数起点。
    motor_front_left.initialized = 0;
    motor_front_right.initialized = 0;
    motor_rear_left.initialized = 0;
    motor_rear_right.initialized = 0;

    wheel_feedback.front_left = 0.0f;
    wheel_feedback.front_right = 0.0f;
    wheel_feedback.rear_left = 0.0f;
    wheel_feedback.rear_right = 0.0f;

    wheel_feedback_valid = 0;
    // 底盘重新初始化后，清除旧积分，并要求重新配置四轮PI。
    chassis_pi_ready = 0;
    Pid_Reset(&pid_front_left);
    Pid_Reset(&pid_front_right);
    Pid_Reset(&pid_rear_left);
    Pid_Reset(&pid_rear_right);
}

uint8_t App_Chassis_Set_Target(const ChassisSpeed *target)
{
    if (target == NULL)
    {
        return 0;
    }

    chassis_target = *target;
    return 1;
}
uint8_t App_Chassis_Update_Ramp(const ChassisRampConfig *config,
                               float dt_s)
{
    // || 从左向右判断，空指针时不会继续读取配置成员。
    if (config == NULL ||
        config->linear_accel <= 0.0f ||
        config->linear_decel <= 0.0f ||
        config->angular_accel <= 0.0f ||
        config->angular_decel <= 0.0f ||
        dt_s <= 0.0f)
    {
        return 0;
    }

    chassis_ramp.vx = Ramp_Update(chassis_ramp.vx,
                                 chassis_target.vx,
                                 config->linear_accel,
                                 config->linear_decel,
                                 dt_s);

    chassis_ramp.vy = Ramp_Update(chassis_ramp.vy,
                                 chassis_target.vy,
                                 config->linear_accel,
                                 config->linear_decel,
                                 dt_s);

    chassis_ramp.wz = Ramp_Update(chassis_ramp.wz,
                                 chassis_target.wz,
                                 config->angular_accel,
                                 config->angular_decel,
                                 dt_s);

    return 1;
}
uint8_t App_Chassis_Update_Wheel(const MecanumConfig *config,
                                float max_speed)
{
    WheelSpeed next_target;

    if (Mecanum_Calc(config, &chassis_ramp, &next_target) == 0)
    {
        return 0;
    }

    if (Mecanum_Limit(&next_target, max_speed) == 0)
    {
        return 0;
    }

    wheel_target = next_target;
    return 1;
}

uint8_t App_Chassis_Update_Feedback(const ChassisEncoder *encoder,
                                   uint32_t count_range,
                                   uint32_t counts_per_rev,
                                   float dt_s)
{
    // 临时保存本次轮速；只有四轮全部算好，才读取并复制整份结果。
    WheelSpeed next_feedback;
    // 普通局部变量，每次调用都从0开始，统计本次成功的轮子数量。
    uint8_t valid_count = 0;

    // 取消本次反馈的有效资格，不清空旧轮速；旧值不能当成本次测量使用。
    wheel_feedback_valid = 0;

    // 没有读数时取消旧起点，避免恢复后用单周期时间除跨周期计数差。
    if (encoder == NULL)
    {
        motor_front_left.initialized = 0;
        motor_front_right.initialized = 0;
        motor_rear_left.initialized = 0;
        motor_rear_right.initialized = 0;
        return 0;
    }

    // Motor_Update把速度写入最后一个参数指向的位置，返回值仅表示成功1或失败0。
    // +=累加的是成功标志，不是轮速；四条调用分别执行，前轮失败也不跳过后轮。
    valid_count += Motor_Update(
        &motor_front_left, encoder->front_left,
        count_range, counts_per_rev, dt_s,
        &next_feedback.front_left);

    valid_count += Motor_Update(
        &motor_front_right, encoder->front_right,
        count_range, counts_per_rev, dt_s,
        &next_feedback.front_right);

    valid_count += Motor_Update(
        &motor_rear_left, encoder->rear_left,
        count_range, counts_per_rev, dt_s,
        &next_feedback.rear_left);

    valid_count += Motor_Update(
        &motor_rear_right, encoder->rear_right,
        count_range, counts_per_rev, dt_s,
        &next_feedback.rear_right);

    // 首次采样只建立起点，也返回0；未成功的临时字段不能读取。
    if (valid_count != 4)
    {
        return 0;
    }

    // 结构体赋值复制四个轮速；保存完成后，再标记本次反馈有效。
    wheel_feedback = next_feedback;
    wheel_feedback_valid = 1;
    return 1;
}

uint8_t App_Chassis_Init_PI(float kp,
                           float ki,
                           float integral_max,
                           float output_min,
                           float output_max)
{
    uint8_t valid_count = 0;

    // 即使之前配置成功，本次也要四轮全部通过后才能重新允许计算。
    chassis_pi_ready = 0;

    // 返回值是成功1或失败0；同一组参数分别写入四份独立的PI数据。
    valid_count += Pid_Init(
        &pid_front_left, kp, ki, integral_max,
        output_min, output_max);

    valid_count += Pid_Init(
        &pid_front_right, kp, ki, integral_max,
        output_min, output_max);

    valid_count += Pid_Init(
        &pid_rear_left, kp, ki, integral_max,
        output_min, output_max);

    valid_count += Pid_Init(
        &pid_rear_right, kp, ki, integral_max,
        output_min, output_max);

    if (valid_count != 4)
    {
        return 0;
    }

    chassis_pi_ready = 1;
    return 1;
}
uint8_t App_Chassis_Calc_Output(float dt_s,
                               ChassisOutput *output)
{
    if (output == NULL)
    {
        return 0;
    }

    // 先给出零请求，条件不满足时不会把调用者的旧请求留下来。
    *output = (ChassisOutput){0};

    if (chassis_pi_ready == 0 ||
        wheel_feedback_valid == 0 ||
        dt_s <= 0.0f)
    {
        // 本次计算条件无效，取消旧反馈资格，恢复后必须重新采样。
        wheel_feedback_valid = 0;
        Pid_Reset(&pid_front_left);
        Pid_Reset(&pid_front_right);
        Pid_Reset(&pid_rear_left);
        Pid_Reset(&pid_rear_right);
        return 0;
    }

    // 本次反馈只使用一次，下一次计算前必须重新更新测速。
    wheel_feedback_valid = 0;

    // 每个轮子使用自己的PI、目标和反馈，积分互不影响。
    output->front_left = Pid_Calc(
        &pid_front_left,
        wheel_target.front_left,
        wheel_feedback.front_left,
        dt_s);

    output->front_right = Pid_Calc(
        &pid_front_right,
        wheel_target.front_right,
        wheel_feedback.front_right,
        dt_s);

    output->rear_left = Pid_Calc(
        &pid_rear_left,
        wheel_target.rear_left,
        wheel_feedback.rear_left,
        dt_s);

    output->rear_right = Pid_Calc(
        &pid_rear_right,
        wheel_target.rear_right,
        wheel_feedback.rear_right,
        dt_s);

    return 1;
}

void App_Chassis_Stop(ChassisOutput *output)
{
    // 清除旧命令，避免下一周期继续追踪停止前的目标。
    chassis_target = (ChassisSpeed){0};
    chassis_ramp = (ChassisSpeed){0};
    wheel_target = (WheelSpeed){0};

    // 清除历史积分，保留已配置的PI参数和就绪状态。
    Pid_Reset(&pid_front_left);
    Pid_Reset(&pid_front_right);
    Pid_Reset(&pid_rear_left);
    Pid_Reset(&pid_rear_right);

    // 停止前的反馈不可再用于计算；编码器仍应继续周期采样。
    wheel_feedback_valid = 0;

    // 非空时将调用者的四轮请求一并清零；本函数不直接操作电机。
    if (output != NULL)
    {
        *output = (ChassisOutput){0};
    }
}

uint8_t App_Chassis_Update(const ChassisConfig *config,
                           const ChassisEncoder *encoder,
                           uint8_t enabled,
                           float dt_s,
                           ChassisOutput *output)
{
    uint8_t feedback_valid;

    if (config == NULL)
    {
        // 无法按配置更新测速，取消旧计数起点，避免下次跨周期误算。
        App_Chassis_Update_Feedback(NULL, 0, 0, dt_s);
        App_Chassis_Stop(output);
        return 0;
    }

    // 先测速，即使禁止运行，也保持编码器采样连续。
    feedback_valid = App_Chassis_Update_Feedback(
        encoder,
        config->count_range,
        config->counts_per_rev,
        dt_s);

    if (output == NULL ||
        enabled == 0 ||
        feedback_valid == 0)
    {
        App_Chassis_Stop(output);
        return 0;
    }

    // 先平滑整车目标，再换算为四个轮子的目标。
    if (App_Chassis_Update_Ramp(&config->ramp, dt_s) == 0 ||
        App_Chassis_Update_Wheel(
            &config->mecanum, config->max_wheel_speed) == 0)
    {
        App_Chassis_Stop(output);
        return 0;
    }

    // 目标和反馈都准备好后，才允许计算驱动请求。
    if (App_Chassis_Calc_Output(dt_s, output) == 0)
    {
        App_Chassis_Stop(output);
        return 0;
    }

    return 1;
}
