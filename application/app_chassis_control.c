// 底盘运动控制：斜坡、麦轮解算、轮速反馈与速度 PI。
#include "app_chassis_control.h"
#include "app_chassis_wheels.h"
#include "app_config.h"
#include "ramp.h"
#include <math.h>
#include <stddef.h>

// 本模块保存的整车速度目标，只允许通过接口修改。
static ChassisSpeed chassis_target = {0};

// 上次斜坡处理后的目标，下一次更新从这里继续。
static ChassisSpeed chassis_ramp = {0};
// 保存解算并限速后的四轮目标，供后续四轮速度PI使用。
static WheelSpeed wheel_target = {0};

// 保存完整四轮反馈；有效反馈只允许使用一次。
static WheelSpeed wheel_feedback = {0};
static uint8_t wheel_feedback_valid = 0;

// 四轮PI全部配置成功后才置1；后续控制计算须先检查此标志。
static uint8_t chassis_pi_ready = 0;

void Chassis_Control_Init(void)
{
    uint8_t wheel_index;

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
    for (wheel_index = 0U; wheel_index < WHEEL_COUNT; wheel_index++)
    {
        Chassis_Wheel_ResetSpeed(wheel_index);
    }

    wheel_feedback.front_left = 0.0f;
    wheel_feedback.front_right = 0.0f;
    wheel_feedback.rear_left = 0.0f;
    wheel_feedback.rear_right = 0.0f;

    wheel_feedback_valid = 0;
    // 底盘重新初始化后，清除旧积分，并要求重新配置四轮PI。
    chassis_pi_ready = 0;
    for (wheel_index = 0U; wheel_index < WHEEL_COUNT; wheel_index++)
    {
        Chassis_Wheel_ResetPI(wheel_index);
    }
}

void Chassis_Control_SetTarget(ChassisSpeed target)
{
    chassis_target = target;
}

uint8_t Chassis_Control_IsStopped(float max_wheel_speed)
{
    return !(fabsf(wheel_feedback.front_left) > max_wheel_speed ||
        fabsf(wheel_feedback.front_right) > max_wheel_speed ||
        fabsf(wheel_feedback.rear_left) > max_wheel_speed ||
        fabsf(wheel_feedback.rear_right) > max_wheel_speed ||
        chassis_ramp.vx != 0.0f || chassis_ramp.vy != 0.0f ||
        chassis_ramp.wz != 0.0f);
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

MotorSampleResult App_Chassis_Update_Feedback(const ChassisEncoder *encoder,
                                   uint32_t count_range,
                                   uint32_t counts_per_rev,
                                   float dt_s)
{
    uint8_t wheel_index;
    uint8_t error_count = 0U;
    uint8_t ready_count = 0U;
    float next_speed[WHEEL_COUNT];
    uint16_t current[WHEEL_COUNT];
    MotorSampleResult result;

    // 旧轮速不可作为本次反馈使用。
    wheel_feedback_valid = 0;

    if (encoder == NULL)
    {
        for (wheel_index = 0U; wheel_index < WHEEL_COUNT; wheel_index++)
        {
            Chassis_Wheel_ResetSpeed(wheel_index);
        }
        return MOTOR_SAMPLE_ERROR;
    }

    // 明确轮位映射，不把结构体强转成数组。
    current[WHEEL_FRONT_LEFT] = encoder->front_left;
    current[WHEEL_FRONT_RIGHT] = encoder->front_right;
    current[WHEEL_REAR_LEFT] = encoder->rear_left;
    current[WHEEL_REAR_RIGHT] = encoder->rear_right;

    for (wheel_index = 0U; wheel_index < WHEEL_COUNT; wheel_index++)
    {
        // 一路异常也继续更新其他轮，分别统计异常和正常准备。
        result = Chassis_Wheel_UpdateSpeed(wheel_index, current[wheel_index],
                                           count_range, counts_per_rev, dt_s,
                                           &next_speed[wheel_index]);
        if (result == MOTOR_SAMPLE_ERROR)
        {
            error_count++;
        }
        else if (result == MOTOR_SAMPLE_READY)
        {
            ready_count++;
        }
    }

    // 异常优先，不能把某轮异常当作其他轮的正常准备。
    if (error_count != 0U)
    {
        return MOTOR_SAMPLE_ERROR;
    }
    if (ready_count != 0U)
    {
        return MOTOR_SAMPLE_READY;
    }

    // 只有四轮均有效，才读取临时轮速并提交完整反馈。
    // motor与Wheel保留原始计数方向，此处仅修正一次，前进反馈统一为正。
    wheel_feedback.front_left = next_speed[WHEEL_FRONT_LEFT] * CHASSIS_ENCODER_SIGN_FL;
    wheel_feedback.front_right = next_speed[WHEEL_FRONT_RIGHT] * CHASSIS_ENCODER_SIGN_FR;
    wheel_feedback.rear_left = next_speed[WHEEL_REAR_LEFT] * CHASSIS_ENCODER_SIGN_RL;
    wheel_feedback.rear_right = next_speed[WHEEL_REAR_RIGHT] * CHASSIS_ENCODER_SIGN_RR;
    wheel_feedback_valid = 1;
    return MOTOR_SAMPLE_VALID;
}

uint8_t App_Chassis_Init_PI(float kp,
                           float ki,
                           float integral_max,
                           float output_min,
                           float output_max)
{
    uint8_t valid_count = 0U;
    uint8_t wheel_index;
    const int8_t output_sign[WHEEL_COUNT] = {
        CHASSIS_OUTPUT_SIGN_FL, CHASSIS_OUTPUT_SIGN_FR,
        CHASSIS_OUTPUT_SIGN_RL, CHASSIS_OUTPUT_SIGN_RR
    };

    // 每轮参数可以相同，但积分记录独立。全部成功才允许计算。
    chassis_pi_ready = 0;
    for (wheel_index = 0U; wheel_index < WHEEL_COUNT; wheel_index++)
    {
        // 将补偿后的硬件范围反算到PI域，让条件积分看到真实饱和点。
        // 当前硬件正向可达1000，乘2之前只能到500；反向可到-600。
        const float positive_limit = CHASSIS_HARDWARE_DUTY_MAX /
                                     CHASSIS_POSITIVE_DUTY_SCALE;
        const float wheel_min = output_sign[wheel_index] > 0 ?
            CHASSIS_HARDWARE_DUTY_MIN : -positive_limit;
        const float wheel_max = output_sign[wheel_index] > 0 ?
            positive_limit : -CHASSIS_HARDWARE_DUTY_MIN;
        valid_count += Chassis_Wheel_InitPI(wheel_index, kp, ki, integral_max,
                                           fmaxf(output_min, wheel_min),
                                           fminf(output_max, wheel_max));
    }
    if (valid_count != WHEEL_COUNT)
    {
        return 0;
    }

    chassis_pi_ready = 1;
    return 1;
}
uint8_t App_Chassis_Calc_Output(float dt_s,
                               ChassisOutput *output)
{
    uint8_t wheel_index;

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
        for (wheel_index = 0U; wheel_index < WHEEL_COUNT; wheel_index++)
        {
            Chassis_Wheel_ResetPI(wheel_index);
        }
        return 0;
    }

    // 本次反馈只使用一次，下一次计算前必须重新更新测速。
    wheel_feedback_valid = 0;

    // 每个轮子使用自己的PI、目标和反馈，积分互不影响。
    output->front_left = Chassis_Wheel_CalcOutput(
        WHEEL_FRONT_LEFT,
        wheel_target.front_left,
        wheel_feedback.front_left,
        dt_s);

    output->front_right = Chassis_Wheel_CalcOutput(
        WHEEL_FRONT_RIGHT,
        wheel_target.front_right,
        wheel_feedback.front_right,
        dt_s);

    output->rear_left = Chassis_Wheel_CalcOutput(
        WHEEL_REAR_LEFT,
        wheel_target.rear_left,
        wheel_feedback.rear_left,
        dt_s);

    output->rear_right = Chassis_Wheel_CalcOutput(
        WHEEL_REAR_RIGHT,
        wheel_target.rear_right,
        wheel_feedback.rear_right,
        dt_s);

    return 1;
}

void App_Chassis_Stop(ChassisOutput *output)
{
    uint8_t wheel_index;

    // 清除旧命令，避免下一周期继续追踪停止前的目标。
    chassis_target = (ChassisSpeed){0};
    chassis_ramp = (ChassisSpeed){0};
    wheel_target = (WheelSpeed){0};

    // 清除历史积分，保留已配置的PI参数和就绪状态。
    for (wheel_index = 0U; wheel_index < WHEEL_COUNT; wheel_index++)
    {
        Chassis_Wheel_ResetPI(wheel_index);
    }

    // 停止前的反馈不可再用于计算；编码器仍应继续周期采样。
    wheel_feedback_valid = 0;

    // 非空时将调用者的四轮请求一并清零；本函数不直接操作电机。
    if (output != NULL)
    {
        *output = (ChassisOutput){0};
    }
}

ChassisControlResult Chassis_Control_Update(const ChassisConfig *config,
                                            const ChassisEncoder *encoder,
                                            uint8_t enabled, float dt_s,
                                            ChassisOutput *output)
{
    MotorSampleResult feedback_result;
    uint8_t wheel_index;

    if (config == NULL)
    {
        // 无法按配置更新测速，取消旧计数起点，避免下次跨周期误算。
        App_Chassis_Update_Feedback(NULL, 0, 0, dt_s);
        App_Chassis_Stop(output);
        return CHASSIS_CONTROL_NO_OUTPUT;
    }

    // 先测速，即使禁止运行，也保持编码器采样连续。
    feedback_result = App_Chassis_Update_Feedback(
        encoder,
        config->count_range,
        config->counts_per_rev,
        dt_s);

    if (output == NULL ||
        enabled == 0 ||
        feedback_result == MOTOR_SAMPLE_ERROR)
    {
        App_Chassis_Stop(output);
        if (enabled != 0U && feedback_result == MOTOR_SAMPLE_ERROR)
        {
            return CHASSIS_CONTROL_ENCODER_ERROR;
        }
        return CHASSIS_CONTROL_NO_OUTPUT;
    }

    if (feedback_result == MOTOR_SAMPLE_READY)
    {
        // 正常准备测速：不输出驱动力，但保留已接受的整车目标。
        // 平滑目标从零重新起步，避免恢复时沿用旧斜坡状态。
        chassis_ramp = (ChassisSpeed){0};
        wheel_target = (WheelSpeed){0};
        for (wheel_index = 0U; wheel_index < WHEEL_COUNT; wheel_index++)
        {
            Chassis_Wheel_ResetPI(wheel_index);
        }
        *output = (ChassisOutput){0};
        return CHASSIS_CONTROL_NO_OUTPUT;
    }

    // 先平滑整车目标，再换算为四个轮子的目标。
    if (App_Chassis_Update_Ramp(&config->ramp, dt_s) == 0 ||
        App_Chassis_Update_Wheel(
            &config->mecanum, config->max_wheel_speed) == 0)
    {
        App_Chassis_Stop(output);
        return CHASSIS_CONTROL_NO_OUTPUT;
    }

    // 目标和反馈都准备好后，才允许计算驱动请求。
    if (App_Chassis_Calc_Output(dt_s, output) == 0)
    {
        App_Chassis_Stop(output);
        return CHASSIS_CONTROL_NO_OUTPUT;
    }

    return CHASSIS_CONTROL_OK;
}
