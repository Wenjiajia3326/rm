// 保存底盘目标，后续在本模块组织周期控制。
#include "app_chassis.h"
#include "ramp.h"
#include <stddef.h>
#include "motor.h"
#include "pid.h"
#include "app_config.h"
#include <math.h>
#include "app_line.h"

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
// 本次任务选择的路线：1~3为有效路线，0表示尚未选择。
static uint8_t chassis_route = 0U;

// 目前只填写已确认的入口动作，每行对应一条路线。
static const LineAction line_route_action[3][1] =
{
    {LINE_ACTION_LEFT},      // 路线1：进入左侧支路。
    {LINE_ACTION_STRAIGHT},  // 路线2：进入中间支路。
    {LINE_ACTION_RIGHT}      // 路线3：进入右侧支路。
};

// 当前待完成的动作下标，0表示入口选择尚未完成。
static uint8_t line_step = 0U;

static LineAction App_Chassis_Get_Line_Action(void)
{
    // 路线号从1开始，数组下标从0开始。
    if (chassis_route < 1U || chassis_route > 3U)
    {
        return LINE_ACTION_WAIT;
    }

    if (line_step >= sizeof(line_route_action[0]) /
                     sizeof(line_route_action[0][0]))
    {
        // 路线1、3完成入口选择后，先按缓弯方案连续循迹。
        // 路线2等待后续直角顺序；表末尾不表示到达汇合点。
        if (chassis_route == 1U || chassis_route == 3U)
        {
            return LINE_ACTION_FOLLOW;
        }
        return LINE_ACTION_WAIT;
    }
    return line_route_action[chassis_route - 1U][line_step];
}
// 1表示本次任务正常完成；0表示尚未完成，取消任务不能置1。
static uint8_t chassis_task_done = 0U;

// 当前底盘状态，由本模块管理，其他模块通过查询函数读取。
static ChassisStatus chassis_status = CHASSIS_STATUS_NOT_READY;

// 本模块保存的整车速度目标，只允许通过接口修改。
static ChassisSpeed chassis_target = {0};

// 上次斜坡处理后的目标，下一次更新从这里继续。
static ChassisSpeed chassis_ramp = {0};
// 保存解算并限速后的四轮目标，供后续四轮速度PI使用。
static WheelSpeed wheel_target = {0};

// 内部轮序固定为左前、右前、左后、右后，不等同于电路板接口编号。
typedef enum
{
    WHEEL_FRONT_LEFT = 0,
    WHEEL_FRONT_RIGHT,
    WHEEL_REAR_LEFT,
    WHEEL_REAR_RIGHT,
    WHEEL_COUNT
} WheelId;

// 每个轮子独立保存测速历史和PI积分。
static MotorData motor[WHEEL_COUNT] = {0};
static PidData pid[WHEEL_COUNT] = {0};

// 保存完整四轮反馈；有效反馈只允许使用一次。
static WheelSpeed wheel_feedback = {0};
static uint8_t wheel_feedback_valid = 0;

// 四轮PI全部配置成功后才置1；后续控制计算须先检查此标志。
static uint8_t chassis_pi_ready = 0;

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
    uint8_t wheel_index;

    // 软件状态被重新初始化后，须重新完成启动流程。
    chassis_status = CHASSIS_STATUS_NOT_READY;

    // 重新初始化后，不继续执行之前的自动任务。
    chassis_task = CHASSIS_TASK_NONE;
    chassis_route = 0U;
    line_step = 0U;
    chassis_task_done = 0U;

    // 初始化或接收新任务时，清除上一次的计数与确认标记。
    App_Line_Reset();

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
        motor[wheel_index].initialized = 0;
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
        Pid_Reset(&pid[wheel_index]);
    }
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
    chassis_target = *target;
    return 1;
}
uint8_t App_Chassis_StartLine1(uint8_t route)
{
    // 空闲或上一任务正常完成后，才能接收新任务。
    if (chassis_status != CHASSIS_STATUS_IDLE &&
        chassis_status != CHASSIS_STATUS_DONE)
    {
        return 0;
    }

    // 路线只允许1、2、3。
    if (route < 1U || route > 3U)
    {
        return 0;
    }

    // 清除旧手动目标，后续由循迹逻辑生成新目标。
    chassis_target.vx = 0.0f;
    chassis_target.vy = 0.0f;
    chassis_target.wz = 0.0f;

    chassis_task = CHASSIS_TASK_LINE1;
    chassis_route = route;
    line_step = 0U;  // 新任务从入口选择开始。
    chassis_task_done = 0U;

    // 初始化或接收新任务时，清除上一次的计数与确认标记。
    App_Line_Reset();

    chassis_status = CHASSIS_STATUS_RUNNING;

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
            motor[wheel_index].initialized = 0;
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
        result = Motor_Update(&motor[wheel_index], current[wheel_index],
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
    wheel_feedback.front_left = next_speed[WHEEL_FRONT_LEFT];
    wheel_feedback.front_right = next_speed[WHEEL_FRONT_RIGHT];
    wheel_feedback.rear_left = next_speed[WHEEL_REAR_LEFT];
    wheel_feedback.rear_right = next_speed[WHEEL_REAR_RIGHT];
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

    // 每轮参数可以相同，但积分记录独立。全部成功才允许计算。
    chassis_pi_ready = 0;
    for (wheel_index = 0U; wheel_index < WHEEL_COUNT; wheel_index++)
    {
        valid_count += Pid_Init(&pid[wheel_index], kp, ki, integral_max,
                                output_min, output_max);
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
            Pid_Reset(&pid[wheel_index]);
        }
        return 0;
    }

    // 本次反馈只使用一次，下一次计算前必须重新更新测速。
    wheel_feedback_valid = 0;

    // 每个轮子使用自己的PI、目标和反馈，积分互不影响。
    output->front_left = Pid_Calc(
        &pid[WHEEL_FRONT_LEFT],
        wheel_target.front_left,
        wheel_feedback.front_left,
        dt_s);

    output->front_right = Pid_Calc(
        &pid[WHEEL_FRONT_RIGHT],
        wheel_target.front_right,
        wheel_feedback.front_right,
        dt_s);

    output->rear_left = Pid_Calc(
        &pid[WHEEL_REAR_LEFT],
        wheel_target.rear_left,
        wheel_feedback.rear_left,
        dt_s);

    output->rear_right = Pid_Calc(
        &pid[WHEEL_REAR_RIGHT],
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
        Pid_Reset(&pid[wheel_index]);
    }

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
    MotorSampleResult feedback_result;
    uint8_t wheel_index;

    if (config == NULL)
    {
        // 无法按配置更新测速，取消旧计数起点，避免下次跨周期误算。
        App_Chassis_Update_Feedback(NULL, 0, 0, dt_s);
        App_Chassis_Stop(output);
        return 0;
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
        return 0;
    }

    if (feedback_result == MOTOR_SAMPLE_READY)
    {
        // 正常准备测速：不输出驱动力，但保留已接受的整车目标。
        // 平滑目标从零重新起步，避免恢复时沿用旧斜坡状态。
        chassis_ramp = (ChassisSpeed){0};
        wheel_target = (WheelSpeed){0};
        for (wheel_index = 0U; wheel_index < WHEEL_COUNT; wheel_index++)
        {
            Pid_Reset(&pid[wheel_index]);
        }
        *output = (ChassisOutput){0};
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
uint8_t App_Chassis_Read_Encoder(const ChassisMotorMap *map,
                                ChassisEncoder *encoder)//读取编码器的量
{
    ChassisEncoder next_encoder;

    if (map == NULL || encoder == NULL)
    {
        return 0;
    }

    // 先读取到局部变量，避免失败时只更新一部分轮子的计数。
    if (BSP_Motor_Read_Count(map->front_left,
                            &next_encoder.front_left) == 0 ||
        BSP_Motor_Read_Count(map->front_right,
                            &next_encoder.front_right) == 0 ||
        BSP_Motor_Read_Count(map->rear_left,
                            &next_encoder.rear_left) == 0 ||
        BSP_Motor_Read_Count(map->rear_right,
                            &next_encoder.rear_right) == 0)
    {
        return 0;
    }

    // 四路全部成功后，再把完整结果交给调用者。
    *encoder = next_encoder;
    return 1;
}
uint8_t App_Chassis_Write_Output(const ChassisMotorMap *map,
                                const ChassisOutput *output)//将四轮算好量给对应电机接口
{
    uint8_t valid_count = 0;

    if (map == NULL || output == NULL)
    {
        return 0;
    }

    // 转成整数之前检查范围；这种写法也能拒绝NaN。
    if (!(output->front_left >= -1000.0f &&
          output->front_left <= 1000.0f) ||
        !(output->front_right >= -1000.0f &&
          output->front_right <= 1000.0f) ||
        !(output->rear_left >= -1000.0f &&
          output->rear_left <= 1000.0f) ||
        !(output->rear_right >= -1000.0f &&
          output->rear_right <= 1000.0f))
    {
        return 0;
    }

    // 每路都尝试设置，统计成功数量，不因一路失败跳过其他轮。
    valid_count += BSP_Motor_Set_Output(
        map->front_left, (int16_t)output->front_left);

    valid_count += BSP_Motor_Set_Output(
        map->front_right, (int16_t)output->front_right);

    valid_count += BSP_Motor_Set_Output(
        map->rear_left, (int16_t)output->rear_left);

    valid_count += BSP_Motor_Set_Output(
        map->rear_right, (int16_t)output->rear_right);

    return valid_count == BSP_MOTOR_COUNT;
}
uint8_t App_Chassis_Control(const ChassisConfig *config,
                           const ChassisMotorMap *map,
                           uint8_t enabled,
                           float dt_s)
{
    ChassisEncoder encoder;
    ChassisOutput output = {0};
    uint8_t control_valid;
    uint32_t motor_index;

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
        App_Chassis_Stop(&output);

        // 不依赖可能出错的轮位映射，逐个尝试将四个接口归零。
        for (motor_index = 0;
             motor_index < BSP_MOTOR_COUNT;
             motor_index++)
        {
            // 无论本路是否成功，都继续尝试其他接口；本分支最终返回0。
            (void)BSP_Motor_Set_Output((BSPMotorId)motor_index, 0);
        }

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
uint8_t App_Chassis_Run(const ChassisMotorMap *map,
                       uint8_t enabled,
                       float dt_s)
{
    uint8_t motion_allowed = enabled;  // B的许可只能被任务逻辑进一步收紧。

    // 禁用或离开第一段循迹后，清除计数与确认标记。
    if (enabled == 0U || chassis_task != CHASSIS_TASK_LINE1)
    {
        App_Line_Reset();
    }

    // 根据当前任务，决定本周期是否允许执行运动控制。
    switch (chassis_task)
    {
        case CHASSIS_TASK_NONE:
        {
            // 没有自动任务，沿用B传入的运动许可。
            // 手动目标由SetTarget保存。
            break;
        }

        case CHASSIS_TASK_LINE1:
        {
            ChassisSpeed line_target = {0};  // 循迹模块交回的整车速度请求。
            LineResult line_result;         // 本周期的循迹处理结果。
            LineAction action = App_Chassis_Get_Line_Action();  // 当前进度要求的动作。

            // 每周期先清掉旧目标，防止暂时无目标时沿用上一次的行驶命令。
            chassis_target = (ChassisSpeed){0};

            // 特殊路段处理尚未完成，迁移后仍保持自动运动关闭。
            motion_allowed = 0U;
            // 最终失败后保持故障，不在下一周期自动重新开始循迹。
            if (enabled == 0U || chassis_status == CHASSIS_STATUS_FAULT)
            {
                break;
            }

            // 传入动作意图，由循迹模块等待合适的特征再执行。
            line_result = App_Line_Update(
                &line_target, dt_s, action);
            switch (line_result)
            {
                case LINE_TARGET_READY:
                {
                    chassis_target = line_target;
                    break;
                }

                case LINE_FEATURE_DONE:
                {
                    // 只完成一处特征，不将整段循迹设为DONE。
                    // 完成反馈携带有效循迹速度，流程交接不主动插入零目标。
                    chassis_target = line_target;
                    // 明确动作完成才推进；普通循迹和等待不推进。
                    if (action == LINE_ACTION_STRAIGHT ||
                        action == LINE_ACTION_LEFT ||
                        action == LINE_ACTION_RIGHT)
                    {
                        line_step++;
                    }
                    break;
                }

                case LINE_FAILED:
                {
                    // 后续寻线超时会返回此结果；由底盘层记录任务故障。
                    chassis_status = CHASSIS_STATUS_FAULT;
                    break;
                }

                default:
                {
                    // LINE_NO_TARGET：暂时没有合适目标，本周期保持零请求。
                    break;
                }
            }
            break;
        }
        default:
        {
            // 其他自动任务尚未实现，暂时保持零输出。
            motion_allowed = 0U;
            break;
        }
    }

    // 无论是否允许运动，都继续执行采样和相应的输出处理。
    return App_Chassis_Control(
        &chassis_config, map, motion_allowed, dt_s);
}