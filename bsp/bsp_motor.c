// 负责底盘电机的硬件操作，不计算轮速和PI。
#include "bsp_motor.h"
#include "bsp_config.h"
#include <stddef.h>

// 四路PWM全部启动成功后才允许处理输出，初始化和输出由同一控制任务调用。
static uint8_t motor_output_ready = 0;
// 分别保存四路最后一次成功设置输出的时刻，单位ms。
static uint32_t motor_last_update_ms[BSP_MOTOR_COUNT];

// 记录每一路是否有非零驱动请求，0表示无需进行超时置零。
static uint8_t motor_output_active[BSP_MOTOR_COUNT];


uint8_t BSP_Motor_Encoder_Init(void)
{
    uint8_t valid_count = 0;

    // 每路都尝试启动，累加的是比较后的成功标志，不是HAL状态码。
    valid_count += (HAL_TIM_Encoder_Start(
        BSP_MOTOR_1_ENCODER_TIM, TIM_CHANNEL_ALL) == HAL_OK);

    valid_count += (HAL_TIM_Encoder_Start(
        BSP_MOTOR_2_ENCODER_TIM, TIM_CHANNEL_ALL) == HAL_OK);

    valid_count += (HAL_TIM_Encoder_Start(
        BSP_MOTOR_3_ENCODER_TIM, TIM_CHANNEL_ALL) == HAL_OK);

    valid_count += (HAL_TIM_Encoder_Start(
        BSP_MOTOR_4_ENCODER_TIM, TIM_CHANNEL_ALL) == HAL_OK);

    // 任一路失败都返回0，由上层保持电机禁止运行。
    return valid_count == 4;
}

uint8_t BSP_Motor_Read_Count(BSPMotorId motor_id,
                            uint16_t *count)
{
    TIM_HandleTypeDef *encoder_tim;

    if (count == NULL)
    {
        return 0;
    }

    // 根据电机接口编号，选择对应的编码器定时器。
    switch (motor_id)
    {
        case BSP_MOTOR_1:
            encoder_tim = BSP_MOTOR_1_ENCODER_TIM;
            break;

        case BSP_MOTOR_2:
            encoder_tim = BSP_MOTOR_2_ENCODER_TIM;
            break;

        case BSP_MOTOR_3:
            encoder_tim = BSP_MOTOR_3_ENCODER_TIM;
            break;

        case BSP_MOTOR_4:
            encoder_tim = BSP_MOTOR_4_ENCODER_TIM;
            break;

        default:
            return 0;
    }

    // 读取当前计数，不清零；计数差由motor组件计算。
    *count = (uint16_t)__HAL_TIM_GET_COUNTER(encoder_tim);

    return 1;
}
uint8_t BSP_Motor_Set_Output(BSPMotorId motor_id,
                             int16_t duty)
{
    TIM_HandleTypeDef *pwm_tim;
    uint32_t pwm_channel;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    uint32_t period_count;
    uint32_t compare;
    GPIO_PinState in2_state;
    GPIO_PinState current_in2;
    uint32_t saved_primask;

    // PWM未初始化成功，或请求超出范围时，不操作硬件。
    if (motor_output_ready == 0 ||
        duty < -1000 || duty > 1000)
    {
        return 0;
    }

    // 根据电机接口编号，找到PWM和IN2对应的硬件。
    switch (motor_id)
    {
        case BSP_MOTOR_1:
            pwm_tim = BSP_MOTOR_1_PWM_TIM;
            pwm_channel = BSP_MOTOR_1_PWM_CHANNEL;
            in2_port = BSP_MOTOR_1_IN2_PORT;
            in2_pin = BSP_MOTOR_1_IN2_PIN;
            break;

        case BSP_MOTOR_2:
            pwm_tim = BSP_MOTOR_2_PWM_TIM;
            pwm_channel = BSP_MOTOR_2_PWM_CHANNEL;
            in2_port = BSP_MOTOR_2_IN2_PORT;
            in2_pin = BSP_MOTOR_2_IN2_PIN;
            break;

        case BSP_MOTOR_3:
            pwm_tim = BSP_MOTOR_3_PWM_TIM;
            pwm_channel = BSP_MOTOR_3_PWM_CHANNEL;
            in2_port = BSP_MOTOR_3_IN2_PORT;
            in2_pin = BSP_MOTOR_3_IN2_PIN;
            break;

        case BSP_MOTOR_4:
            pwm_tim = BSP_MOTOR_4_PWM_TIM;
            pwm_channel = BSP_MOTOR_4_PWM_CHANNEL;
            in2_port = BSP_MOTOR_4_IN2_PORT;
            in2_pin = BSP_MOTOR_4_IN2_PIN;
            break;

        default:
            return 0;
    }

    // 从0数到ARR，一个周期共有ARR+1个计数。
    period_count = __HAL_TIM_GET_AUTORELOAD(pwm_tim) + 1U;

    if (duty >= 0)
    {
        // 正向：IN2为低，PWM高电平期间驱动。
        in2_state = GPIO_PIN_RESET;
        compare = period_count * (uint32_t)duty / 1000U;
    }
    else
    {
        // 反向：IN2为高，PWM低电平期间驱动。
        in2_state = GPIO_PIN_SET;
        compare = period_count
                - period_count * (uint32_t)(-duty) / 1000U;
    }

    // 保存中断状态，避免硬件切换过程被普通中断打断。
    saved_primask = __get_PRIMASK();
    __disable_irq();

    // 仅临时关闭当前通道的比较值预装载，使本次写入直接生效。
    __HAL_TIM_DISABLE_OCxPRELOAD(pwm_tim, pwm_channel);

    // 读取输出寄存器中的IN2电平，不代表读取电机实际方向。
    current_in2 = ((in2_port->ODR & in2_pin) != 0U)
                ? GPIO_PIN_SET : GPIO_PIN_RESET;

    if (current_in2 != in2_state)
    {
        // 先准备IN1，再切换IN2；两次写入不是同时发生。
        // 准备期间可能短暂施加原方向驱动，波形和电流须上板验证。
        if (in2_state == GPIO_PIN_SET)
        {
            __HAL_TIM_SET_COMPARE(pwm_tim, pwm_channel, period_count);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(pwm_tim, pwm_channel, 0U);
        }

        HAL_GPIO_WritePin(in2_port, in2_pin, in2_state);
    }

    // 写入目标比较值；立即更新可能改变本周期脉宽。
    __HAL_TIM_SET_COMPARE(pwm_tim, pwm_channel, compare);

    __HAL_TIM_ENABLE_OCxPRELOAD(pwm_tim, pwm_channel);
    ___set_PRIMASK(saved_primask);

// 只有请求通过检查并写入硬件后，才更新对应记录。
motor_last_update_ms[motor_id] = HAL_GetTick();
motor_output_active[motor_id] = (duty != 0);

return 
uint8_t BSP_Motor_Output_Init(void)
{
    uint8_t valid_count = 0;

    motor_output_ready = 0;

    // 首次启动前，将四路IN2设为低电平。
    HAL_GPIO_WritePin(BSP_MOTOR_1_IN2_PORT,
                      BSP_MOTOR_1_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BSP_MOTOR_2_IN2_PORT,
                      BSP_MOTOR_2_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BSP_MOTOR_3_IN2_PORT,
                      BSP_MOTOR_3_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BSP_MOTOR_4_IN2_PORT,
                      BSP_MOTOR_4_IN2_PIN, GPIO_PIN_RESET);

    // 四路PWM均从零占空比开始。
    __HAL_TIM_SET_COMPARE(BSP_MOTOR_1_PWM_TIM,
                          BSP_MOTOR_1_PWM_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(BSP_MOTOR_2_PWM_TIM,
                          BSP_MOTOR_2_PWM_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(BSP_MOTOR_3_PWM_TIM,
                          BSP_MOTOR_3_PWM_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(BSP_MOTOR_4_PWM_TIM,
                          BSP_MOTOR_4_PWM_CHANNEL, 0U);

    valid_count += (HAL_TIM_PWM_Start(
        BSP_MOTOR_1_PWM_TIM, BSP_MOTOR_1_PWM_CHANNEL) == HAL_OK);

    valid_count += (HAL_TIM_PWM_Start(
        BSP_MOTOR_2_PWM_TIM, BSP_MOTOR_2_PWM_CHANNEL) == HAL_OK);

    valid_count += (HAL_TIM_PWM_Start(
        BSP_MOTOR_3_PWM_TIM, BSP_MOTOR_3_PWM_CHANNEL) == HAL_OK);

    valid_count += (HAL_TIM_PWM_Start(
        BSP_MOTOR_4_PWM_TIM, BSP_MOTOR_4_PWM_CHANNEL) == HAL_OK);

    motor_output_ready = (valid_count == 4);
    return motor_output_ready;
}