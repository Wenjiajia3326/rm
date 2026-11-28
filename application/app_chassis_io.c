// 底盘硬件读写：按轮位映射调用 BSP。
#include "app_chassis_io.h"
#include "app_config.h"
#include <stddef.h>

// 输入为车身方向PI输出；输出为BSP硬件方向千分比。
static int16_t Chassis_IO_ToDuty(float output, int8_t output_sign)
{
    float duty = output * output_sign;
    if (duty > 0.0f)
    {
        duty *= CHASSIS_POSITIVE_DUTY_SCALE;
    }
    if (duty > CHASSIS_HARDWARE_DUTY_MAX)
    {
        duty = CHASSIS_HARDWARE_DUTY_MAX;
    }
    else if (duty < CHASSIS_HARDWARE_DUTY_MIN)
    {
        duty = CHASSIS_HARDWARE_DUTY_MIN;
    }
    // 与单轮测试一致，补偿后再四舍五入到整数千分比。
    return (int16_t)(duty >= 0.0f ? duty + 0.5f : duty - 0.5f);
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
        map->front_left, Chassis_IO_ToDuty(output->front_left, CHASSIS_OUTPUT_SIGN_FL));

    valid_count += BSP_Motor_Set_Output(
        map->front_right, Chassis_IO_ToDuty(output->front_right, CHASSIS_OUTPUT_SIGN_FR));

    valid_count += BSP_Motor_Set_Output(
        map->rear_left, Chassis_IO_ToDuty(output->rear_left, CHASSIS_OUTPUT_SIGN_RL));

    valid_count += BSP_Motor_Set_Output(
        map->rear_right, Chassis_IO_ToDuty(output->rear_right, CHASSIS_OUTPUT_SIGN_RR));

    return valid_count == BSP_MOTOR_COUNT;
}

void Chassis_IO_TryZeroAll(void)
{
    for (uint32_t motor_index = 0; motor_index < BSP_MOTOR_COUNT; motor_index++)
    {
        // 不依赖轮位映射；任一路失败也继续尝试其余接口。
        (void)BSP_Motor_Set_Output((BSPMotorId)motor_index, 0);
    }
}
