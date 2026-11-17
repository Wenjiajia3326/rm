#include "line.h"
#include <stddef.h>

uint8_t Line_Analyze(const LineInput *input, LinePattern *pattern)
{
    LinePattern next = {0};  // 本次统计表，所有成员先清零。
    uint8_t sensor_index;    // 当前检查的探头编号。

    if (input == NULL || pattern == NULL)
    {
        return 0;
    }

    if (input->sample_valid != 1U)
    {
        return 0;
    }

    for (sensor_index = 0U;
         sensor_index < LINE_SENSOR_COUNT;
         sensor_index++)
    {
        // 每路只允许用0、1表示白色和黑色。
        if (input->black[sensor_index] > 1U)
        {
            return 0;
        }

        // 白色不参与黑色统计，直接检查下一个探头。
        if (input->black[sensor_index] == 0U)
        {
            continue;
        }

        next.black_count++;

        // 编号1、2、3属于中间三路。
        if (sensor_index >= 1U && sensor_index <= 3U)
        {
            next.middle_count++;
        }

        // 当前为黑色，且位于开头或前一个为白色，就是新的一组。
        // 编号为0时，||会跳过右侧判断，不会读取前一项。
        if (sensor_index == 0U ||
            input->black[sensor_index - 1U] == 0U)
        {
            next.group_count++;
        }
    }

    next.left_black = input->black[0];
    next.right_black = input->black[LINE_SENSOR_COUNT - 1U];

    // 全部检查成功后，再把完整结果交给调用者。
    *pattern = next;
    return 1;
}

uint8_t Line_Calc_Offset(const LineInput *input, float *offset)
{
    // 依据传感器尺寸资料，中间三路横向位置为左15mm、中0、右15mm。
    // 左正右负；中心探头前突，因此这里只是横向偏差的粗略估计。
    static const float middle_position_mm[3] =
    {
        15.0f, 0.0f, -15.0f
    };

    float position_sum_mm = 0.0f;  // 中间黑色探头的位置之和。
    uint8_t black_count = 0U;      // 中间三路中的黑色数量。
    uint8_t sensor_index;          // 当前检查的探头编号。

    if (input == NULL || offset == NULL)
    {
        return 0;
    }

    if (input->sample_valid != 1U)
    {
        return 0;
    }

    // 完整五路数据都应当只包含0或1。
    for (sensor_index = 0U;
         sensor_index < LINE_SENSOR_COUNT;
         sensor_index++)
    {
        if (input->black[sensor_index] > 1U)
        {
            return 0;
        }
    }

    // 只统计编号1、2、3，也就是中间三路。
    for (sensor_index = 1U;
         sensor_index <= 3U;
         sensor_index++)
    {
        if (input->black[sensor_index] == 1U)
        {
            // 探头编号1到3，对应位置数组的下标0到2。
            position_sum_mm += middle_position_mm[sensor_index - 1U];
            black_count++;
        }
    }

    // 中间三路全白时，不能提供普通纠偏所需的偏差。
    if (black_count == 0U)
    {
        return 0;
    }

    *offset = position_sum_mm / (float)black_count;
    return 1;
}

float Line_Calc_Turn(float offset_mm, float kp, float max_turn)
{
    float turn;  // 本次希望底盘转动的角速度，单位rad/s。

    // 偏移越大，要求的转向角速度越大。
    turn = kp * offset_mm;

    // 限制转向速度，避免纠偏过猛。
    if (turn > max_turn)
    {
        turn = max_turn;
    }
    else if (turn < -max_turn)
    {
        turn = -max_turn;
    }

    return turn;
}
