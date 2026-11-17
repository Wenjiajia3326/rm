#include "line.h"
#include <stddef.h>

uint8_t Line_Calc_Offset(const LineInput *input, float *offset)
{
    // 依据当前传感器尺寸资料，尚未实物核对。
    // 中心到内侧15mm，内侧到外侧28mm；左正右负，单位mm。
    // 仅用于横向位置的粗略估计；修改尺寸时同步更新line.h中的范围说明。
    static const float position_mm[LINE_SENSOR_COUNT] =
    {
        43.0f, 15.0f, 0.0f, -15.0f, -43.0f
    };

    float position_sum_mm = 0.0f;  // 检测到黑色的探头位置之和。
    uint8_t black_count = 0U;      // 检测到黑色的探头数量。
    uint8_t sensor_index;          // 当前正在查看哪个探头。

    if (input == NULL || offset == NULL)
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
        // 每个探头只能用0或1表示检测结果。
        if (input->black[sensor_index] > 1U)
        {
            return 0;
        }

        if (input->black[sensor_index] == 1U)
        {
            position_sum_mm += position_mm[sensor_index];
            black_count++;
        }
    }

    // 全白时不知道线在哪里，不能当成“偏移为0”。
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
