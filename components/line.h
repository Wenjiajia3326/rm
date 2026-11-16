#ifndef LINE_H
#define LINE_H

#include <stdint.h>

#define LINE_SENSOR_COUNT 5U

/// @brief 一次采样得到的五路黑线检测结果。
typedef struct
{
    // 按车体左到右排列：0最左，2中间，4最右。
    // 每项1表示检测到黑色，0表示检测到白色。
    uint8_t black[LINE_SENSOR_COUNT];

    // 1表示本次采样有效；0表示没有取得可信的新数据。
    uint8_t sample_valid;
} LineInput;

/// @brief 根据五路黑色检测结果计算线路偏移。
/// @param input 本次采样，black各项及sample_valid只允许0或1。
/// @param offset 保存偏移，左正右负，范围-2到+2，无单位。
/// @return 可计算返回1；参数无效或全白返回0，输出保持不变。
/// @note 使用位置编号求平均，不代表真实距离。
///       返回1不代表已经排除路口、停线等特殊图案。
uint8_t Line_Calc_Offset(const LineInput *input, float *offset);

#endif