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
/// @brief 五路灰度图案的分析结果。
typedef struct
{
    uint8_t black_count;   // 五路中检测到黑色的探头总数，0到5。
    uint8_t middle_count;  // 中间三路检测到黑色的数量，0到3。
    uint8_t group_count;   // 相邻黑色形成的组数，0到3。
    uint8_t left_black;    // 最左探头是否为黑色，1是、0否。
    uint8_t right_black;   // 最右探头是否为黑色，1是、0否。
} LinePattern;
/// @brief 分析本次五路灰度的黑色数量、分组和外侧读数。
/// @param input 本次五路采样，各黑白值只允许0或1，sample_valid必须为1。
/// @param pattern 保存分析结果。
/// @return 成功返回1；参数或采样无效返回0，结果保持不变。
/// @note 全白也能分析成功，分组仅表示数组中相邻的黑色，结果不直接代表转弯或到站。
uint8_t Line_Analyze(const LineInput *input, LinePattern *pattern);


/// @brief 判断是否为一段清晰的中间线，要求两侧外侧探头均为白色。
/// @param pattern 来自Line_Analyze成功分析的本周期图案。
/// @return 中间有1或2个连续黑色且外侧全白时返回1，否则返回0；NULL返回0。
/// @note 只判断图案，不代表已经进入正确支路或完成转弯。
uint8_t Line_Is_Clear(const LinePattern *pattern);

/// @brief 判断中间三路是否有1或2个连续黑色，允许外侧同时检测到黑色。
/// @param input 已通过Line_Analyze验证的本周期有效五路采样。
/// @return 中间线可用返回1；中间全白、全黑、分开的101或NULL返回0。
/// @note 调用者先验证采样；本函数不读硬件，不改变状态，不判断路线或路口。
uint8_t Line_Has_Middle(const LineInput *input);

/// @brief 根据中间三路黑色检测结果估计线路横向偏移。
/// @param input 完整五路采样，black各项及sample_valid只允许0或1。
/// @param offset 保存偏移估计，单位mm，左正右负，范围-15到+15。
/// @return 成功返回1；参数无效或中间三路全白返回0，输出保持不变。
/// @note 外侧两路不参与偏差平均，保留给特征及恢复逻辑使用。
///       结果是粗略估计，返回1不代表已排除路口、宽黑区域等特殊图案。
uint8_t Line_Calc_Offset(const LineInput *input, float *offset);
/// @brief 根据线路偏移计算转向角速度。
/// @param offset_mm 线路偏移估计，单位mm，左正右负。
/// @param kp 比例系数，必须非负，单位(rad/s)/mm。
/// @param max_turn 转向角速度的绝对值上限，必须非负，单位rad/s。
/// @return 转向角速度，左转为正，单位rad/s。
/// @note 调用者保证输入有限、偏移可用，且已排除特殊图案。
float Line_Calc_Turn(float offset_mm, float kp, float max_turn);
#endif