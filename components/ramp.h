// 让目标逐步变化，避免一次突然改变太多。
#ifndef RAMP_H
#define RAMP_H

/// @brief 计算下一次应该给出的目标
/// @param current 上一次给出的目标，例如0.2 m/s
/// @param target 最终希望达到的目标，例如1 m/s
/// @param rate_limit 每秒最多改变多少，例如2 m/s²
/// @param dt_s 距离上次更新经过的秒数，例如0.01秒
/// @return 本次计算出的新目标，例如0.22 m/s
/// @note 调用后要保存返回值，下次作为current使用。
///       rate_limit或dt_s不大于0时，保持原目标。
///       输入必须是有限数值；函数不操作硬件，也不等待。
float Ramp_Update(float current, float target, float rate_limit, float dt_s);

#endif