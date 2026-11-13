// 目标斜坡接口：让目标逐步变化，避免一次突然改变太多。
#ifndef RAMP_H
#define RAMP_H

/// @brief 计算下一次应该给出的目标
/// @param current 上一次给出的目标，例如 0.2 m/s
/// @param target 最终希望达到的目标，例如 1.0 m/s
/// @param accel_limit 加速时每秒最多改变多少（目标绝对值在增大），必须 > 0
/// @param decel_limit 减速时每秒最多改变多少（目标绝对值在减小或反向），必须 > 0
/// @param dt_s 距上次更新经过的秒数，必须 > 0
/// @return 本次计算出的新目标；任一限制参数或dt_s不大于0时返回current不变
/// @note 所有输入必须为有限数值；调用后保存返回值，下次作为current传入。
///       反向时先减速到零，下次调用再反向加速；跨零周期不复用剩余时间。
///       本函数不负责急停；非阻塞，不操作硬件。
float Ramp_Update(float current, float target,
                  float accel_limit, float decel_limit, float dt_s);

#endif
