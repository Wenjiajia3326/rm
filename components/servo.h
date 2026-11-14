/**
 * @file    servo.h
 * @brief   众灵总线舵机驱动：ASCII 指令组包与发送（UART5，只发不收）。
 * @owner   电控B
 *
 * 协议（《总线舵机通信协议》）：
 *   #<ID 3位>P<脉宽 4位>T<时间 4位>!     例：#001P1500T1000!
 *   ID 001~004，脉宽 500~2500，时间 0~9999 ms，串口 115200 8N1
 *
 * 本模块只负责"拼指令 + 发出去"，具体脉宽值来自标定表（机械B 提供）。
 */

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

/** 舵机脉宽范围（手册规定） */
#define SERVO_PULSE_MIN         500U
#define SERVO_PULSE_MAX         2500U

/** 舵机时间参数上限（ms） */
#define SERVO_TIME_MAX          9999U

/**
 * @brief 让某个舵机转到目标脉宽位置
 * @param id      舵机 ID（1~4）
 * @param pulse   目标脉宽（500~2500）
 * @param time_ms 期望转动时间（0~9999 ms），0 表示最快
 * @return 无
 * @note   脉宽/时间超出协议范围会被夹到边界，避免发出非法指令
 */
void Servo_Move(uint8_t id, uint16_t pulse, uint16_t time_ms);

/**
 * @brief 读取舵机当前位置（回包走 UART5，需接收才能用）
 * @param id 舵机 ID
 * @return 无
 * @note   只发指令，不解析回包；当前工程 UART5 只发，回包会被丢弃
 */
void Servo_ReadPosition(uint8_t id);

/**
 * @brief 修改舵机 ID（标定工具用）
 * @param old_id 当前 ID
 * @param new_id 目标 ID（1~254）
 * @return 无
 * @note   4 只舵机出厂都是 000，必须逐只单独接线改成 001~004
 */
void Servo_SetId(uint8_t old_id, uint8_t new_id);

/**
 * @brief 释放 / 恢复扭力
 * @param id     舵机 ID
 * @param enable 1 = 恢复扭力（PULR），0 = 释放扭力（PULK）
 * @return 无
 * @note   上电释力 + 缓动到 HOME 后再恢复扭力，可避免上电猛冲
 */
void Servo_SetTorque(uint8_t id, uint8_t enable);

/**
 * @brief 停止在当前位置（之后需重新发目标才能再动）
 * @param id 舵机 ID
 * @return 无
 */
void Servo_Stop(uint8_t id);

/**
 * @brief 把当前位置设为最小限位（先手动摆到位再发）
 * @param id 舵机 ID
 * @return 无
 */
void Servo_SetMinLimit(uint8_t id);

/**
 * @brief 把当前位置设为最大限位（先手动摆到位再发）
 * @param id 舵机 ID
 * @return 无
 */
void Servo_SetMaxLimit(uint8_t id);

#endif /* SERVO_H */
