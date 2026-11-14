/**
 * @file    servo.c
 * @brief   众灵总线舵机驱动实现：拼 ASCII 指令并走 UART5 发送。
 * @owner   电控B
 */

#include "servo.h"
#include "bsp_uart.h"

#include <stdio.h>

/** 指令缓冲：#001P2500T9999! 共 15 字符，留余量 */
#define SERVO_CMD_SIZE          24U

/**
 * @brief 发送一条已经拼好的指令
 * @param cmd 以 '\0' 结尾的指令字符串
 * @return 无
 * @note   阻塞发送（UART5），单条指令约 1.3 ms
 */
static void Servo_SendCommand(const char *cmd)
{
    uint16_t length = 0U;

    while (cmd[length] != '\0')
    {
        length++;
    }

    BSP_Uart_SendData(BSP_UART_SERVO, (const uint8_t *)cmd, length);
}

/**
 * @brief 把数值夹到 [low, high] 范围
 * @param value 输入值
 * @param low   下界
 * @param high  上界
 * @return 夹取后的值
 */
static uint16_t Servo_Clamp(uint16_t value, uint16_t low, uint16_t high)
{
    if (value < low)
    {
        value = low;
    }
    else if (value > high)
    {
        value = high;
    }

    return value;
}

void Servo_Move(uint8_t id, uint16_t pulse, uint16_t time_ms)
{
    char cmd[SERVO_CMD_SIZE];

    pulse = Servo_Clamp(pulse, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    time_ms = Servo_Clamp(time_ms, 0U, SERVO_TIME_MAX);

    (void)snprintf(cmd, sizeof(cmd), "#%03uP%04uT%04u!", id, pulse, time_ms);

    Servo_SendCommand(cmd);
}

void Servo_ReadPosition(uint8_t id)
{
    char cmd[SERVO_CMD_SIZE];

    (void)snprintf(cmd, sizeof(cmd), "#%03uPRAD!", id);

    Servo_SendCommand(cmd);
}

void Servo_SetId(uint8_t old_id, uint8_t new_id)
{
    char cmd[SERVO_CMD_SIZE];

    (void)snprintf(cmd, sizeof(cmd), "#%03uPID%03u!", old_id, new_id);

    Servo_SendCommand(cmd);
}

void Servo_SetTorque(uint8_t id, uint8_t enable)
{
    char cmd[SERVO_CMD_SIZE];

    if (enable != 0U)
    {
        (void)snprintf(cmd, sizeof(cmd), "#%03uPULR!", id);
    }
    else
    {
        (void)snprintf(cmd, sizeof(cmd), "#%03uPULK!", id);
    }

    Servo_SendCommand(cmd);
}

void Servo_Stop(uint8_t id)
{
    char cmd[SERVO_CMD_SIZE];

    (void)snprintf(cmd, sizeof(cmd), "#%03uPDST!", id);

    Servo_SendCommand(cmd);
}

void Servo_SetMinLimit(uint8_t id)
{
    char cmd[SERVO_CMD_SIZE];

    (void)snprintf(cmd, sizeof(cmd), "#%03uPSMI!", id);

    Servo_SendCommand(cmd);
}

void Servo_SetMaxLimit(uint8_t id)
{
    char cmd[SERVO_CMD_SIZE];

    (void)snprintf(cmd, sizeof(cmd), "#%03uPSMX!", id);

    Servo_SendCommand(cmd);
}
