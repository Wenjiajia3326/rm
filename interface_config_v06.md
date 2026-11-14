# 悍妇战队接口配置 V0.6（按最终原理图修订）

依据：SCH_Schematic1_2026-11-14.pdf。电机驱动确定为四路 AT8236；本次同步 rm.ioc 与 Core 初始化代码，不包含电机驱动业务。

## 底盘电机

| 电机 | PWM，连接驱动 IN1 | GPIO，连接驱动 IN2 | 编码器 A/B |
| --- | --- | --- | --- |
| M1 | PA1 / TIM5_CH2 / M1_PWM | PB0 / M1_IN2 | PA15、PB3 / TIM2 |
| M2 | PA2 / TIM5_CH3 / M2_PWM | PB4 / M2_IN2 | PA6、PA7 / TIM3 |
| M3 | PA3 / TIM5_CH4 / M3_PWM | PB9 / M3_IN2 | PB6、PB7 / TIM4 |
| M4 | PA8 / TIM1_CH1 / M4_PWM | PC5 / M4_IN2 | PC6、PC7 / TIM8 |

电机编号和车上实际轮位、编码器 A/B 相序、电机正转方向需要上板逐一确认。旧版 Mx_IN1 GPIO 标签改为 Mx_IN2，PWM 标签仍保留，用于连接 AT8236 的 IN1。

取消旧 TB6612 的 PA5 MOTOR_STBY；PA4、PA5 配置为模拟输入。后续 BSP 必须按 AT8236 真值表处理正反转、PWM 极性和停止方式，不能直接沿用 TB6612 的 STBY 和方向逻辑。

TIM1 与 TIM5 保持 64MHz 定时器时钟，PSC=0、ARR=3199，PWM 20kHz，初始 CCR=0；两只定时器未配置相位同步。编码器 TIM2/3/4/8 参数保持原值。

## 通信与灰度接口

| 用途 | 外设 | STM32 TX | STM32 RX | 其他信号 |
| --- | --- | --- | --- | --- |
| 外接 CH340 调试 | USART1 | PA9 | PA10 | 共地；TX 接对方 RX，RX 接对方 TX |
| 视觉通信 | USART3 | PB10 | PB11 | 与舵机独立 |
| LoRa | UART4 | PC10 | PC11 | PB1=M0，PB5=M1，PC4=AUX |
| 总线舵机，只发送 | UART5 | PC12 | 不启用 | PD2 配置为模拟输入；电气连接按舵机手册及原理图 |

LoRa 按当前原理图保留三个辅助信号连接：PB1/LORA_M0 与 PB5/LORA_M1 为推挽输出，初始低电平；PC4/LORA_AUX 为无上下拉输入，不驱动模块 AUX 输出。M0/M1 的低低模式须与实际模块手册和配置一致。

五路灰度输入依次为 PB8、PB12、PB13、PB14、PB15，均保持无上下拉输入。

四路串口保持原有 115200、8N1 和中断优先级 5。UART5 改为 TX only，保留中断以支持后续中断发送。115200 为当前工程值，不代表外接设备的实际波特率已经逐一验证；联调时按设备配置确认。

## 时钟、调试与验证范围

保持 HSI+PLL 64MHz、CMake、SWD（PA13/PA14），JTAG 禁用以释放 PA15/PB3/PB4。PA0 未配置，不承担底盘 PWM；本自制 PCB 的 PA8 是否接其他支路应以最终硬件为准，不沿用旧开发板 LED 假设。

本次直接同步 .ioc 和生成代码中的对应配置；未调用 CubeMX 重新生成整个工程。后续重新生成时需使用匹配的 CubeMX/F1 固件版本，并复核 GPIO、UART5 与本表一致。

配置检查与固件编译由本次修改后执行；未上板验证。编译通过不等于电机、LoRa、视觉或舵机已经通信成功。
