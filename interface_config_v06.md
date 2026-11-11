# 悍妇战队接口配置 V0.6

当前有效方案恢复V0.4的PWM映射：M1=PA1/TIM5_CH2，M2=PA2/TIM5_CH3，M3=PA3/TIM5_CH4，M4=PA8/TIM1_CH1。
PA0保留WK-UP按键，不配置为PWM；V0.5四通道集中方案已撤销，不用于接线。
TIM1与TIM5均按64MHz定时器时钟配置PSC=0、ARR=3199，PWM 20kHz，初始CCR=0；两只定时器同频但未配置相位同步。
PA8有板载LED支路，接驱动前核对波形和电平。
其他已补齐配置保留：TIM2/3/4/8编码器、方向GPIO、PA5 STBY、五路灰度、USART1/3与UART4/5、SWD。
保持HSI+PLL 64MHz和CMake。UART4/5的115200 8N1仍为待确认占位参数。
尚未添加电机启动业务，尚无上板验证。
