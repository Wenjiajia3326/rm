#include "bsp_gray.h"
#include "main.h"
#include <stddef.h>

uint8_t BSP_Gray_Read(uint8_t level[5])
{
    if (level == NULL)
    {
        return 0;
    }

    // 按通道编号保存原始电平，探头左右位置对应关系由应用层处理。
    level[0] = (HAL_GPIO_ReadPin(
        GRAY_1_GPIO_Port, GRAY_1_Pin) == GPIO_PIN_SET);

    level[1] = (HAL_GPIO_ReadPin(
        GRAY_2_GPIO_Port, GRAY_2_Pin) == GPIO_PIN_SET);

    level[2] = (HAL_GPIO_ReadPin(
        GRAY_3_GPIO_Port, GRAY_3_Pin) == GPIO_PIN_SET);

    level[3] = (HAL_GPIO_ReadPin(
        GRAY_4_GPIO_Port, GRAY_4_Pin) == GPIO_PIN_SET);

    level[4] = (HAL_GPIO_ReadPin(
        GRAY_5_GPIO_Port, GRAY_5_Pin) == GPIO_PIN_SET);

    return 1;
}
