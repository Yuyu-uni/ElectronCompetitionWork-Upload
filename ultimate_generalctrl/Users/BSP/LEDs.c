#include "leds.h"
static uint8_t last_on_flag[LED_COUNT] = {0}; // 用于标记LED是否处于持续开启状态
static uint16_t last_on_duration[LED_COUNT] = {0}; // 用于记录LED持续开启的时间
void leds_init(void)
{
    // 初始化LED引脚为输出模式并设置初始状态
    HAL_GPIO_WritePin(ULED1_Core_GPIO_Port, ULED1_Core_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ULED2_Core_GPIO_Port, ULED2_Core_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ULED3_Driver_GPIO_Port, ULED3_Driver_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ULED4_Driver_GPIO_Port, ULED4_Driver_Pin, GPIO_PIN_RESET);

}

/* @brief 控制LED灯的开
 * @param led: LED编号ULED1, ULED2, ULED3, ULED4
 */
void leds_on(enum LEDNum led)
{
    switch (led) {
    case ULED1:
        HAL_GPIO_WritePin(ULED1_Core_GPIO_Port, ULED1_Core_Pin, GPIO_PIN_SET);
        break;
    case ULED2:
        HAL_GPIO_WritePin(ULED2_Core_GPIO_Port, ULED2_Core_Pin, GPIO_PIN_SET);
        break;
    case ULED3:
        HAL_GPIO_WritePin(ULED3_Driver_GPIO_Port, ULED3_Driver_Pin, GPIO_PIN_SET);
        break;
    case ULED4:
        HAL_GPIO_WritePin(ULED4_Driver_GPIO_Port, ULED4_Driver_Pin, GPIO_PIN_SET);
        break;
    
    default:
        break;
    }
}
/* @brief 控制LED灯的关
 * @param led: LED编号ULED1, ULED2, ULED3, ULED4
 */
void leds_off(enum LEDNum led)
{
    switch (led) {
    case ULED1:
        HAL_GPIO_WritePin(ULED1_Core_GPIO_Port, ULED1_Core_Pin, GPIO_PIN_RESET);
        break;
    case ULED2:
        HAL_GPIO_WritePin(ULED2_Core_GPIO_Port, ULED2_Core_Pin, GPIO_PIN_RESET);
        break;
    case ULED3:
        HAL_GPIO_WritePin(ULED3_Driver_GPIO_Port, ULED3_Driver_Pin, GPIO_PIN_RESET);
        break;
    case ULED4:
        HAL_GPIO_WritePin(ULED4_Driver_GPIO_Port, ULED4_Driver_Pin, GPIO_PIN_RESET);
        break;
    
    default:
        break;
    }
}
/* @brief 控制LED灯的持续开启
 * @param led: LED编号ULED1, ULED2, ULED3, ULED4
 * @param duration: 持续时间，单位为毫秒
 */
void leds_last_on(enum LEDNum led, uint16_t duration)
{
    leds_on(led);
    last_on_duration[led] = duration;
    last_on_flag[led] = 1; // 设置持续开启标志
}

void leds_tick(enum LEDNum led)
{
    if (last_on_flag[led])
    {
        if (last_on_duration[led] > 0)
        {
            last_on_duration[led]--;
        }
        else
        {
            leds_off(led);
            last_on_flag[led] = 0; // 重置持续开启标志
        }
    }
}
