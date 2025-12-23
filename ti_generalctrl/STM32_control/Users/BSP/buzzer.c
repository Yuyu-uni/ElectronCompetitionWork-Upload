#include "buzzer.h"
static uint8_t last_on_flag = 0; // 用于标记蜂鸣器是否处于持续开启状态
static uint16_t last_on_duration = 0; // 用于记录蜂鸣器持续开启的时间

void buzzer_init(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); // 初始化蜂鸣器引脚为低电平（关闭状态）
    last_on_flag = 0; // 初始化标志位
    last_on_duration = 0; // 初始化持续时间
}
void buzzer_on(void)
{
    // Code to turn on the buzzer
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    // Assuming BUZZER_GPIO_Port and BUZZER_Pin are defined in the HAL configuration
}
void buzzer_off(void)
{
    // Code to turn off the buzzer
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

/*
 * @brief 开启蜂鸣器并持续指定时间
 * @param duration: 持续时间，单位为毫秒
 * @note 
 */
void buzzer_last_on(uint16_t duration)
{
    buzzer_on();
    last_on_duration = duration;
    last_on_flag = 1; // 设置持续开启标志
}

void buzzer_tick(void)
{
    if (last_on_flag)
    {
        if (last_on_duration > 0)
        {
            last_on_duration--;
        }
        else
        {
            buzzer_off();
            last_on_flag = 0; // 重置持续开启标志
        }
    }
}
