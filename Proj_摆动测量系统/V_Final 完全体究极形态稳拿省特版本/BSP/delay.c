#include "delay.h"
#include "main.h"

extern TIM_HandleTypeDef htim15; // 定义定时器句柄

// 定时器12实现微妙延时，定时器分频到1MHZ，不能打开自动预装载，是我们手动重装载
void delay_us(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim15, 0); // 清零计数器
    HAL_TIM_Base_Start(&htim15);       // 启动定时器

    while (__HAL_TIM_GET_COUNTER(&htim15) < us)
        ; // 等待计数器达到指定的延时值

    HAL_TIM_Base_Stop(&htim15); // 停止定时器
}
