#include "delay.h"
#include "main.h"

extern TIM_HandleTypeDef htim6; // 定义定时器句柄

// 定时器6无中断实现微妙延时，定时器分频到1MHZ，不能打开自动预装载，是我们手动重装载

/*
 * @brief 延时函数，使用定时器6实现微秒级延时
 * @note 阻塞运行，可以放中断里面
 * @param us 延时时间，单位为微秒
 * @return 无返回值
 */
void delay_us(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim6, 0); // 清零计数器
    HAL_TIM_Base_Start(&htim6);       // 启动定时器

    while (__HAL_TIM_GET_COUNTER(&htim6) < us)
        ; // 等待计数器达到指定的延时值

    HAL_TIM_Base_Stop(&htim6); // 停止定时器
}
