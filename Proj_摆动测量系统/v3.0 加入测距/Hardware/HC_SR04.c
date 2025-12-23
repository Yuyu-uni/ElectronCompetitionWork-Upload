#include "HC_SR04.h"
#include "stm32g4xx_hal.h"
#include "usart.h"
#include "stdio.h"
#include "main.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////
int distance;     // 距离
int distance_num; // 距离计数
int times = 0;    // 时间
int flag = 0;     // 标志位

extern TIM_HandleTypeDef htim4; // 定时器3句柄

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 清除中断标志位
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin); // 清除中断标志位
    // 判断是否为超声波模块的Echo引脚
    if (GPIO_Pin == ECHO_Pin)
    {
        if (flag == 0) // 上升沿
        {
            flag = 1;
            distance_num = 0;                 // 距离计数清零
            __HAL_TIM_SET_COUNTER(&htim4, 0); // 计数器清零
            HAL_TIM_Base_Start_IT(&htim4);    // 使能TIM4
        }
        else if (flag == 1) // 下降沿
        {
            flag = 0;
            HAL_TIM_Base_Stop_IT(&htim4);                                // 关闭TIM3
            times = distance_num * 1000 + __HAL_TIM_GET_COUNTER(&htim4); // 计算时间us
        }
    }
}

// 超声波模块测距
uint16_t HC_SR04_GetDistance(void)
{
    uint16_t distance = 0;
    times = 0;

    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);   // Trig高电平
    delay_us(10);                      //10us                                     
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET); // Trig低电平
    HAL_Delay(60);                                               // 延时60ms

    distance = (times * 0.34262 / 2); // 计算距离，单位mm
    return distance;
}

// 超声波模块测距10次取平均值
uint16_t HC_SR04_GetDistance_Average(void)
{
    uint16_t i = 0;
    uint32_t distance = 0;

    for (i = 0; i < 10; i++)
    {
        distance += HC_SR04_GetDistance();
        HAL_Delay(65); // 延时60ms
    }
    return distance / 10;
}
