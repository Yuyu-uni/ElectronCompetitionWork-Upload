#include "main.h"
#include "Encoder.h"

extern TIM_HandleTypeDef htim1; // 电机1
extern TIM_HandleTypeDef htim2; // 电机2
extern TIM_HandleTypeDef htim3; // 电机3
extern TIM_HandleTypeDef htim4; // 电机4
extern TIM_HandleTypeDef htim5; // 定时器5,用于采集速度

// 选用后轮来测速度
float right_speed; // 电机1速度
float left_speed;  // 电机3速度
double distance;   // 轮子行驶的距离

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    // 清空编码器的数
    __HAL_TIM_SET_COUNTER(&htim1, 0); // 清空电机3的编码器计数值
    __HAL_TIM_SET_COUNTER(&htim3, 0); // 清空电机4的编码器计数值
    // 清空定时器6的数
    __HAL_TIM_SET_COUNTER(&htim5, 0); // 清空定时器5的计数值
    // HAL_TIM_Base_Start_IT(&htim5);    // 启动定时器5中断,每隔1ms采集一次速度
}
