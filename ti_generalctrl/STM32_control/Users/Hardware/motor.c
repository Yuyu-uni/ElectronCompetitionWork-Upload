#include "motor.h"
#include "stm32g4xx_hal.h"
#include "main.h"

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim20;
/**
 * @brief 限制电机 PWM 输出范围
 * @param motoA 电机 A 的 PWM 值指针
 * @param motoB 电机 B 的 PWM 值指针
 */
void Limit(float *motoA, float *motoB)
{
    if (*motoA > PWM_MAX)
        *motoA = PWM_MAX;
    if (*motoA < PWM_MIN)
        *motoA = PWM_MIN;

    if (*motoB > PWM_MAX)
        *motoB = PWM_MAX;
    if (*motoB < PWM_MIN)
        *motoB = PWM_MIN;
}

/**
 * @brief 求绝对值
 * @param p 输入值
 * @return 返回绝对值
 */
int GFP_abs(int p)
{
    return (p > 0) ? p : (-p);
}

/**
 * @brief 设置四个电机的 PWM 输出和方向
 * @param moto1 电机 1 的 PWM 值
 * @param moto2 电机 2 的 PWM 值
 * @param moto3 电机 3 的 PWM 值
 * @param moto4 电机 4 的 PWM 值
 */
void Load(int moto1, int moto2, int moto3, int moto4)
{
    // 设置电机 1 的方向和 PWM
    if (moto1 > 0)
    {
        RB_IN(0);
    }
    else
    {
        RB_IN(1);
    }
    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_1, GFP_abs(moto1)); // 使用 TIM5 通道 1

    // 设置电机 2 的方向和 PWM
    if (moto2 > 0)
    {
        RF_IN(0);
    }
    else
    {
        RF_IN(1);
    }
    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_2, GFP_abs(moto2)); // 使用 TIM5 通道 2

    // 设置电机 3 的方向和 PWM
    if (moto3 > 0)
    {
        LB_IN(0);
    }
    else
    {
        LB_IN(1);
    }
    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_3, GFP_abs(moto3)); // 使用 TIM20 通道 3

    // 设置电机 4 的方向和 PWM
    if (moto4 > 0)
    {
        LF_IN(0);
    }
    else
    {
        LF_IN(1);
    }
    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_4, GFP_abs(moto4)); // 使用 TIM20 通道 4
}

char PWM_Zero = 0, stop = 0;

/**
 * @brief 停止电机
 * @param Med_Jiaodu 中间角度
 * @param Jiaodu 当前角度
 */
void Stop(float *Med_Jiaodu, float *Jiaodu)
{
    if (GFP_abs(*Jiaodu - *Med_Jiaodu) > 60)
    {
        Load(PWM_Zero, PWM_Zero, PWM_Zero, PWM_Zero);
        stop = 1;
    }
}

void Motor_Init(void)
{
    // 启动 PWM 输出
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_4);
}
