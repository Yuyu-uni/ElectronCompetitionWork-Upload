#include "main.h"
#include "task.h"
#include "drv_math.h"
#include "PID.h"
#include "jy901.h"
#include "motor.h"
#include "Encoder.h"
#include "delay.h"
#include "usart.h"
#include "stm32g4xx_it.h"
#include "BSP.h"
#include "drv_math.h"
#include "FollowingLine_8Chan_GW.h"
#include "StepMotor.h"
extern TIM_HandleTypeDef htim7;
extern I2C_HandleTypeDef hi2c3;

KeyStruct keys1, keys2, keys3, keys4; // 定义按键结构体

// 云台参数目标追踪
PIDController point;
StepMotorStruct stepMotorA, stepMotorB; // 定义步进电机结构体
float x_angle;                          // x轴角度值
float y_angle;                          // y轴角度值

/*
 * @brief  任务初始化函数
 *         初始化PID控制器、传感器、按键等
 * @param  None
 * @retval None
 * @note   该函数在系统启动时调用，确保所有必要的组件都已正确在cubemx配置和初始化。并且如果卡死属于是初始化没有找到该设备，hal库死循环了。
 */
void task_init(void)
{
    pid_init(&point, -0.00002, 0, -0.0001, 0, 0, 1000, 1000, 1, 1000, 1100, 1000, 0, 0, PID_D_First_DISABLE, PID_I_Separate_DISABLE, PID_I_Variable_Speed_DISABLE); // 云台追踪

    // BSP Init--------------------------------------------------------------
    BSP_Init(&keys1, &keys2, &keys3, &keys4);

#ifdef _STEP_DIR_CTRL_MODE_
    StepMotor_Init(&stepMotorA, StepMotorA, 128); // 初始化步进电机A，细分模式为8和128
    StepMotor_Init(&stepMotorB, StepMotorB, 128); // 初始化步进电机B，细分模式为8和128
    StepMotor_SetSpeed(&stepMotorA, 20);          // 设置步进电机A的速度为100转/分钟
    StepMotor_SetSpeed(&stepMotorB, 20);          // 设置步进电机B的速度为100转/分钟
#endif
    StepMotor_Init(&stepMotorA, 1, 128); // 初始化步进电机A，细分模式为8和128

    StepMotor_Init(&stepMotorB, 2, 128); // 初始化步进电机A，细分模式为8和128

    // HAL_TIM_Base_Start_IT(&htim7); // 启动定时器7状态机采样pid数据

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_data, 20);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/**
 * @brief 任务点位跟随函数
 *        该函数用于处理点位跟随任务
 * @param None
 * @retval None
 * @note 该函数在系统主循环中调用，确保步进电机根据PID控制器的输出调整角度。
 */
void task_point_follow(void)
{
    // 定义增量式pid控制器
    volatile int16_t X_angle_error = (int16_t)(uart_rx_values[0]); // 获取X轴角度误差
    volatile int16_t Y_angle_error = (int16_t)(uart_rx_values[1]); // 获取Y轴角度误差
    // 计算pid值
    volatile float X_angle_error_output = PID_Calculate_DSP_Witherror(&point, X_angle_error); // 计算x轴角度误差
    volatile float Y_angle_error_output = PID_Calculate_DSP_Witherror(&point, Y_angle_error); // 计算y轴角度误差
    // 调整舵机角度
    x_angle += X_angle_error_output; // 调整x轴角度
    y_angle += Y_angle_error_output; // 调整y轴角度

    // 限制角度范围
    x_angle = limit(x_angle, 30, 150); // 限制x轴角度在30到150度之间
    y_angle = limit(y_angle, 30, 150); // 限制y轴角度在30到150度之间

    // 设置角度
    StepMotor_SetAngle(&stepMotorA, x_angle); // 设置步进电机A的角度
    StepMotor_SetAngle(&stepMotorB, y_angle); // 设置步进电机B的角度
}
