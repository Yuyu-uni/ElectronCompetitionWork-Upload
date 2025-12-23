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
#include "StepMotor.h"
extern TIM_HandleTypeDef htim7;
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

    // 调节pid，注意所有函数P和D正负号对应的输出结果就是正负的,p和d是同号的

    // 角度度环和速度环的PID控制器初始化
    Angle.pid_init(&Angle, 130, 150.0f / 200.0f, 0.5, 0, 0, 1000, 10000, 1, 0, 0, 0, 0, 0, PID_D_First_DISABLE, PID_I_Separate_DISABLE, PID_I_Variable_Speed_DISABLE); // 转向环

    // p[0,100],d[0,1000]
    Follow.pid_init(&Follow, 11, 0, 0, 0, 0, 1.5, 5000, 1, 0, 0, 0, 0, 0, PID_D_First_DISABLE, PID_I_Separate_ENABLE, PID_I_Variable_Speed_DISABLE); // 位置环

    speed.pid_init(&speed, 0.23, 0.23 / 200, 0, 0, 0.75, 1.5, 30, 1, 0, 0, 0, 0, 0, PID_D_First_DISABLE, PID_I_Separate_ENABLE, PID_I_Variable_Speed_DISABLE); // 速度环

    pid_init(&point, -15.0f / 10000000.0f, 0, 0, 0, 0, 1000, 1000, 1, 1000, 1100, 1000, 0, 0, PID_D_First_DISABLE, PID_I_Separate_DISABLE, PID_I_Variable_Speed_DISABLE); // 云台追踪

    jy901_zeroXY();       // 归零角度
    jy901_dire_read_IT(); // 中断读取jy901数据

    // BSP Init--------------------------------------------------------------
    KeyStruct keys1, keys2, keys3, keys4; // 定义按键结构体
    BSP_Init(&keys1, &keys2, &keys3, &keys4);

#ifdef _STEP_DIR_CTRL_MODE_
    StepMotor_Init(&stepMotorA, StepMotorA, 128); // 初始化步进电机A，细分模式为8和128
    StepMotor_Init(&stepMotorB, StepMotorB, 128); // 初始化步进电机B，细分模式为8和128
    StepMotor_SetSpeed(&stepMotorA, 20);          // 设置步进电机A的速度为20转/分钟
    StepMotor_SetSpeed(&stepMotorB, 20);          // 设置步进电机B的速度为20转/分钟
#endif

    Motor_Init();   // 打开四个电机PWM通道
    Encoder_Init(); // 清空编码器计数值，并开启编码器

    HAL_TIM_Base_Start_IT(&htim7); // 启动定时器7状态机采样pid数据
    // 打开串口中断，接受Maxicam指令
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

// 测试函数，输出50%占空比的PWM信号驱动电机，同时读出编码器的值，并且将数值打印出来
void test_motor(void)
{
    Load(500, 500, 500, 500); // 设置四个电机的 PWM 输出
    delay_us(100);            // 延时100ms
    Uart_printf(&huart4, "Motor 1 Speed: %.2f, Motor 3 Speed: %.2f\r\n", right_speed, left_speed);
}

// 二维卡尔曼滤波测试相关变量
float kalman_state[2] = {0.0f, 0.0f};           // 状态：[x, y]
float kalman_errorCovariance[2] = {1.0f, 1.0f}; // 误差协方差
uint8_t kalman_data_ready = 0;                  // 数据准备标志
float input_measurement[2] = {0.0f, 0.0f};      // 输入测量值
// 二维卡尔曼滤波测试函数
void test_kalman_filter(void)
{

    // 定义过程模型矩阵 (简单模型，状态保持)
    static float processModel[2][2] = {
        {1.0f, 0.0f}, // x方向
        {0.0f, 1.0f}  // y方向
    };

    // 定义测量噪声矩阵
    static float measurementNoise[2][2] = {
        {0.1f, 0.0f}, // x方向测量噪声
        {0.0f, 0.1f}  // y方向测量噪声
    };

    // 定义过程噪声矩阵
    static float processNoise[2][2] = {
        {0.01f, 0.0f}, // x方向过程噪声
        {0.0f, 0.01f}  // y方向过程噪声
    };

    // 执行卡尔曼滤波
    kalmanFilter2D(input_measurement, kalman_state, kalman_errorCovariance,
                   processModel, measurementNoise, processNoise);
}
