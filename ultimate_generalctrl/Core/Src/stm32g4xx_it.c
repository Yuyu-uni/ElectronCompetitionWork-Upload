/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32g4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Encoder.h"
#include "motor.h"
#include "bsp.h"
#include "StepMotor.h"
#include "jy901.h"
#include "PID.h"
#include "JY901S_REG.h"
#include "task.h"
#include "pid.h"
#include "drv_math.h"
#include "usart.h"
#include "Encoder.h"
#include "FollowingLine_8Chan_GW.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim1; // 电机1的编码器计数定时器
extern TIM_HandleTypeDef htim3; // 电机3的编码器计数定时器

extern float right_speed; // 电机1的速度
extern float left_speed;  // 电机3的速度

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1)
    {
    }
    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
    /* USER CODE BEGIN SVCall_IRQn 0 */

    /* USER CODE END SVCall_IRQn 0 */
    /* USER CODE BEGIN SVCall_IRQn 1 */

    /* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
    /* USER CODE BEGIN PendSV_IRQn 0 */

    /* USER CODE END PendSV_IRQn 0 */
    /* USER CODE BEGIN PendSV_IRQn 1 */

    /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles FDCAN1 interrupt 0.
 */
void FDCAN1_IT0_IRQHandler(void)
{
    /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

    /* USER CODE END FDCAN1_IT0_IRQn 0 */
    HAL_FDCAN_IRQHandler(&hfdcan1);
    /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

    /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
 * @brief This function handles I2C2 event interrupt / I2C2 wake-up interrupt through EXTI line 24.
 */
void I2C2_EV_IRQHandler(void)
{
    /* USER CODE BEGIN I2C2_EV_IRQn 0 */

    /* USER CODE END I2C2_EV_IRQn 0 */
    HAL_I2C_EV_IRQHandler(&hi2c2);
    /* USER CODE BEGIN I2C2_EV_IRQn 1 */

    /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
 * @brief This function handles I2C2 error interrupt.
 */
void I2C2_ER_IRQHandler(void)
{
    /* USER CODE BEGIN I2C2_ER_IRQn 0 */

    /* USER CODE END I2C2_ER_IRQn 0 */
    HAL_I2C_ER_IRQHandler(&hi2c2);
    /* USER CODE BEGIN I2C2_ER_IRQn 1 */

    /* USER CODE END I2C2_ER_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
 */
void USART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */

    /* USER CODE END USART1_IRQn 0 */
    HAL_UART_IRQHandler(&huart1);
    /* USER CODE BEGIN USART1_IRQn 1 */

    /* USER CODE END USART1_IRQn 1 */
}

/**
 * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
 */
void USART2_IRQHandler(void)
{
    /* USER CODE BEGIN USART2_IRQn 0 */

    /* USER CODE END USART2_IRQn 0 */
    HAL_UART_IRQHandler(&huart2);
    /* USER CODE BEGIN USART2_IRQn 1 */

    /* USER CODE END USART2_IRQn 1 */
}

/**
 * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
 */
void USART3_IRQHandler(void)
{
    /* USER CODE BEGIN USART3_IRQn 0 */

    /* USER CODE END USART3_IRQn 0 */
    HAL_UART_IRQHandler(&huart3);
    /* USER CODE BEGIN USART3_IRQn 1 */

    /* USER CODE END USART3_IRQn 1 */
}

/**
 * @brief This function handles TIM8 capture compare interrupt.
 */
void TIM8_CC_IRQHandler(void)
{
    /* USER CODE BEGIN TIM8_CC_IRQn 0 */

    /* USER CODE END TIM8_CC_IRQn 0 */
    HAL_TIM_IRQHandler(&htim8);
    /* USER CODE BEGIN TIM8_CC_IRQn 1 */

    /* USER CODE END TIM8_CC_IRQn 1 */
}

/**
 * @brief This function handles TIM5 global interrupt.
 */
void TIM5_IRQHandler(void)
{
    /* USER CODE BEGIN TIM5_IRQn 0 */

    /* USER CODE END TIM5_IRQn 0 */
    HAL_TIM_IRQHandler(&htim5);
    /* USER CODE BEGIN TIM5_IRQn 1 */

    /* USER CODE END TIM5_IRQn 1 */
}

/**
 * @brief This function handles UART4 global interrupt / UART4 wake-up interrupt through EXTI line 34.
 */
void UART4_IRQHandler(void)
{
    /* USER CODE BEGIN UART4_IRQn 0 */

    /* USER CODE END UART4_IRQn 0 */
    HAL_UART_IRQHandler(&huart4);
    /* USER CODE BEGIN UART4_IRQn 1 */

    /* USER CODE END UART4_IRQn 1 */
}

/**
 * @brief This function handles UART5 global interrupt / UART5 wake-up interrupt through EXTI line 35.
 */
void UART5_IRQHandler(void)
{
    /* USER CODE BEGIN UART5_IRQn 0 */

    /* USER CODE END UART5_IRQn 0 */
    HAL_UART_IRQHandler(&huart5);
    /* USER CODE BEGIN UART5_IRQn 1 */

    /* USER CODE END UART5_IRQn 1 */
}

/**
 * @brief This function handles TIM7 global interrupt, DAC2 and DAC4 channel underrun error interrupts.
 */
void TIM7_DAC_IRQHandler(void)
{
    /* USER CODE BEGIN TIM7_DAC_IRQn 0 */

    /* USER CODE END TIM7_DAC_IRQn 0 */
    HAL_TIM_IRQHandler(&htim7);
    /* USER CODE BEGIN TIM7_DAC_IRQn 1 */

    /* USER CODE END TIM7_DAC_IRQn 1 */
}

/**
 * @brief This function handles I2C3 event interrupt / I2C3 wake-up interrupt through EXTI line 27.
 */
void I2C3_EV_IRQHandler(void)
{
    /* USER CODE BEGIN I2C3_EV_IRQn 0 */

    /* USER CODE END I2C3_EV_IRQn 0 */
    HAL_I2C_EV_IRQHandler(&hi2c3);
    /* USER CODE BEGIN I2C3_EV_IRQn 1 */

    /* USER CODE END I2C3_EV_IRQn 1 */
}

/**
 * @brief This function handles I2C3 error interrupt.
 */
void I2C3_ER_IRQHandler(void)
{
    /* USER CODE BEGIN I2C3_ER_IRQn 0 */

    /* USER CODE END I2C3_ER_IRQn 0 */
    HAL_I2C_ER_IRQHandler(&hi2c3);
    /* USER CODE BEGIN I2C3_ER_IRQn 1 */

    /* USER CODE END I2C3_ER_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        // 减速比30,线数11
        const float cm_per_pulse = 2.0f * PI * 2.97f / (660.0f / 2.0f); // 每脉冲对应的距离（cm）

        volatile float right_pulse = (short)__HAL_TIM_GET_COUNTER(&htim1) / 2.0f;
        volatile float left_pulse = (short)__HAL_TIM_GET_COUNTER(&htim3) / 2.0f;

        right_speed = right_pulse * cm_per_pulse / 0.001f; // cm/s
        left_speed = left_pulse * cm_per_pulse / 0.001f;

        distance += (right_pulse + left_pulse) * cm_per_pulse / 2.0f; // cm

        __HAL_TIM_SET_COUNTER(&htim1, 0);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
    }
    else if (htim->Instance == TIM16)
    {
        // 定时器TIM16用于BSP库状态机
        BSP_Tick(); // 调用BSP状态机函数 每1ms执行一次
    }
    else if (htim->Instance == TIM7)
    {
        // volatile float Anglepid_output = Angle_PIDcaculate(&Angle, AngleData.z_Yaw, GyroData.z, follow_control.angle); // 角度环PID计算

        // volatile float Followpid_output = pid_calculate(&Follow, follow_control.direction, 0); // 位置环 PID计算

        // volatile float Pointpid_output_x = PID_Calculate_DSP_Witherror(&point, x_angle); // 点位环PID计算X轴
        // volatile float Pointpid_output_y = PID_Calculate_DSP_Witherror(&point, y_angle); // 点位环PID计算Y轴

        // // 控制电机速度，0-10000PWM
        // Load(
        //     (int)(follow_control.base_speed + Followpid_output),
        //     (int)(follow_control.base_speed + Followpid_output),
        //     (int)(follow_control.base_speed - Followpid_output),
        //     (int)(follow_control.base_speed - Followpid_output));

        // 只输入转向环
        //  Load(
        //      (int)Anglepid_output + (int)Followpid_output,
        //      (int)Anglepid_output + (int)Followpid_output,
        //      -(int)Anglepid_output - (int)Followpid_output,
        //      -(int)Anglepid_output - (int)Followpid_output);

        // 只输入角度环
        // Load(
        //     (int)Anglepid_output,
        //     (int)Anglepid_output,
        //     -(int)Anglepid_output,
        //     -(int)Anglepid_output);
    }
}

#ifdef _STEP_DIR_CTRL_MODE_
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8) // TIM8用于步进电机
    {
        // 步进电机的PWM脉冲上升沿发生回调
        // 在这里进行脉冲数计数
        StepMotor_Tick(StepMotorA);
    }
    else if (htim->Instance == TIM17) // TIM17用于步进电机
    {
        // 步进电机的PWM脉冲上升沿发生回调
        // 在这里进行脉冲数计数
        StepMotor_Tick(StepMotorB);
    }
}
#endif

#ifdef _CAN_CTRL_MODE_

static FDCAN_RxHeaderTypeDef RxHeader;

/**
 * @brief 该函数用于接收FDCAN1的消息，并将接收到的数据回传。
 * @param hfdcan FDCAN句柄
 * @param RxFifo0ITs 接收FIFO0的中断状态
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs && FDCAN_IT_RX_FIFO0_NEW_MESSAGE != RESET)
    {
        if (hfdcan->Instance == FDCAN1)
        {
            // 处理接收到的数据
            uint8_t rxData[8] = {0};
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, rxData) == HAL_OK)
            {
                // 将数据rxData回传并进行下一步的解析
                FDCAN1_ReceiveMessageFromRxFifo(rxData, &RxHeader);
            }
        }
    }
}

#endif

// 串口中断回调
uint8_t uart_rx_state = 0;   // 0: 未知, 1: 接收角度值, 2: 接收字符指令
uint8_t uart_rx_buffer[2];   // 十六位缓冲区
uint16_t uart_rx_values[10]; // 接收的两个十六位的数据
int16_t uart_rx_value;       // 接收的一个十六位的数据
uint8_t rx_byte;             // 接收缓冲区
static uint8_t value_index = 0;
uint8_t rx_data[20] = {0}; // 串口空闲接收缓冲区

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t index = 0;    // 用来接收16位数据的索引
    static uint8_t cmd_type = 0; // 0: 未知, 1: 角度值, 2: 字符指令

    if (huart->Instance == UART5)
    {
        if (rx_byte == 0xAA)
        {
            uart_rx_state = 1; // 角度值数据
            cmd_type = 1;
            index = 0;
            HAL_UART_Receive_IT(huart, &rx_byte, 1);
            return;
        }
        else if (rx_byte == 0xBB)
        {
            uart_rx_state = 1; // 字符指令
            cmd_type = 2;
            index = 0;
            HAL_UART_Receive_IT(huart, &rx_byte, 1);
            return;
        }
        else if (rx_byte == 0xEE)
        {
            uart_rx_state = 0;
            cmd_type = 0;
            HAL_UART_Receive_IT(huart, &rx_byte, 1);
            return;
        }

        switch (uart_rx_state)
        {
        case 1:
            uart_rx_buffer[0] = rx_byte;
            if (cmd_type == 2)
            {
                // 字符指令
                uart_rx_value = uart_rx_buffer[0];
                if (uart_rx_value == 'S')
                {
                }
                else if (uart_rx_value == 'Z')
                {
                }
                else if (uart_rx_value == 'L')
                {
                }
                else if (uart_rx_value == 'R')
                {
                }
                cmd_type = 0;
            }
            else
            {
                uart_rx_state = 3; // 接收角度状态
            }
            break;
        case 2:
            uart_rx_buffer[1] = rx_byte;
            uart_rx_state = 0;

            uart_rx_value = (uart_rx_buffer[1] << 8) | uart_rx_buffer[0];
            break;
        case 3:
            // 处理其他状态
            // 假设你有一个 uint16_t uart_rx_values[2]; 和一个索引变量
            uart_rx_buffer[1] = rx_byte;
            uart_rx_value = (uart_rx_buffer[1] << 8) | uart_rx_buffer[0];
            uart_rx_values[value_index++] = uart_rx_value;
            if (value_index >= 2)
                value_index = 0; // 只存两个数，循环覆盖
            uart_rx_state = 1;
            break;
        default:
            uart_rx_state = 0;
            cmd_type = 0;
            break;
        }
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
    // 串口4
    else if (huart->Instance == UART4)
    {
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5)
    {
        // 解析数据包
        uint16_t i = 0;
        while (i < Size)
        {
            if (rx_data[i] == 0xBB && (i + 2) <= Size)
            {
                // 字符命令包: 0xBB <cmd> 0xEE
                if (rx_data[i + 2] == 0xEE)
                {
                    uint8_t cmd = rx_data[i + 1];
                    // 处理字符命令
                    // 例如: 'S', 'Z', 'L', 'R'
                    // switch(cmd) { ... }
                    i += 3;
                    continue; // 继续遍历整个缓冲区
                }
            }
            else if (rx_data[i] == 0xAA)
            {
                // 数据指令包: 0xAA <data...> 0xEE
                uint16_t j = i + 1;
                // 查找帧尾
                while (j < Size && rx_data[j] != 0xEE)
                    j++;
                if (j < Size && rx_data[j] == 0xEE)
                {
                    uint16_t data_len = j - (i + 1);
                    uint16_t word_count = (data_len + 1) / 2; // 每两个字节一个16位
                    for (uint16_t k = 0; k < word_count; k++)
                    {
                        uint8_t high = (i + 1 + 2 * k < j) ? rx_data[i + 1 + 2 * k] : 0;
                        uint8_t low = (i + 2 + 2 * k < j) ? rx_data[i + 1 + 2 * k + 1] : 0;
                        uart_rx_values[k] = (high << 8) | low;
                    }
                    break; // 处理完数据包后退出循环
                }
                else
                {
                    // 没有帧尾，丢弃
                    break;
                }
            }
            else
            {
                i++;
            }
        }
        // 清空缓冲区
        memset(rx_data, 0, sizeof(rx_data));
        // 重新启动DMA接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_data, sizeof(rx_data));
        __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }
}

// i2c接收中断
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2)
    {
        AngleData.x_Roll = (int16_t)((int16_t)(JY901_data[19] << 8 | JY901_data[18])) / 32768.0 * 180.0;
        AngleData.y_Pitch = (int16_t)((int16_t)(JY901_data[21] << 8 | JY901_data[20])) / 32768.0 * 180.0;
        AngleData.z_Yaw = (uint16_t)((uint16_t)(JY901_data[23] << 8 | JY901_data[22])) / 32768.0 * 180.0;
        jy901_z_angle_zero(); // 将Z轴角度归零运算
        jy901_dire_read_IT(); // 重新开启I2C接收中断
    }
    // 八路灰度传感器
    else if (hi2c->Instance == I2C3)
    {
    }
}

/* USER CODE END 1 */
