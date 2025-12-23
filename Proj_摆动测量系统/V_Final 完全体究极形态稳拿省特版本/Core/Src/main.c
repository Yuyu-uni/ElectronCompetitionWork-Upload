/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "jy901.h"
#include "delay.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "JY901S_REG.h"
#include "math.h"
#include "stdlib.h"
#include "math_tool.h"
#include "Laser-L1.h"
#include "Laser-L1_DMA.h"
#include "arm_math.h"
#include "arm_const_structs.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern int distance; // 距离mm
double lenth;
double period;                    // 计算单摆周期
double angle;                     // 计算倾角
#define COUNT_THRESHOLD 30        // 易拉罐计数-波谷计数阈值（计数突变点）
extern int distance;              // 距离mm
uint8_t count = 0;                // 易拉罐计数
uint32_t Laser_Distance = 0;      // 易拉罐到板子距离(实时更新)
uint32_t Last_Laser_Distance = 0; // 上次激光测距值
float L_distance = 0;             // 激光测距值
float H_distance = 0;             // 激光测距值
double angle_sum;
int i;
int command; // 蓝牙接收命令

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C3_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();
    MX_TIM4_Init();
    MX_TIM15_Init();
    MX_UART4_Init();
    MX_RTC_Init();
    /* USER CODE BEGIN 2 */
    int last_GyroData = 0;
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&command, 1); // 蓝牙接收命令
    Uart_printf(&huart2, "开机启动\r\n");
    Laser_Init(FastMeasareMode);                             // 激光测距初始化
    jy901_zeroXY();                                          // 零点校准----------
    volatile float distance_beginning = Laser_GetDistance(); // 获取激光初始时刻测距值
    volatile float minimum_distance;                         // 激光测距最小值(初始化为1000，用于比较)
    float constant_L_distance = 0;                           // 首次测量的L_distance

    delay_us(1000); // 延时1ms
    // 148.61mm

    volatile float B = 0; // 激光测距角度
                          /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // HAL_UART_Receive_IT(&huart2, (uint8_t *)&command, 1); // 蓝牙接收命令

        // 激光测距
        Last_Laser_Distance = Laser_Distance; // 保存上次激光测距值
        Laser_Distance = Laser_GetDistance(); // 获取激光测距值
        if (Laser_Distance < minimum_distance)
        {
            minimum_distance = Laser_Distance; // 更新最小值
        }
        if (Last_Laser_Distance - COUNT_THRESHOLD > Laser_Distance)
        {
            count++; // 易拉罐计数
        }

        jy901_dire_read();
        // y轴角度为0附近判断为最低点
        /*-----------------------------------------------------------------------------------------*/
        if (AngleData.x_Roll < 5 && AngleData.x_Roll > -5)
        {
            // 点亮LED
            HAL_GPIO_WritePin(UserLED1_OnTop_GPIO_Port, UserLED1_OnTop_Pin, GPIO_PIN_SET); // 点亮LED

            delay_us(10);                                                  // 延迟消抖
            L_distance = Laser_Distance + lenth + 148.61;                  // 获取激光测距值
            constant_L_distance = distance_beginning + lenth + 148.61;     // 获取激光测距值
                                                                           /*distance更改为distance_beginning，使之为定值*/
            lenth = (723 - 148.61 - 12.9 + 10 - Laser_Distance) / 1000.0f; // 单摆长度
            // 723mm为激光测距到激光发射器的距离
            // 148.61mm为罐子底部到激光发射器的距离
            // 12.9mm为底板
            if (lenth < 0)
            {
                lenth = 0;
            }
            period = calculate_T(lenth) + 0.1; // 计算单摆周期

            // angle = calculate_a_planB(L_distance); // 计算倾角
            // angle = degree(angle) + 1;             // 弧度转角度
            //                                        // 发送角度
            // // 滤除小于10和大于20的角度
            // if (angle > 10 && angle < 20)
            // {

            //     angle_sum += angle; // 累加角度
            //     i++;
            //     if (i > 3)
            //     {
            //         angle = angle_sum / 4; // 平均角度
            //         i = 0;
            //         Uart_printf(&huart2, "Angle:%f\r\n", angle);
            //         angle_sum = 0;   // 清零
            //         angle = 0;       // 清零
            //         HAL_Delay(1000); // 延时100ms
            //     }
            // }

            // 发送绳长
            // Uart_printf(&huart2, "Lenth:%f\r\n", lenth);
            delay_us(50); // 延迟消抖
        }
        else
        {
            // 关闭LED
            HAL_GPIO_WritePin(UserLED1_OnTop_GPIO_Port, UserLED1_OnTop_Pin, GPIO_PIN_RESET); // 关闭LED
        }
        /*----------------------------------------------------------------------------------------------*/

        // y轴角速度正负变换判断为到最高点
        if (GyroData.x > 0 && last_GyroData < 0)
        {
            // 点亮LED
            if (AngleData.x_Roll < 0)
            {
                H_distance = Laser_Distance + lenth + 148.61;
                B = AngleData.x_Roll; // 激光测距角度是负的
            } // 获取激光测距值
            HAL_GPIO_WritePin(UserLED2_OnTop_GPIO_Port, UserLED2_OnTop_Pin, GPIO_PIN_SET); // 点亮LED
            delay_us(10);                                                                  // 延迟消抖
            // Uart_printf(&huart2, "Period:%f\r\n", period);
            // Uart_printf(&huart2, "Angle:%f\r\n", angle);
        }
        else if (GyroData.x < 0 && last_GyroData > 0)
        {
            // 点亮LED
            HAL_GPIO_WritePin(UserLED2_OnTop_GPIO_Port, UserLED2_OnTop_Pin, GPIO_PIN_SET); // 点亮LED
            delay_us(10);                                                                  // 延迟消抖
            // Uart_printf(&huart2, "Period:%f\r\n", period);
            // Uart_printf(&huart2, "Angle:%f\r\n", angle);
            angle = acos(minimum_distance / distance_beginning); // 计算角度
            angle = degree(angle);                               // 弧度转角度
            // 发送角度
            // Uart_printf(&huart2, "Angle:%f\r\n", angle);
            minimum_distance = 1000;
        }
        else
        {
            // 关闭LED
            HAL_GPIO_WritePin(UserLED2_OnTop_GPIO_Port, UserLED2_OnTop_Pin, GPIO_PIN_RESET); // 关闭LED
        }
        last_GyroData = GyroData.x;
        HAL_Delay(150);
        /*-------------------------------------------------------------------------------------------------*/
        // // 发送角度
        // angle = LIMIT(angle, 10.5, 19.5); // 限制角度范围
        // angle_sum += angle;               // 累加角度
        // i++;

        // if (i > 10)
        // {
        //     angle = angle_sum / 10; // 平均角度
        //     i = 0;
        //     Uart_printf(&huart2, "Angle:%f\r\n", angle);
        //     angle_sum = 0; // 清零
        //     angle = 0;     // 清零
        // }

        /*-----------------------------------------------------------------------------------------------*/

        // if (GyroData.z < 10 && GyroData.z > -10)
        // {
        //     // 点亮LED
        //     HAL_GPIO_WritePin(UserLED2_OnTop_GPIO_Port, UserLED2_OnTop_Pin, SET); // 点亮LED
        //     Uart_printf(&huart2, "HIGH\r\n");
        //     HAL_Delay(10); // 延迟消抖
        // }
        // else
        // {
        //     // 关闭LED
        //     HAL_GPIO_WritePin(UserLED2_OnTop_GPIO_Port, UserLED2_OnTop_Pin, RESET); // 关闭LED
        //     Uart_printf(&huart2, "Gyr:%f\r\n", GyroData.z);
        // }

        // y轴角速度正负变换判断为到最高点

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /** Configure LSE Drive Capability
     */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
