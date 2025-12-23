/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

    /* Private includes ----------------------------------------------------------*/
    /* USER CODE BEGIN Includes */

    /* USER CODE END Includes */

    /* Exported types ------------------------------------------------------------*/
    /* USER CODE BEGIN ET */
    extern struct
    {
        double x_Roll;
        double y_Pitch;
        double z_Yaw;
    } AngleData;
    extern struct
    {
        double x;
        double y;
        double z;
    } AccelData;
    extern struct
    {
        double x;
        double y;
        double z;
    } GyroData;

    extern int32_t pressureData; // 气压计
    extern int32_t heightData;   // 高度值

    /* USER CODE END ET */

    /* Exported constants --------------------------------------------------------*/
    /* USER CODE BEGIN EC */

    /* USER CODE END EC */

    /* Exported macro ------------------------------------------------------------*/
    /* USER CODE BEGIN EM */

    /* USER CODE END EM */

    /* Exported functions prototypes ---------------------------------------------*/
    void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UserLED2_OnTop_Pin GPIO_PIN_13
#define UserLED2_OnTop_GPIO_Port GPIOC
#define ECHO_Pin GPIO_PIN_6
#define ECHO_GPIO_Port GPIOC
#define ECHO_EXTI_IRQn EXTI9_5_IRQn
#define TRIG_Pin GPIO_PIN_7
#define TRIG_GPIO_Port GPIOC
#define UserLED1_OnTop_Pin GPIO_PIN_1
#define UserLED1_OnTop_GPIO_Port GPIOE

    /* USER CODE BEGIN Private defines */

    /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
