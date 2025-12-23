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
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define RB_PWM_Pin GPIO_PIN_2
#define RB_PWM_GPIO_Port GPIOE
#define RF_PWM_Pin GPIO_PIN_3
#define RF_PWM_GPIO_Port GPIOE
#define RB_IN_Pin GPIO_PIN_4
#define RB_IN_GPIO_Port GPIOE
#define RF_IN_Pin GPIO_PIN_5
#define RF_IN_GPIO_Port GPIOE
#define ULED2_Core_Pin GPIO_PIN_13
#define ULED2_Core_GPIO_Port GPIOC
#define LF_PWM_Pin GPIO_PIN_3
#define LF_PWM_GPIO_Port GPIOF
#define B_Raw_PWM_Pin GPIO_PIN_9
#define B_Raw_PWM_GPIO_Port GPIOF
#define A_Raw_PWM_Pin GPIO_PIN_10
#define A_Raw_PWM_GPIO_Port GPIOF
#define RB_ENC_A_Pin GPIO_PIN_0
#define RB_ENC_A_GPIO_Port GPIOC
#define RB_ENC_B_Pin GPIO_PIN_1
#define RB_ENC_B_GPIO_Port GPIOC
#define LB_IN_Pin GPIO_PIN_2
#define LB_IN_GPIO_Port GPIOC
#define LF_IN_Pin GPIO_PIN_3
#define LF_IN_GPIO_Port GPIOC
#define LB_PWM_Pin GPIO_PIN_2
#define LB_PWM_GPIO_Port GPIOF
#define RF_ENC_A_Pin GPIO_PIN_0
#define RF_ENC_A_GPIO_Port GPIOA
#define RF_ENC_B_Pin GPIO_PIN_1
#define RF_ENC_B_GPIO_Port GPIOA
#define USW2_Core_Pin GPIO_PIN_2
#define USW2_Core_GPIO_Port GPIOA
#define FLASH_SPI1_NSS_Pin GPIO_PIN_4
#define FLASH_SPI1_NSS_GPIO_Port GPIOA
#define FLASH_SPI1_SCK_Pin GPIO_PIN_5
#define FLASH_SPI1_SCK_GPIO_Port GPIOA
#define FLASH_SPI1_MISO_Pin GPIO_PIN_6
#define FLASH_SPI1_MISO_GPIO_Port GPIOA
#define FLASH_SPI1_MOSI_Pin GPIO_PIN_7
#define FLASH_SPI1_MOSI_GPIO_Port GPIOA
#define Servo_SIG_TX_Pin GPIO_PIN_10
#define Servo_SIG_TX_GPIO_Port GPIOB
#define USW1_Core_Pin GPIO_PIN_11
#define USW1_Core_GPIO_Port GPIOB
#define DRIVER_SPI2_SCK_Pin GPIO_PIN_13
#define DRIVER_SPI2_SCK_GPIO_Port GPIOB
#define DRIVER_SPI2_MISO_Pin GPIO_PIN_14
#define DRIVER_SPI2_MISO_GPIO_Port GPIOB
#define DRIVER_SPI2_MOSI_Pin GPIO_PIN_15
#define DRIVER_SPI2_MOSI_GPIO_Port GPIOB
#define LF_ENC_A_Pin GPIO_PIN_12
#define LF_ENC_A_GPIO_Port GPIOD
#define LF_ENC_B_Pin GPIO_PIN_13
#define LF_ENC_B_GPIO_Port GPIOD
#define LB_ENC_A_Pin GPIO_PIN_6
#define LB_ENC_A_GPIO_Port GPIOC
#define LB_ENC_B_Pin GPIO_PIN_7
#define LB_ENC_B_GPIO_Port GPIOC
#define ADVANCE_ALERT_Pin GPIO_PIN_2
#define ADVANCE_ALERT_GPIO_Port GPIOG
#define ADVANCE_I2C3_SCL_Pin GPIO_PIN_8
#define ADVANCE_I2C3_SCL_GPIO_Port GPIOC
#define ADVANCE_I2C3_SDA_Pin GPIO_PIN_9
#define ADVANCE_I2C3_SDA_GPIO_Port GPIOC
#define DRIVER_I2C2_SDA_Pin GPIO_PIN_8
#define DRIVER_I2C2_SDA_GPIO_Port GPIOA
#define DRIVER_I2C2_SCL_Pin GPIO_PIN_9
#define DRIVER_I2C2_SCL_GPIO_Port GPIOA
#define DebugUART4_TX_Pin GPIO_PIN_10
#define DebugUART4_TX_GPIO_Port GPIOC
#define DebugUART4_RX_Pin GPIO_PIN_11
#define DebugUART4_RX_GPIO_Port GPIOC
#define USW3_Driver_Pin GPIO_PIN_5
#define USW3_Driver_GPIO_Port GPIOG
#define USW4_Driver_Pin GPIO_PIN_6
#define USW4_Driver_GPIO_Port GPIOG
#define ADVANCE_CAN_RX_Pin GPIO_PIN_0
#define ADVANCE_CAN_RX_GPIO_Port GPIOD
#define ADVANCE_CAN_TX_Pin GPIO_PIN_1
#define ADVANCE_CAN_TX_GPIO_Port GPIOD
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOD
#define ULED4_Driver_Pin GPIO_PIN_4
#define ULED4_Driver_GPIO_Port GPIOB
#define StepA_DIR_Pin GPIO_PIN_5
#define StepA_DIR_GPIO_Port GPIOB
#define StepA_STEP_Pin GPIO_PIN_6
#define StepA_STEP_GPIO_Port GPIOB
#define StepB_DIR_Pin GPIO_PIN_7
#define StepB_DIR_GPIO_Port GPIOB
#define StepB_STEP_Pin GPIO_PIN_9
#define StepB_STEP_GPIO_Port GPIOB
#define ULED3_Driver_Pin GPIO_PIN_0
#define ULED3_Driver_GPIO_Port GPIOE
#define ULED1_Core_Pin GPIO_PIN_1
#define ULED1_Core_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
