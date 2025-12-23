/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.h
 * @brief   This file contains all the function prototypes for
 *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

    /* USER CODE BEGIN Includes */

    /* USER CODE END Includes */

    extern UART_HandleTypeDef huart4;

    extern UART_HandleTypeDef huart5;

    extern UART_HandleTypeDef huart1;

    extern UART_HandleTypeDef huart2;

    extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
#define BUFFER_SIZE 256 // 定义环形缓冲区大小

    /* USER CODE END Private defines */

    void MX_UART4_Init(void);
    void MX_UART5_Init(void);
    void MX_USART1_UART_Init(void);
    void MX_USART2_UART_Init(void);
    void MX_USART3_UART_Init(void);

    /* USER CODE BEGIN Prototypes */
    typedef struct
    {
        uint8_t buffer[BUFFER_SIZE]; // 环形缓冲区
        uint16_t readIndex;          // 读索引
        uint16_t writeIndex;         // 写索引
    } RingBuffer;

    extern RingBuffer RingBuffer1; // 定义环形缓冲区

    void Uart_printf(UART_HandleTypeDef *huart, char *format, ...);

    uint8_t Command_Write(uint8_t *data, uint8_t length); // 写入数据到环形缓冲区
    uint8_t Command_GetCommand(uint8_t *command);         // 获取一条完整的指令
    void Command_Send(uint8_t *data, uint8_t length);

    extern RingBuffer RingBuffer1; // 定义环形缓冲区1
    /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
