/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    fdcan.c
 * @brief   This file provides code for the configuration
 *          of the FDCAN instances.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */

#include "StepMotor.h"

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV2;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 17;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 7;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 17;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 7;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.StdFiltersNbr = 28;
  hfdcan1.Init.ExtFiltersNbr = 8;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

#ifdef _CAN_CTRL_MODE_

  FDCAN1_Config(); // 配置FDCAN1，包括过滤器设置和消息发送头的初始化

#endif /* _CAN_CTRL_MODE_ */

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = ADVANCE_CAN_RX_Pin|ADVANCE_CAN_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, ADVANCE_CAN_RX_Pin|ADVANCE_CAN_TX_Pin);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#ifdef _CAN_CTRL_MODE_
static FDCAN_TxHeaderTypeDef TxHeader;

/**
 * @brief 该函数进行了FDCAN1的配置，包括过滤器设置和消息发送头的初始化，并且启用了FDCAN1.
 * @note 函数名称虽然为FDCAN1_Config，但实际上其中进行了了FDCAN的开启
 */
void FDCAN1_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig = {0};

  /* Configure the filter to accept all messages (标准帧格式) */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;                        // Filter index总共有0-27个标准ID过滤器
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;         // Use mask filter type
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // Use FIFO 0 for received messages
  sFilterConfig.FilterID1 = 0x00000000;                 // Accept all standard IDs
  sFilterConfig.FilterID2 = 0x00000000;                 // Accept all standard IDs
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /*Configure the filter to accept all messages (扩展帧格式) */
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 1;                        // Filter index总共有0-7个扩展ID过滤器
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;         // Use mask filter type
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // Use FIFO 0 for received messages
  sFilterConfig.FilterID1 = 0x00000000;                 // Accept all extended IDs
  sFilterConfig.FilterID2 = 0x00000000;                 // Accept all extended IDs
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /*配置过滤所有遥控帧（标准 & 扩展帧格式） 拒绝所有不符合过滤器条件的邮件（标准 & 扩展帧格式）*/
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate notifications for Rx FIFO 0 new message */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* TxHeader Config 采用扩展帧格式*/
  TxHeader.Identifier = 0x474;                      // CAN ID
  TxHeader.IdType = FDCAN_EXTENDED_ID;              // 使用扩展ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;          // 数据帧
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;          // 数据长度为8字节
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 错误状态指示器---在CAN 2.0协议中被忽略
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           // 禁用比特率切换---在CAN 2.0协议中被忽略
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            // 使用经典CAN格式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 不使用
  TxHeader.MessageMarker = 0;                       // 消息标记---在CAN 2.0协议中被忽略

  /*Start FDCAN*/
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief 向FDCAN1的发送FIFO添加消息
 * @param txData 要发送的数据 默认宽度为8字节
 * @note 注意：此函数仅将Data放入发送FIFO，发送操作由MCU自动完成
 */
void FDCAN1_AddMessageToTxFifo(uint8_t txData[], uint32_t CanID, uint32_t DataLength)
{
  TxHeader.Identifier = CanID;      // 设置CAN ID
  TxHeader.DataLength = DataLength; // 设置数据长度
  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0)
  {
    // 等待发送FIFO有空余空间
  }
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txData) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief 从FDCAN1的接收FIFO接收消息
 * @param rxData 接收的数据缓冲区
 * @param rxHeader 接收消息的头部信息
 * @note 注意：此函数将由接收数据callback函数调用 将fifo中的数据读取并继续回传至请求消息的函数
 */
void FDCAN1_ReceiveMessageFromRxFifo(uint8_t rxData[], FDCAN_RxHeaderTypeDef *rxHeader)
{
  StepMotor_ReturnMsgCallback(rxData, rxHeader->Identifier, rxHeader->DataLength);
}
#endif /* _CAN_CTRL_MODE_ */

/* USER CODE END 1 */
