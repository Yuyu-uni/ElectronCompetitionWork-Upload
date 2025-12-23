#include "stm32g474xx.h"
#include "string.h"

#include "usart.h"
#include "dma.h"
#include "Laser-L1_DMA.h"

#define Laser_UART_Handler huart4 // 串口传输句柄

extern uint8_t CommandBuffer[CommandBufferSize];   // define the command buffer size
extern uint8_t ResponseBuffer[ResponseBufferSize]; // define the response buffer size
extern uint8_t ResponseFuncCode;
extern uint8_t Laser_Responde_Data[4];

void Laser_ReceiveDMA_Start(void)
{
    HAL_UART_Receive_DMA(&Laser_UART_Handler, ResponseBuffer, ResponseBufferSize); // start the DMA reception
}
void Laser_TransmitDMA(void)
{
    HAL_UART_Transmit_DMA(&Laser_UART_Handler, CommandBuffer, CommandBufferSize); // start the DMA transmission
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &Laser_UART_Handler)
    {
        HAL_UART_DMAPause(&Laser_UART_Handler); // pause the DMA reception
        if (ResponseBuffer[0] == 0xB4 && ResponseBuffer[1] == 0x69 && ResponseBuffer[2] == ResponseFuncCode)
        {
            memcpy(Laser_Responde_Data, &ResponseBuffer[3], 4); // copy the response data to the Laser_Responde_Data//将三后面四个字节的数据拷贝到Laser_Responde_Data
        }
        HAL_UART_DMAResume(&Laser_UART_Handler); // resume（复活） the DMA reception
    }
}
