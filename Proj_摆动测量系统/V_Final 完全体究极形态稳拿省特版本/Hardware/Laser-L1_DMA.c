#include "stm32g474xx.h"
#include "string.h"

#include "usart.h"
#include "dma.h"
#include "Laser-L1_DMA.h"

#define Laser_UART_Handler huart4 // 串口传输句柄

extern int command; // 激光传感器命令
extern double lenth;
extern double period; // 计算单摆周期
extern double angle;  // 计算倾角
extern uint8_t count; // 易拉罐计数
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
        // HAL_UART_DMAPause(&Laser_UART_Handler); // pause the DMA reception
        for (uint8_t i = 0; i < ResponseBufferSize; i++)
        {
            if (ResponseBuffer[i] == 0xB4)
            {
                if (ResponseBuffer[i + 1] == 0x69 && ResponseBuffer[i + 2] == ResponseFuncCode)
                {
                    memcpy(Laser_Responde_Data, &ResponseBuffer[3], 4); // copy the response data to the Laser_Responde_Data//将三后面四个字节的数据拷贝到Laser_Responde_Data
                }
                // HAL_UART_DMAResume(&Laser_UART_Handler); // resume（复活） the DMA reception
                break;
            }
        }
    }
    if (huart == &huart2) // 蓝牙接收命令
    {
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&command, 1); // 继续接收下一个命令
        // 接收到A发送绳长
        if (command == 'A')
        {
            // 发送绳长
            Uart_printf(&huart2, "Lenth:%f\r\n", lenth);
        }
        else if (command == 'B')
        {
            // 发送周期
            Uart_printf(&huart2, "Period:%f\r\n", period);
        }
        else if (command == 'C')
        {
            // 发送角度
            Uart_printf(&huart2, "Angle:%f\r\n", angle);
        }
        else if (command == 'E')
        {
            // 发送计数
            Uart_printf(&huart2, "Count:%d\r\n", count);
        }
    }
}
