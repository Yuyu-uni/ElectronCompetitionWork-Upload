#include "stm32g474xx.h"

#include "Laser-L1.h"
#include "Laser-L1_DMA.h"

uint8_t CommandBuffer[CommandBufferSize] = {0};
uint8_t ResponseBuffer[ResponseBufferSize] = {0};
uint8_t ResponseFuncCode;
uint8_t Laser_Responde_Data[4] = {0};
/*
 * @brief  激光传感器初始化
 * @param  MeasureMode
 * @param  测试模式OnceMeasureMode（单次测量）
 * @param  ConstantMeasureMode（连续精确测量8Hz）
 * @param  FastMeasareMode（快速测量20Hz）
 * @note   This function is used to initialize the laser sensor.
 */
void Laser_Init(enum MeasureMode mode)
{ // 启动DMA 发送启动指令

    // 设定指令
    CommandBuffer[0] = 0xA5;
    CommandBuffer[1] = 0x5A;
    CommandBuffer[2] = mode;
    CommandBuffer[3] = 0x00;
    CommandBuffer[4] = (uint8_t)(CommandBuffer[0] ^ CommandBuffer[1] ^ CommandBuffer[2] ^ CommandBuffer[3]);
    ResponseFuncCode = mode + 0x80; // 设置响应函数码
    // 传输指令
    Laser_TransmitDMA();

    // 开启DMA接收
    Laser_ReceiveDMA_Start();
}
/*
 * @brief  获取激光传感器的距离
 * @note   This function is used to get the distance(mm) from the laser sensor.
 * @retval The distance value（uint32）.
 */
uint32_t Laser_GetDistance(void)
{
    uint32_t distance = Laser_Responde_Data[0] << 24 | Laser_Responde_Data[1] << 16 | Laser_Responde_Data[2] << 8 | Laser_Responde_Data[3];
    return distance;
}
