#include "stm32g474xx.h"
#include "ADS1115_REG.h"
#include "ADS1115.h"

#include "stdio.h"
#include "i2c.h"
// Initialize the ADS1115 with default settings
void ADS1115_Init(void)
{
    ADS1115_WriteReg(config_MSB, config_LSB); // Default configuration
}

/**
 * @brief 向 ADS1115 设备写入配置寄存器
 *
 * @param ConfigData_H 高字节配置数据
 * @param ConfigData_L 低字节配置数据
 *
 */
void ADS1115_WriteReg(uint8_t ConfigData_H, uint8_t ConfigData_L)
{
    uint8_t TranData[3] = {REG_config, ConfigData_H, ConfigData_L};
    HAL_I2C_Master_Transmit(&ADS1115_I2C_Handle, ADS1115_W, TranData, 3, 1000);

    uint8_t appliedPGA = (0x0E & ConfigData_H) >> 1; // 提取 PGA 位
    switch (appliedPGA)                              // 根据 PGA 值设置 FS
    {
    case 0x00:
        FS = 6.144; // ±6.144 V (默认)
        break;
    case 0x01:
        FS = 4.096; // ±4.096 V
        break;
    case 0x02:
        FS = 2.048; // ±2.048 V
        break;
    case 0x03:
        FS = 1.024; // ±1.024 V
        break;
    case 0x04:
        FS = 0.512; // ±0.512 V
        break;
    case 0x05:
        FS = 0.256; // ±0.256 V
        break;
    default:
        break;
    }
}

/**
 * @brief 从 ADS1115 设备读取指定寄存器的值
 *
 * @param regADD 寄存器地址
 * @return uint16_t 读取到的寄存器值
 */
uint16_t ADS1115_ReadReg(uint8_t regADD)
{
    uint8_t receiveReg[2] = {0};
    HAL_I2C_Master_Receive(&ADS1115_I2C_Handle, ADS1115_W, &regADD, 1, 1000);

    HAL_I2C_Master_Receive(&ADS1115_I2C_Handle, ADS1115_R, receiveReg, 2, 1000);

    uint16_t regValue = (receiveReg[0] << 8) | receiveReg[1];
    return regValue;
}

/**
 * @brief 从 ADS1115 设备读取指定通道的 ADC 值 (伪多通道模式，单次只能测量一个通道)
 *
 * @param channel 测量通道
 * @note 通道枚举值可以是 AIN0_1, AIN0_3, AIN1_3, AIN2_3, AIN0_GND, AIN1_GND, AIN2_GND, AIN3_GND，STILL保持上次测量通道（速度更快）
 * @return int16_t 读取到的 ADC 值
 */
float ADS1115_ReadADC(enum MeasChannel channel)
{
    if (channel != STILL)
    {

        uint8_t configData_H = (OS << 7) | (channel << 4) | (PGA << 1) | MODE; // 配置寄存器高字节

        ADS1115_WriteReg(configData_H, config_LSB); // 写入配置寄存器
        HAL_Delay(10);                              // 等待配置测量完成
    }
    uint16_t RawAdcValue = ADS1115_ReadReg(REG_Conversion); // 读取转换寄存器

    float adcValue = 0.0f;

    if (RawAdcValue == 0x7FFF)
    {
        adcValue = 10.0f; // 数据已超最大量程
    }
    else if (RawAdcValue == 0x8000)
    {
        adcValue = -10.0f; // 数据已超最小量程
    }
    else
    {
        int16_t signedValue = (int16_t)RawAdcValue;      // 将无符号值转换为有符号值
        adcValue = ((float)signedValue * FS) / 32767.0f; // 将 ADC 值转换为实际电压值
    }

    return adcValue; // 返回转换后的电压值
}
