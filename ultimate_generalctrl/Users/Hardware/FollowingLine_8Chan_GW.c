#include "stm32g474xx.h"
#include "i2c.h"
#include "stdio.h"
#include "FollowingLine_8Chan_GW.h"

uint8_t Follow_Data; // 用于中断，存储感为八路灰度传感器的数字数据
int8_t Follow_value; // 用于存储感为八路灰度传感器的加权输出
int Follow_num;      // 用于存储感为八路灰度传感器的通道数
/**
 * @brief 该函数进行感为八路灰度传感器的初始化。
 *
 *
 * @note
 * 此函数对传感器进行Ping测试，直至传感器返回0x66代表传感器准备好
 * @note
 * 在主控与传感器同时上电时，该函数可能会产生一定阻塞
 */
void FollowingLine_8Chan_GW_Init(void)
{
    uint8_t pingTestData = 0x00;
    uint8_t pPingTestREG = PingTestREG; // Ping测试寄存器地址
    while (pingTestData != 0x66)
    {
        HAL_I2C_Master_Transmit(&FOLLOWINGLINE_8CHAN_GW_I2C_Handle, FOLLOWINGLINEADDR, &pPingTestREG, 1, 1000);
        HAL_I2C_Master_Receive(&FOLLOWINGLINE_8CHAN_GW_I2C_Handle, FOLLOWINGLINEADDR, &pingTestData, 1, 1000);
    }
    uint8_t pDigitalDataREG = DigitalDataREG; // 数字数据寄存器地址
    HAL_I2C_Master_Transmit(&FOLLOWINGLINE_8CHAN_GW_I2C_Handle, FOLLOWINGLINEADDR, &pDigitalDataREG, 1, 1000);
}
/**
 * @brief 该函数从感为八路灰度传感器获取数据。
 * @return uint8_t 返回传感器的数字数据 该值是一个8位的二进制数，从低位到高位分别对应1-8路的通道结果，1表示白色，0表示黑色。
 * @note 该函数仅用于读取数字量结果，无法读取模拟量结果。
 *       如果需要读取模拟量结果，请使用其他函数。
 */
uint8_t FollowingLine_8Chan_GW_Read(void)
{
    uint8_t digitalData = 0x00;
    HAL_I2C_Master_Receive(&FOLLOWINGLINE_8CHAN_GW_I2C_Handle, FOLLOWINGLINEADDR, &digitalData, 1, 1000);

    return digitalData;
}
/**
 * @brief 该函数向感为八路灰度传感器写入数据。
 * @param reg 寄存器地址
 * @param data 数据指针
 * @param len 数据长度
 * @note 该函数用于向传感器写入数据，用于配置传感器的工作模式或参数。
 * @note 该函数仅将向感为传感器发送指令封装起来，无特殊功能，一般用不到
 */
void FollowingLine_8Chan_GW_Write_Common(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Master_Transmit(&FOLLOWINGLINE_8CHAN_GW_I2C_Handle, FOLLOWINGLINEADDR, &reg, len, 1000);
}

/**
 * @brief 该函数为解析感为八路灰度传感器的寄存器数据。
 * @param Follow_Data 感为八路灰度传感器的寄存器数据
 * @return output 返回解析后的数据
 */
uint8_t FollowingLine_Analysis(uint8_t Follow_Data)
{
    Follow_num = 0;                                          // 初始化通道数
    static int8_t weights[8] = {7, 5, 3, 1, -1, -3, -5, -7}; // 左到右权值
    int8_t output = 0;
    for (int i = 0; i < 8; i++)
    {
        if ((Follow_Data >> i) & 0x01)
        {
            output += weights[i];
        }
        Follow_num++; // 统计通道数
    }
    return output; // 返回加权和
}
