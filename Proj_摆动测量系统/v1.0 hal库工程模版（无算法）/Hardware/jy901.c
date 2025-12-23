#include "jy901.h"
#include "JY901S_REG.h"
#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "string.h"
#include "stdio.h"
extern I2C_HandleTypeDef hi2c3;

// 所有传感器数据
// 角度
struct
{
    double x_Roll;
    double y_Pitch;
    double z_Yaw;
} AngleData;

// 加速度
struct
{
    double x;
    double y;
    double z;
} AccelData;

// 角速度
struct
{
    double x;
    double y;
    double z;
} GyroData;

// 磁力计
struct
{
    double x;
    double y;
    double z;
} MagData;

// 气压计
int32_t pressureData;
// 高度值
int32_t heightData;

/*
 * @brief  向 JY901S 总线发送数据（不带寄存器地址）
 * @param  data: 指向要发送数据的指针
 * @param  length: 要发送的数据长度
 */
void jy901_writeMASTER(uint8_t *data, uint16_t length)
{
    HAL_I2C_Master_Transmit(&hi2c3, JY901S_ADDRESS, data, length, 100);
}

/*
 * @brief 向指定寄存器写入数据\指令
 * @param reg: 寄存器地址
 * @param data: 要写入的数据\指令
 * @param length: 要写入的数据长度\指令长度
 * @param delayBeforeWrite: 写入前延时(<=0表示不延时)
 * @param delayAfterWrite: 写入后延时(<=0表示不延时)
 * @note 该函数向指定寄存器写入数据，长度为length。其中包含解锁指令
 */
void jy901_writeREG(uint8_t reg, uint8_t *data, uint16_t length, uint8_t delayBeforeWrite, uint8_t delayAfterWrite)
{
    uint8_t unlockData[2] = {0x88, 0xB5};                                                         // 解锁数据
    uint8_t saveData[2] = {0x00, 0x00};                                                           // 保存数据
    HAL_I2C_Mem_Write(&hi2c3, JY901S_ADDRESS, KEY_REG, I2C_MEMADD_SIZE_8BIT, unlockData, 2, 100); // 解锁JY901 准备读数据
    if (delayBeforeWrite > 0)
    {
        HAL_Delay(delayBeforeWrite); // 延时
    }
    // 传输个人寄存器值start
    HAL_I2C_Mem_Write(&hi2c3, JY901S_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length, 100); // 向指定寄存器写入数据
    // 传输个人寄存器值end
    if (delayAfterWrite > 0)
    {
        HAL_Delay(delayAfterWrite); // 延时
    }
    HAL_I2C_Mem_Write(&hi2c3, JY901S_ADDRESS, SAVE, I2C_MEMADD_SIZE_8BIT, saveData, 2, 100); // 解锁JY901 准备读数据
}

/*
 * IIC 写入的时序数据格式如下
IICAddr<<1 RegAddr (IICAddr<<1)|1 Data1L Data1H Data2L Data2H ……
首先 IIC 主机向 JY-901L 模块发送一个 Start 信号，在将模块的 IIC 地址 IICAddr 写
入，在写入寄存器地址 RegAddr，主机再向模块发送一个读信号(IICAddr<<1)|1，如果是
默认地址 0x51，那么发送的数据为 0xa1，此后模块将按照先低字节，后高字节的顺序输
出数据，主机需在收到每一个字节后，拉低 SDA 总线，向模块发出一个应答信号，待
接收完指定数量的数据以后，主机不再向模块回馈应答信号，此后模块将不再输出数据，
主机向模块再发送一个停止信号，以结束本次操作。
以读出模块的角度数据为例，RedAddr 为 0x3d、0x3e、0x3f，连续读取 6 个字节
(一个寄存器两个字节，先低后高)
 */

/*
 * @brief  Read data from JY901S
 * @param  reg: Register address to read from
 * @param  data: Pointer to the buffer to store the read data
 * @param  length: Number of bytes to read
 * @retval None
 * @note   This function reads data from the specified register of the JY901S sensor.
 */
void jy901_readREG(uint8_t reg, uint8_t *data, uint16_t length)
{
    // HAL_I2C_Master_Transmit(&2hi2c2, JY901S_ADDRESS, &reg, 1, 100);
    // HAL_I2C_Master_Receive(&2hi2c2, JY901S_ADDRESS, data, length, 100);
    HAL_I2C_Mem_Read(&hi2c3, JY901S_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length, 100); // 向i2c指定从设备的某一存储地址读出一个数，jy910s读完这个寄存器后指针会自增，可以连续读
    return;
}

/*角度参考
角度参考是以传感器当前的实际位置，让xy轴的角度归零，做一个相对归零操作。
指令操作流程：
1.解锁：FF AA 69 88 B5
1.1延时200ms
2.校准：FF AA 01 08 00
2.1延时3秒
3.保存: FF AA 00 00 00
*/
/*
 * @brief  xy轴角度归零
 * @note   该函数将传感器的当前角度作为参考点，将XY轴的角度归零
 * @note   该函数涉及3.2s的延时，可能会影响运行效率
 */
void jy901_zeroXY(void)
{
    uint8_t data[2] = {0x08, 0x00};
    jy901_writeREG(CALSW, data, 2, 200, 3000);
}

/*
/**
 * @brief 读取方向相关传感器的值（double类型）
 *
 * @note 读取数据已写入AccelData（加速度）、GyroData（角速度）、MagData（磁力计）和AngleData（角度）结构体中
 * @note AccelData 单位为 m/s^2
 * @note GyroData 单位为 °/s
 * @note MagData 单位为 uT（微特斯拉）
 * @note AngleData 单位为 °
 *
 */
void jy901_dire_read(void)
{
    uint8_t data[24];
    jy901_readREG(AX, data, 24); // 读取数据
    // 加速度

    AccelData.x = (double)((int16_t)(data[1] << 8 | data[0])) / 32768.0 * 16.0 * GRAVITY;
    AccelData.y = (double)((int16_t)(data[3] << 8 | data[2])) / 32768.0 * 16.0 * GRAVITY;
    AccelData.z = (double)((int16_t)(data[5] << 8 | data[4])) / 32768.0 * 16.0 * GRAVITY;
    // 角速度
    GyroData.x = (double)((int16_t)(data[7] << 8 | data[6])) / 32768.0 * 2000.0;
    GyroData.y = (double)((int16_t)(data[9] << 8 | data[8])) / 32768.0 * 2000.0;
    GyroData.z = (double)((int16_t)(data[11] << 8 | data[10])) / 32768.0 * 2000.0;
    // 磁力计
    MagData.x = (double)((int16_t)(data[13] << 8 | data[12]));
    MagData.y = (double)((int16_t)(data[15] << 8 | data[14]));
    MagData.z = (double)((int16_t)(data[17] << 8 | data[16]));
    // 角度
    AngleData.x_Roll = (double)((int16_t)(data[19] << 8 | data[18])) / 32768.0 * 180.0;
    AngleData.y_Pitch = (double)((int16_t)(data[21] << 8 | data[20])) / 32768.0 * 180.0;
    AngleData.z_Yaw = (double)((int16_t)(data[23] << 8 | data[22])) / 32768.0 * 180.0;
}
// 读取高度相关传感器的值
/*
 * @brief 读取高度相关传感器的值（int32类型）
 *
 * @note 读取数据已写入pressureData(气压)、heightData(高度计)中
 * @note pressureData 单位为 Pa
 * @note heightData 单位为 cm
 *
 */
void jy901_height_read(void)
{
    uint8_t data[8];
    jy901_readREG(PressureL, data, 8);
    pressureData = (int32_t)((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);
    heightData = (int32_t)((data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4]);
}
