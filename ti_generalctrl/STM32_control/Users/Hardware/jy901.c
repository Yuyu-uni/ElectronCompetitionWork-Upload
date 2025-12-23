#include "jy901.h"
#include "JY901S_REG.h"
#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "string.h"
#include "stdio.h"
extern I2C_HandleTypeDef hi2c2;
uint8_t JY901_data[24];

struct AccelData AccelData; // 加速度
struct GyroData GyroData;   // 角速度
struct MagData MagData;     // 磁力计
struct AngleData AngleData; // 角度

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
    HAL_I2C_Master_Transmit(&hi2c2, JY901S_ADDRESS, data, length, 100);
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
void jy901_writeREG(uint8_t reg, uint8_t *data, uint16_t length, uint16_t delayBeforeWrite, uint16_t delayAfterWrite)
{
    uint8_t unlockData[2] = {0x88, 0xB5};                                                         // 解锁数据
    uint8_t saveData[2] = {0x00, 0x00};                                                           // 保存数据
    HAL_I2C_Mem_Write(&hi2c2, JY901S_ADDRESS, KEY_REG, I2C_MEMADD_SIZE_8BIT, unlockData, 2, 100); // 解锁JY901 准备读数据
    if (delayBeforeWrite > 0)
    {
        HAL_Delay(delayBeforeWrite); // 延时
    }
    // 传输个人寄存器值start
    HAL_I2C_Mem_Write(&hi2c2, JY901S_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length, 100); // 向指定寄存器写入数据
    // 传输个人寄存器值end
    if (delayAfterWrite > 0)
    {
        HAL_Delay(delayAfterWrite); // 延时
    }
    HAL_I2C_Mem_Write(&hi2c2, JY901S_ADDRESS, SAVE, I2C_MEMADD_SIZE_8BIT, saveData, 2, 100); // 解锁JY901 准备读数据
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
    uint8_t data[2] = {0x08, 0x00}; // 设置角度参考，保存
    jy901_writeREG(CALSW, data, 2, 200, 3000);

    // 归零后，主角度Z轴（Yaw）为0
    AngleData.primary_angleZ = 0; // 主角度Z轴（Yaw）归零
    // 获取一次角度值
    jy901_dire_read(); // 读取方向相关传感器的值
    HAL_Delay(1000);   // 等待数据稳定
    // 获取z轴初始角度
    AngleData.primary_angleZ = AngleData.z_Yaw;
}

void jy901_readREG_IT(uint8_t reg, uint8_t *data, uint16_t length)
{
    HAL_I2C_Mem_Read_IT(&hi2c2, JY901S_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length); // 向i2c指定从设备的某一存储地址读出一个数，jy910s读完这个寄存器后指针会自增，可以连续读
    return;
}
void jy901_readREG(uint8_t reg, uint8_t *data, uint16_t length)
{
    HAL_I2C_Mem_Read(&hi2c2, JY901S_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length, 1000); // 向i2c指定从设备的某一存储地址读出一个数，jy910s读完这个寄存器后指针会自增，可以连续读
    return;
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
    uint8_t temt_data[8];
    jy901_readREG(PressureL, temt_data, 8);
    pressureData = (int32_t)((temt_data[3] << 24) | (temt_data[2] << 16) | (temt_data[1] << 8) | temt_data[0]);
    heightData = (int32_t)((temt_data[7] << 24) | (temt_data[6] << 16) | (temt_data[5] << 8) | temt_data[4]);
}

/*
 * @brief 读取方向相关传感器的值（double类型）
 *
 * @note 读取数据已写入AccelData（加速度）、GyroData（角速度）、MagData（磁力计）和AngleData（角度）结构体中
 * @note AccelData 单位为 m/s^2
 * @note GyroData 单位为 °/s
 * @note MagData 单位为 uT（微特斯拉）
 * @note AngleData 单位为 °
 *
 */
void jy901_dire_read_IT(void)
{
    jy901_readREG_IT(AX, JY901_data, 24); // 读取数据
}
void jy901_dire_read(void)
{
    jy901_readREG(AX, JY901_data, 24); // 读取数据
    // AccelData.x = (int16_t)((int16_t)(JY901_data[1] << 8 | JY901_data[0])) / 32768.0 * 16.0 * GRAVITY;
    // AccelData.y = (int16_t)((int16_t)(JY901_data[3] << 8 | JY901_data[2])) / 32768.0 * 16.0 * GRAVITY;
    // AccelData.z = (int16_t)((int16_t)(JY901_data[5] << 8 | JY901_data[4])) / 32768.0 * 16.0 * GRAVITY;
    //  角速度
    GyroData.x = (int16_t)((int16_t)(JY901_data[7] << 8 | JY901_data[6])) / 32768.0 * 2000.0;
    // GyroData.y = (int16_t)((int16_t)(JY901_data[9] << 8 | JY901_data[8])) / 32768.0 * 2000.0;
    GyroData.z = (int16_t)((int16_t)(JY901_data[11] << 8 | JY901_data[10])) / 32768.0 * 2000.0;
    // 磁力计
    // MagData.x = (int16_t)((int16_t)(JY901_data[13] << 8 | JY901_data[12]));
    // MagData.y = (int16_t)((int16_t)(JY901_data[15] << 8 | JY901_data[14]));
    // MagData.z = (int16_t)((int16_t)(JY901_data[17] << 8 | JY901_data[16]));
    // 角度
    AngleData.x_Roll = (int16_t)((int16_t)(JY901_data[19] << 8 | JY901_data[18])) / 32768.0 * 180.0;
    // AngleData.y_Pitch = (int16_t)((int16_t)(JY901_data[21] << 8 | JY901_data[20])) / 32768.0 * 180.0;
    AngleData.z_Yaw = (uint16_t)((uint16_t)(JY901_data[23] << 8 | JY901_data[22])) / 32768.0 * 180.0;
}

// jy901 z轴角度软件置0
void jy901_z_angle_zero(void)
{
    AngleData.z_Yaw = AngleData.z_Yaw - AngleData.primary_angleZ; // 主角度Z轴（Yaw）归零
    // 限制到0-360度范围
    if (AngleData.z_Yaw < 0)
    {
        AngleData.z_Yaw += 360.0;
    }
    else if (AngleData.z_Yaw >= 360.0)
    {
        AngleData.z_Yaw -= 360.0;
    }
}
