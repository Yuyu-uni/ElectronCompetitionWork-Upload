#ifndef __JY901_H__
#define __JY901_H__
#include "stdio.h"
#include "stm32g4xx_hal.h"

#define GRAVITY 9.80665f
void jy901_zeroXY(void);
void jy901_writeREG(uint8_t reg, uint8_t *data, uint16_t length, uint16_t delayBeforeWrite, uint16_t delayAfterWrite);
void jy901_readREG(uint8_t reg, uint8_t *data, uint16_t length);
void jy901_dire_read(void);
void jy901_dire_read_IT(void);
void jy901_readREG_IT(uint8_t reg, uint8_t *data, uint16_t length);
void jy901_z_angle_zero(void);

// 所有传感器数据
// 角度
struct AngleData
{
    double x_Roll;
    double y_Pitch;
    double z_Yaw;
    double primary_angleZ; // 主角度Z轴（Yaw）
};

// 加速度
struct AccelData
{
    double x;
    double y;
    double z;
};

// 角速度
struct GyroData
{
    double x;
    double y;
    double z;
};

// 磁力计
struct MagData
{
    double x;
    double y;
    double z;
};

extern uint8_t JY901_data[24];

extern struct AccelData AccelData;
extern struct GyroData GyroData;
extern struct MagData MagData;
extern struct AngleData AngleData;

extern int32_t pressureData;
extern int32_t heightData;


#endif
