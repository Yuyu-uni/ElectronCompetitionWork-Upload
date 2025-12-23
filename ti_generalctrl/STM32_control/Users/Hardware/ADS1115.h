#ifndef __ADS1115_H__
#define __ADS1115_H__

#define ADS1115_I2C_Handle hi2c2 // I2C handle for ADS1115, change as needed
#include "stm32g474xx.h"
#include "ADS1115_REG.h"
#include "stdio.h"


enum MeasChannel
{
    AIN0_1 = 0x00,
    AIN0_3= 0x01,
    AIN1_3 = 0x02,
    AIN2_3 = 0x03,
    AIN0_GND = 0x04, // AIN0 and GND
    AIN1_GND = 0x05, // AIN1 and GND
    AIN2_GND = 0x06, // AIN2 and GND
    AIN3_GND = 0x07, // AIN3 and GND
    STILL = 0x08
};

/* -----------------------------------------------------------------------------------
 * 14:12 | MUX [2:0]      | 输入复用多路配置
 *       |                | 000 : AINP = AIN0 and AINN = AIN1
 *       |                | 001 : AINP = AIN0 and AINN = AIN3
 *       |                | 010 : AINP = AIN1 and AINN = AIN3
 *       |                | 011 : AINP = AIN2 and AINN = AIN3
 *       |                | 100 : AINP = AIN0 and AINN = GND
 *       |                | 101 : AINP = AIN1 and AINN = GND
 *       |                | 110 : AINP = AIN2 and AINN = GND
 *       |                | 111 : AINP = AIN3 and AINN = GND
 */

float FS = 0.0f; // Full Scale Range

void ADS1115_Init(void);
void ADS1115_WriteReg(uint8_t ConfigData_H, uint8_t ConfigData_L);
uint16_t ADS1115_ReadReg(uint8_t regADD);
float ADS1115_ReadADC(enum MeasChannel channel);

#endif // __ADS1115_H__
