#ifndef __JY901_H__
#define __JY901_H__
#include "stdio.h"
#include "stm32g4xx_hal.h"

#define GRAVITY 9.80665f
void jy901_zeroXY(void);
void jy901_writeREG(uint8_t reg, uint8_t *data, uint16_t length, uint16_t delayBeforeWrite, uint16_t delayAfterWrite);
void jy901_readREG(uint8_t reg, uint8_t *data, uint16_t length);
void jy901_dire_read(void);

extern int32_t pressureData; // 气压计
extern int32_t heightData;   // 高度值

#endif
