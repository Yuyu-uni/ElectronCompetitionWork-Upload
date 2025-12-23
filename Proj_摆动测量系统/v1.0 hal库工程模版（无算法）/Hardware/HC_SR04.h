#ifdef __HC_SR04_H
#define __HC_SR04_H

#include "HC_SR04.h"
#include "stm32g4xx_hal.h"
#include "main.h"

uint16_t HC_SR04_GetDistance_Average(void);
uint16_t HC_SR04_GetDistance(void);
void HC_SR04_Init(void);

#endif
