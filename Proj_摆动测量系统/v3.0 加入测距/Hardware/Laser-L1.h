#ifndef LASER_L1_H
#define LASER_L1_H
#include "stm32g474xx.h"

enum MeasureMode
{
    OnceMeasureMode = 0x02,
    ConstantMeasureMode = 0x03,
    FastMeasareMode = 0x04,
};

void Laser_Init(enum MeasureMode mode);
uint32_t Laser_GetDistance(void);

#endif /* LASER_L1_H */
