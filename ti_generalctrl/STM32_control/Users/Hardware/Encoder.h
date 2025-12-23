#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

void Encoder_Init(void);

extern float right_speed; // 电机3速度
extern float left_speed;  // 电机4速度
extern double distance;   // 轮子行驶的距离

#endif // !ENCODER_H
