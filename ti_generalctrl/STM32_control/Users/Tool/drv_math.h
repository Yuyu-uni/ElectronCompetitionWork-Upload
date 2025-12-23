#ifndef drv_math_h
#define drv_math_h

#define PI (3.14159265f)

#include "main.h"

float abs(float x);
float limit(float value, double min, double max);

// 传入state返回预测值，用于运动目标追踪
float kalmanFilter(float measurement, float *state, float *errorCovariance, float processModel, float measurementNoise, float processNoise);

// 传入state实时更新为预测值，用于传感器融合和二维目标追踪
void kalmanFilter2D(float measurement[2], float *state, float *errorCovariance, float processModel[2][2], float measurementNoise[2][2], float processNoise[2][2]);

float angle_wrap(float angle);
float angle_wrap_360(float angle);

#endif // !drv_math_h
