#include "drv_math.h"
#include "ti_msp_dl_config.h"

#define PI (3.14159265f)
float myabs(float x)
{
    return (x < 0) ? -x : x;
}

// 限幅函数
float limit(float value, double min, double max)
{
    if (value > max)
        return max;
    else if (value < min)
        return min;
    else
        return value;
}
// 角度防跳变函数-180到180度
float angle_wrap(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}
