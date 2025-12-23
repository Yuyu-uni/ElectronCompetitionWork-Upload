#include <stdio.h>
#include "main.h"
#include "math_tool.h"
#include "arm_math.h"
#include "arm_const_structs.h"

#define POW(base, exp) ({              \
    int _result = 1;                   \
    for (int _i = 0; _i < (exp); ++_i) \
    {                                  \
        _result *= (base);             \
    }                                  \
    _result;                           \
}) // 宏定义幂函数

#define G 9.8 // 重力加速度（单位：米/秒）

// 函数：计算单摆周期
double calculate_T(double length)
{
    float sqrt_length = 0;
    arm_sqrt_f32(length / G, &sqrt_length);
    float T = 2 * PI * sqrt_length; // 单摆周期公式
    return T;
}
// 函数：计算单摆长度mm
double calculate_L(double length)
{
    float calulate_length = 70.0f - length;
    return calulate_length; // 单摆长度公式
}

// 函数：计算倾角a
// float calculate_a(float B, float H_distance, float L_distance)
// {
//     float sinB, cosB, sum, sqrtSum, numerator, denominator, temp, a;

//     arm_sin_cos_f32(B, &sinB, &cosB);

//     sum = H_distance * H_distance + L_distance * L_distance - 2.0f * H_distance * L_distance * cosB;

//     arm_sqrt_f32(sum, &sqrtSum);

//     numerator = sinB * H_distance;
//     denominator = sqrtSum;

//     temp = numerator / denominator;
//     a = acos(temp);

//     return a;
// }

float calculate_a(float B, float H_distance, float L_distance)
{
    volatile float sinB, cosB, sum, sqrtSum, numerator, denominator, temp, a;

    arm_sin_cos_f32(-B, &sinB, &cosB);

    sum = H_distance * H_distance + L_distance * L_distance - 2.0f * H_distance * L_distance * cosB;

    arm_sqrt_f32(sum, &sqrtSum);

    numerator = sinB * H_distance;
    denominator = sqrtSum;

    temp = numerator / denominator;
    a = acos(temp);

    return a;
}
float calculate_a_planB(float L_distance)
{
    uint16_t f = 433 - L_distance; // 733mm为绳顶端到地面的距离
    float sina = f / 500.0;        // 500mm为底板长度的一半
    return asin(sina);             // 计算弧度
}
// 弧度制转换
float radian(float angle)
{
    return angle * PI / 180.0f;
}
// 角度制转换
float degree(float angle)
{
    return angle * 180.0f / PI;
}