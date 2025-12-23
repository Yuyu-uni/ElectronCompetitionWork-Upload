#include <stdio.h>
#include <math.h>      // 包含数学函数库
#include <math_tool.h> // 包含自定义的数学工具

#define G 9.8 // 重力加速度（单位：米/秒）

// 函数：计算单摆周期
double calculate_T(double length)
{
    return 2 * PI * sqrt(length / G);
}
// 函数：计算单摆长度mm
double calculate_L(double length)
{
    return 70 - length;
}
// 函数：计算倾角a
double calculate_a(double length)
{
    int B;          // 角度B
    int H_distance; // 高点测得距离
    int L_distance; // 低点测得距离
    double a;
    a = acos(sin(B) * H_distance / sqrt(pow(H_distance, 2) + pow(L_distance, 2) + 2 * H_distance * L_distance * cos(B)));
    return a;
}
