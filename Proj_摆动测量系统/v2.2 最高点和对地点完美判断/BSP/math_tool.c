#include <stdio.h>
#include <math.h>      // 包含数学函数库
#include <math_tool.h> // 包含自定义的数学工具

#define G 9.8        // 重力加速度（单位：米/秒）

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
