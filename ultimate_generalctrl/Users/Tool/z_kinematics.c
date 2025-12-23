#include "z_kinematics.h"
#include <math.h>

#define pi 3.1415926

/*
    设置四个关节的长度
    单位1mm
    直接照着这个设置完了
        setup_kinematics(110, 105, 75, 190, &kinematics); // kinematics 90mm 105mm 98mm 150mm
*/

void setup_kinematics(float L0, float L1, float L2, float L3, kinematics_t *kinematics)
{
    // 放大10倍
    kinematics->L0 = L0 * 10;
    kinematics->L1 = L1 * 10;
    kinematics->L2 = L2 * 10;
    kinematics->L3 = L3 * 10;
}

/*
    x,y 为映射到平面的坐标
    z为距离地面的距离
    Alpha 为爪子和平面的夹角 -25~-65范围比较好
*/

int kinematics_analysis(double x, double y, double z, float Alpha, kinematics_t *kinematics)
{
    double theta3, theta4, theta5, theta6; // 各关节角度
    float l0, l1, l2, l3;                  // 机械臂各段长度
    float aaa, bbb, ccc, zf_flag;          // 中间变量

    // 放大10倍，单位转换，*10消浮点数，求算角度同比例放大不影响
    x = x * 10; // x = x * 10
    y = y * 10; // y = y * 10
    z = z * 10; // z = z * 10

    // 注意这是小写L，难绷
    l0 = kinematics->L0; // 读取基座到第一关节长度
    l1 = kinematics->L1; // 读取第一关节到第二关节长度
    l2 = kinematics->L2; // 读取第二关节到第三关节长度
    l3 = kinematics->L3; // 读取第三关节到末端长度

    if (x == 0)
    {
        theta6 = 0.0; // 若x为0，末端旋转角为0
    }
    else
    {
        theta6 = atan(x / y) * 180.0 / pi; // theta6 = atan(x/y) * 180 / pi，末端基座旋转角
    }

    y = sqrt(x * x + y * y);                     // y = sqrt(x^2 + y^2)，将x、y合成平面距离
    y = y - (double)l3 * cos((double)Alpha * (double)pi / 180.0);      // y = y - l3*cos(Alpha)，减去末端投影长度
    z = z - l0 - (double)l3 * sin((double)Alpha * (double)pi / 180.0); // z = z - l0 - l3*sin(Alpha)，减去基座高度和末端高度

    if (z < -l0) // 比第一关节低
    {
        return 1; // 超出机械臂下限
    }
    if (sqrt(y * y + z * z) > (l1 + l2))
    {
        return 2; // 超出机械臂最大工作半径
    }

    /*上面是第一关节迭代解算完成，接下来开始第二次迭代*/

    ccc = acos(y / sqrt(y * y + z * z));                                        // ccc = arccos(y / sqrt(y^2 + z^2))，夹角
    bbb = (y * y + z * z + l1 * l1 - l2 * l2) / (2 * l1 * sqrt(y * y + z * z)); // 余弦定理，bbb = (y^2 + z^2 + l1^2 - l2^2) / (2*l1*sqrt(y^2+z^2))
    if (bbb > 1 || bbb < -1)
    {
        return 5; // 超出可达范围
    }
    if (z < 0)
    {
        zf_flag = -1; // z为负，方向标志
    }
    else
    {
        zf_flag = 1; // z为正，方向标志
    }
    theta5 = ccc * zf_flag + acos(bbb); // theta5 = ccc*符号 + arccos(bbb)
    theta5 = theta5 * 180.0 / pi;       // 角度制转换
    if (theta5 > 180.0 || theta5 < 0.0)
    {
        return 6; // 超出关节极限
    }

    aaa = -(y * y + z * z - l1 * l1 - l2 * l2) / (2 * l1 * l2); // 余弦定理，aaa = -(y^2 + z^2 - l1^2 - l2^2) / (2*l1*l2)
    if (aaa > 1 || aaa < -1)
    {
        return 3; // 超出可达范围
    }
    theta4 = acos(aaa);                   // theta4 = arccos(aaa)
    theta4 = 180.0 - theta4 * 180.0 / pi; // 角度制转换，theta4 = 180 - theta4
    if (theta4 > 135.0 || theta4 < -135.0)
    {
        return 4; // 超出关节极限
    }

    theta3 = Alpha - theta5 + theta4; // theta3 = Alpha - theta5 + theta4，末端姿态补偿
    if (theta3 > 90.0 || theta3 < -90.0)
    {
        return 7; // 超出关节极限
    }

    kinematics->servo_angle[0] = theta6;      // 舵机0角度
    kinematics->servo_angle[1] = theta5 - 90; // 舵机1角度
    kinematics->servo_angle[2] = theta4;      // 舵机2角度
    kinematics->servo_angle[3] = theta3;      // 舵机3角度

    kinematics->servo_pwm[0] = (int)(1500 - 2000.0f * kinematics->servo_angle[0] / 270.0f); // PWM = 1500 - 2000*角度/270
    kinematics->servo_pwm[1] = (int)(1500 + 2000.0f * kinematics->servo_angle[1] / 270.0f); // PWM = 1500 + 2000*角度/270
    kinematics->servo_pwm[2] = (int)(1500 + 2000.0f * kinematics->servo_angle[2] / 270.0f); // PWM = 1500 + 2000*角度/270
    kinematics->servo_pwm[3] = (int)(1500 + 2000.0f * kinematics->servo_angle[3] / 270.0f); // PWM = 1500 + 2000*角度/270

    return 0; // 逆运动学求解成功
}
