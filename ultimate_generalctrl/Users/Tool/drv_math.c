#include "main.h"
#include "drv_math.h"

#define PI (3.14159265f)
float abs(float x)
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

// 标准一维卡尔曼滤波
/*
 * @brief: 一维卡尔曼滤波器
 * @param measurement: 测量值
 *        例1：float measurement = acc_angle;      // 加速度计角度
 *        例2：float measurement = pos_x;          // x方向位置
 * @param state: 当前状态
 *        例1：float state = 0;                    // 初始角度
 *        例2：float state = 0;                    // 初始x位置
 * @param errorCovariance: 当前误差协方差
 *        例1：float errorCovariance = 1;
 *        例2：float errorCovariance = 0.5;
 * @param processModel: 过程模型，也叫状态转移系数[0,1]，变化越符合线性，系数越接近1
 *        例1：float processModel = 1;             // 匀速或无外部输入
 *        例2：float processModel = 0.98;          // 有衰减或递减趋势
 * @param measurementNoise: 测量噪声，可以测得
 *        例1：float measurementNoise = acc_var;
 *        例2：float measurementNoise = 0.1;
 * @param processNoise: 过程噪声，无法直接测得，需要不断调节来逼近拟合状态，一开始可以设置为0.1或0.01，慢慢调
 *        例1：float processNoise = 0.01;
 *        例2：float processNoise = 0.1;
 * @return: 更新后的状态值
 * This function implements a simple one-dimensional Kalman filter.
 * It predicts the next state based on the current state and process model,
 * then updates the state based on the new measurement and the Kalman gain.
 * The function returns the updated state value.
 * 1. 误差协方差初始值 (errorCovariance)
当前值：通常设为 {1.0f, 1.0f}
快速收敛调整：增大初始值 → {10.0f, 10.0f} 或更大
原理：初始协方差越大，滤波器对测量值的信任度越高，收敛越快
2. 测量噪声 (measurementNoise)
当前值：对角线元素通常为 {0.1f, 0.1f}
快速收敛调整：减小测量噪声 → {0.01f, 0.01f} 或 {0.001f, 0.001f}
原理：测量噪声越小，滤波器越信任测量值，响应越快
3. 过程噪声 (processNoise)
当前值：对角线元素通常为 {0.01f, 0.01f}
快速收敛调整：增大过程噪声 → {0.1f, 0.1f} 或 {0.5f, 0.5f}
原理：过程噪声越大，滤波器对预测的信心越低，更依赖测量值
 */
float kalmanFilter(float measurement, float *state, float *errorCovariance, float processModel, float measurementNoise, float processNoise)
{
    // 预测步骤
    *state = processModel * (*state);                                                   // 状态预测
    *errorCovariance = processModel * (*errorCovariance) * processModel + processNoise; // 误差协方差预测

    // 计算卡尔曼增益
    float kalmanGain = *errorCovariance / (*errorCovariance + measurementNoise);

    // 更新步骤
    *state = *state + kalmanGain * (measurement - *state);    // 更新状态
    *errorCovariance = (1 - kalmanGain) * (*errorCovariance); // 更新误差协方差

    return *state;
}

// 二维卡尔曼滤波
/*
 * @brief: 二维卡尔曼滤波器
 * @param measurement: 测量值数组，包含x和y方向的测量值
 *        例1：float measurement[2] = {acc_angle, gyro_rate};
 *        例2：float measurement[2] = {pos_x, pos_y};
 * @param state: 当前状态数组，包含x和y方向的状态
 *        例1：float state[2] = {0, 0}; // 初始角度和角速度
 *        例2：float state[2] = {0, 0}; // 初始x和y位置
 * @param errorCovariance: 当前误差协方差数组，包含x和y方向的误差协方差
 *        例1：float errorCovariance[2] = {1, 1};
 *        例2：float errorCovariance[2] = {0.5, 0.5};
 * @param processModel: 状态转移矩阵
 *        例1：float processModel[2][2] = { {1, dt}, {0, 1} }; // 位置-速度模型
 *        例2：float processModel[2][2] = { {1, 0}, {0, 1} };  // 简单无耦合模型
 * @param measurementNoise: 测量噪声矩阵
 *        例1：float measurementNoise[2][2] = { {acc_var, 0}, {0, gyro_var} };
 *        例2：float measurementNoise[2][2] = { {0.1, 0}, {0, 0.1} };
 * @param processNoise: 过程噪声矩阵
 *        例1：float processNoise[2][2] = { {q_angle, 0}, {0, q_gyro} };
 *        例2：float processNoise[2][2] = { {0.01, 0}, {0, 0.01} };
 * @note  processModel 决定了当前状态如何由上一次状态线性推导，实际应用时需结合系统物理模型设置。
 * @return: 无返回值，直接更新状态和误差协方差
 */
void kalmanFilter2D(float measurement[2], float *state, float *errorCovariance, float processModel[2][2], float measurementNoise[2][2], float processNoise[2][2])
{
    // 预测步骤
    state[0] = processModel[0][0] * state[0] + processModel[0][1] * state[1]; // x方向状态预测
    state[1] = processModel[1][0] * state[0] + processModel[1][1] * state[1]; // y方向状态预测

    errorCovariance[0] = processModel[0][0] * errorCovariance[0] * processModel[0][0] + processNoise[0][0]; // x方向误差协方差预测
    errorCovariance[1] = processModel[1][1] * errorCovariance[1] * processModel[1][1] + processNoise[1][1]; // y方向误差协方差预测

    // 计算卡尔曼增益
    float kalmanGainX = errorCovariance[0] / (errorCovariance[0] + measurementNoise[0][0]);
    float kalmanGainY = errorCovariance[1] / (errorCovariance[1] + measurementNoise[1][1]);

    // 更新状态，求出后验估计值
    state[0] += kalmanGainX * (measurement[0] - state[0]); // 更新x方向状态
    state[1] += kalmanGainY * (measurement[1] - state[1]); // 更新y方向状态

    // 更新误差协方差
    errorCovariance[0] *= (1 - kalmanGainX); // 更新x方向误差协方差
    errorCovariance[1] *= (1 - kalmanGainY); // 更新y方向误差协方差
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
