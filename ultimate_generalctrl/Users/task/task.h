#ifndef _TASK_H
#define _TASK_H

#include "main.h"
#include "task.h"
#include "PID.h"
#include "StepMotor.h"

extern PIDController point;
extern StepMotorStruct stepMotorA, stepMotorB; // 定义步进电机结构体
extern float x_angle;                          // x轴角度值
extern float y_angle;                          // y轴角度值

void test_motor(void);
void task_init(void);
void test_kalman_filter(void);
extern float kalman_state[2];           // 状态：[x, y]
extern float kalman_errorCovariance[2]; // 误差协方差
extern uint8_t kalman_data_ready;       // 数据准备标志
extern float input_measurement[2];      // 输入测量值
#endif                                  // !_TASK_H
