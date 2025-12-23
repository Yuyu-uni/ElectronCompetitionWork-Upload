#ifndef _TASK_H
#define _TASK_H

#include "main.h"
#include "task.h"
#include "PID.h"
#include "StepMotor.h"
#include <stdint.h>
#include "keys.h"

extern PIDController point;
extern StepMotorStruct stepMotorA, stepMotorB; // 定义步进电机结构体
extern float x_angle;                          // x轴角度值
extern float y_angle;                          // y轴角度值

extern KeyStruct keys1, keys2, keys3, keys4; // 定义按键结构体

void task_init(void);
void task_point_follow(void);

#endif                                  // !_TASK_H
