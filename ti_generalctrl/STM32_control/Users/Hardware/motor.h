#include "stm32g4xx_hal.h"
#include "main.h"

#ifndef _MOTOR_H
#define _MOTOR_H

#define PWM_MAX 17000
#define PWM_MIN 17000

// 更新后的 GPIO 信号定义
#define RB_IN(x) HAL_GPIO_WritePin(RB_IN_GPIO_Port, RB_IN_Pin, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define RF_IN(x) HAL_GPIO_WritePin(RF_IN_GPIO_Port, RF_IN_Pin, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define LB_IN(x) HAL_GPIO_WritePin(LB_IN_GPIO_Port, LB_IN_Pin, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define LF_IN(x) HAL_GPIO_WritePin(LF_IN_GPIO_Port, LF_IN_Pin, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)

// 函数声明
void Stop(float *Med_Jiaodu, float *Jiaodu);
void Motor_Init(void); /* 初始化电机 */
void Limit(float *motoA, float *motoB);
int GFP_abs(int p);
void Load(int moto1, int moto2, int moto3, int moto4);
void Motor_Init(void);

#endif
