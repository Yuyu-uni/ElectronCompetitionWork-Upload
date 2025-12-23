#ifndef _MOTOR_H
#define _MOTOR_H
#include "ti_msp_dl_config.h"
#include "stdint.h"
void Stop(float *Med_Jiaodu, float *Jiaodu);
void Motor_Init(void);
void Limit(float *motoA, float *motoB);
int GFP_abs(int p);
void Load(float moto1, float moto2);
uint8_t FollowingLine_Analysis(uint8_t Follow_Data);
void Follow_direct(void);

extern uint8_t Follow_Data; // 用于中断，存储感为八路灰度传感器的数字数据
extern int8_t Follow_value; // 用于存储感为八路灰度传感器的加权输出
extern int Follow_num;      // 用于存储感为八路灰度传感器的通道数

#define PWM_MAX 800
#define PWM_MIN -800

#endif
