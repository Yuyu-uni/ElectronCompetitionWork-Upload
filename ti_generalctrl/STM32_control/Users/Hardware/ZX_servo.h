#ifndef __ZX_SERVO_H__
#define __ZX_SERVO_H__
#include "stm32g474xx.h"
#include "usart.h"
#include "stdio.h"

#define ZX_SERVO_SIG_UART_HANDLE huart3 // 使用USART3作为ZX_SERVO的信号传输

typedef struct
{
    uint8_t id;          // 设备ID
    char mode;           // 工作模式
    int16_t angle_range; // 角度范围
} ZX_SERVO_Struct;

void ZX_SERVO_Init(ZX_SERVO_Struct *servo, uint8_t id, char mode);

void ZX_SERVO_SetAngle(ZX_SERVO_Struct *servo, int16_t target_angle);
// 普通PWM舵机
void Servo_Init(void);
void Servo_SetAngle(uint8_t servo_id, uint16_t angle);
#endif // ZX_servo.h
