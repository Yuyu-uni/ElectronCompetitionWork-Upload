#ifndef _TASK_H
#define _TASK_H

#include "PID.h"
#include "ti_msp_dl_config.h"

typedef struct
{
    uint8_t rote[2]; // 控制圈数
    uint8_t state;    // 0: STOP, 1: STRAIGHT,2直角
    float base_speed; // 基础速度用于控制直行和转弯
    float location;      // 转向角度，用于输入到角度环
    float direction;  // 循迹方向数据
} FollowControl;

extern PIDController point;
extern FollowControl follow_control; // 跟随控制结构体实例

void task_init(void);

#endif                                  // !_TASK_H
