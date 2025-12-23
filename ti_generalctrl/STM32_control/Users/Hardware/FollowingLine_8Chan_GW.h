#ifndef _FOLLOWINGLINE_8CHAN_GW_H_
#define _FOLLOWINGLINE_8CHAN_GW_H_
#include "stm32g474xx.h"
#include "stdio.h"

#define FOLLOWINGLINE_8CHAN_GW_I2C_Handle hi2c3 // I2C handle for FollowingLine_8Chan_GW, change as needed

/*---------------------------ADDR Define-----------------------------*/
#define FOLLOWINGLINEADDR 0x9E
/*---------------------------REG Define-----------------------------*/
#define DigitalDataREG 0xDD

#define PingTestREG 0xAA

extern uint8_t Follow_Data; // 用于中断，存储感为八路灰度传感器的数字数据
extern int8_t Follow_value; // 用于存储感为八路灰度传感器的加权输出
extern int Follow_num;      // 用于存储感为八路灰度传感器亮起个数

void FollowingLine_8Chan_GW_Init(void);
uint8_t FollowingLine_8Chan_GW_Read(void);
void FollowingLine_8Chan_GW_Write_Common(uint8_t reg, uint8_t *data, uint8_t len);
uint8_t FollowingLine_Analysis(uint8_t Follow_Data);

#endif
