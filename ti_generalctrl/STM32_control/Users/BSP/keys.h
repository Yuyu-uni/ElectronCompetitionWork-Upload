#ifndef _KEYS_H_
#define _KEYS_H_

#include "stm32g474xx.h"
#include "gpio.h"
// Function declarations and definitions for key control

// ATTENTION: 按键的GPIO口需要配置为output模式并且开启内部上拉。引脚口低电平时为按键按下

// 时间间隔设定 宏定义-----------------按键使用不顺手可来这里调参
#define KEY_SCAN_COUNT 20         // 定义按键扫描时间间隔，单位毫秒。这种方式自带消抖，若消抖不够可以增大该值
#define KEY_HOLD_TIME 1000        // 定义按键长按时间阈值，单位毫秒
#define KEY_DOUBLE_TIME 200       // 定义按键双击时间阈值，单位毫秒
#define KEY_REPEAT_TIME 100       // 定义按键重复触发时间间隔，单位毫秒
#define KEY_STATE_ACTIVE_TIME 200 // 为防止按键状态被误读，提供状态可信时间，当状态超过该时间后不被阅读，将自动将状态全部置0

#define KEY_PRESSED 0x01   // 按键按下状态
#define KEY_UNPRESSED 0x00 // 按键未按下状态

// 对应标志位位掩码宏定义
#define KEY_HOLD_MASK 0x01   // 按键长按标志位
#define KEY_DOWN_MASK 0x02   // 按键按下标志位
#define KEY_UP_MASK 0x04     // 按键松开标志位
#define KEY_SINGLE_MASK 0x08 // 按键单击标志位
#define KEY_DOUBLE_MASK 0x10 // 按键双击标志位
#define KEY_LONG_MASK 0x20   // 按键长按标志位
#define KEY_REPEAT_MASK 0x40 // 按键重复触发标志位

typedef struct
{
    uint8_t KeyNum; // 按键编号

    GPIO_TypeDef *port; // 按键GPIO口
    uint16_t pin;       // 按键引脚

    uint8_t state; // 按键状态，用8位0和1表示按键长按短按 双击等状态
                   /*  7    6      5      4      3    2   1    0
                    *  0  REPEAT  LONG  DOUBLE SINGLE UP DOWN HOLD
                    */

    uint8_t key_count;    // key_count进行预分频
    uint8_t currentState; // 当前按键状态
    uint8_t prevState;    // 上一次检测的按键状态
    uint8_t S;            // 状态机状态 具体含义请参照状态转移图

    uint16_t thresholdTime; // threshold time用于记录多个时间阈值 在状态机中由于各个状态互斥，这一个变量可以用于多个阈值的计数 每ms减少1

    uint16_t lastReadDuration; // 此次读取状态与上次读取状态的时间间隔，单位ms。当时间间隔过大时 舍弃这次状态读取，防止这段时间中对按键的误触导致错误输出
} KeyStruct;

void keys_init(KeyStruct *keys, uint8_t keys_num);
uint8_t keys_readState(KeyStruct *keys, uint8_t flag_mask);

uint8_t keys_getState(KeyStruct *keys);

void keys_tick(KeyStruct *keys);
#endif // _KEYS_H_
