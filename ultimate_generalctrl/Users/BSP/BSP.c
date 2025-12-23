#include "BSP.h"
#include "tim.h"
static KeyStruct *keysArr[4]; // 用于存储按键结构体指针数组

/*
 * @brief BSP initialization function
 * @note 在该函数中实现了所有BSP外设的初始化
 *       包括LED、蜂鸣器、按键。并启用了定时器和BSP状态机
 */
void BSP_Init(KeyStruct *keys1, KeyStruct *keys2, KeyStruct *keys3, KeyStruct *keys4)
{
    // 初始化按键
    keys_init(keys1, 1);
    keys_init(keys2, 2);
    keys_init(keys3, 3);
    keys_init(keys4, 4);

    keysArr[0] = keys1;
    keysArr[1] = keys2;
    keysArr[2] = keys3;
    keysArr[3] = keys4;

    // 启用定时器中断
    HAL_TIM_Base_Start_IT(&htim16); // 启动定时器TIM16用于BSP状态机

    buzzer_init(); // 初始化蜂鸣器
    leds_init();   // 初始化LED
}

void BSP_Tick(void)
{
    // 每1ms被中断调用一次BSP状态机
    keys_tick(keysArr[0]);
    keys_tick(keysArr[1]);
    keys_tick(keysArr[2]);
    keys_tick(keysArr[3]);
    buzzer_tick();
    leds_tick(ULED1);
    leds_tick(ULED2);
    leds_tick(ULED3);
    leds_tick(ULED4);
}
