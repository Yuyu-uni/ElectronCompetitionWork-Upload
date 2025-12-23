#ifndef _LEDS_H_
#define _LEDS_H_

#include "stm32g474xx.h"
#include "gpio.h"
// Function declarations and definitions for LED control
#define LED_COUNT 4 // 定义LED数量

enum LEDNum {
    ULED1 = 0,
    ULED2,
    ULED3,
    ULED4
};

void leds_init(void);
void leds_on(enum LEDNum led);
void leds_off(enum LEDNum led);
void leds_last_on(enum LEDNum led, uint16_t duration);

void leds_tick(enum LEDNum led);

#endif // _LEDS_H_
