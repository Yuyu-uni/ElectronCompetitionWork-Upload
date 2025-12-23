#ifndef _BUZZER_H_
#define _BUZZER_H_
#include "stm32g474xx.h"
#include "gpio.h"
// Function declarations and definitions for the buzzer control

void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);

void buzzer_last_on(uint16_t duration);

void buzzer_tick(void);

#endif // _BUZZER_H_
