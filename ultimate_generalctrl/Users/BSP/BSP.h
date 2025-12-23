#ifndef _BSP_H_
#define _BSP_H_

#include "stm32g474xx.h"
#include "gpio.h"
#include "leds.h"
#include "buzzer.h"
#include "keys.h"

// Function declarations and definitions for the BSP (Board Support Package)


void BSP_Init(KeyStruct *keys1, KeyStruct *keys2, KeyStruct *keys3, KeyStruct *keys4);


void BSP_Tick(void);


#endif // _BSP_H_
