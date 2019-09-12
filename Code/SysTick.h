#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f30xe.h"

#define SYSTICK_PERIOD		20

#define SYSTICK_US_VAL		SYSTICK_PERIOD*72	// 20us systick time

void SysTick_Init(uint32_t ticks);
void SysTick_handler(void);
void delay_us(uint32_t n);
void delay_ms(uint32_t n);

#endif /* __SYSTICK_H */

