#ifndef __TIM17_H
#define __TIM17_H

#include "stm32f30xe.h"
#include "utils.h"

#define TIM17_OUTPUT_PORT		B
#define TIM17_OUTPUT_PIN			7

// servo signal frequency
#define LCD_BL_PWMFREQ			199	// 500Hz

// centre position
#define LCD_BL_PWMDEFAULT 		159	// ~80%
#define LCD_BL_PWMMIN			30	//~15%

void TIM17_Init(uint16_t freq, uint16_t duty);
void TIM17_set(uint16_t freq, uint16_t percent);
void TIM17_enable(void);
void TIM17_disable(void);
static uint16_t TIM17_ToVal(uint16_t percent);

#endif /* __TIM17_H */

