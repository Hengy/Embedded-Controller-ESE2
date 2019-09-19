#ifndef __TIM16_H
#define __TIM16_H

#include "stm32f30xe.h"
#include "utils.h"

// servo signal frequency
#define STEP_PWMFREQFAST			389		// ~256Hz
#define STEP_PWMFREQMED				780		// ~128Hz
#define STEP_PWMFREQSLOW			1561	// ~64Hz

void TIM16_Init(uint16_t freq);
void TIM16_set(uint16_t freq);
void TIM16_enable(void);
void TIM16_disable(void);

#endif /* __TIM16_H */

