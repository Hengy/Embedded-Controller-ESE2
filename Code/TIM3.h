#ifndef __TIM3_H
#define __TIM3_H

#include "stm32f30xe.h"
#include "utils.h"

// DC signal frequency
#define DRV8814_DC_FREQ				999	// ~50Hz

#define DRV8814_MIN_DUTY			75	// ~7.5%

void TIM3_Init(uint16_t freq);
void TIM3_set_ch3(uint16_t duty);
void TIM3_set_ch4(uint16_t duty);
void TIM3_enable_ch3(void);
void TIM3_enable_ch4(void);
void TIM3_disable_ch3(void);
void TIM3_disable_ch4(void);

#endif /* __TIM3_H */

