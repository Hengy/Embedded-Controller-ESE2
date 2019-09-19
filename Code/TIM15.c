#include "TIM15.h"

void TIM15_Init(uint32_t freq) {
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;		// enable TIM15 clocks
	
	CLR_BITS(TIM15->CR1, 0x10);		// clear DIR bit (up-counting)
	
	TIM15->PSC = 719;	// 100KHz counter frequency
	
	TIM15->ARR = 25000;
	
	SET_BITS(TIM15->CR1,0x80);			// auto-reload preload value
	
	SET_BITS(TIM15->CR1,0x01);					// enable counter
	
	// enable update interrupts
	TIM15->DIER |= TIM_DIER_UIE;
	
	// enable TIM15 interrupt in NVIC
	NVIC_EnableIRQ(TIM15_IRQn);
}

void TIM15_set(uint32_t freq) {
	if (freq) {
		TIM15->ARR = freq;		
	} else {
		TIM15_disable();
	}
}

void TIM15_enable() {
	SET_BITS(TIM15->BDTR,0x8000);			// Enable master output
}

void TIM15_disable() {
	CLR_BITS(TIM15->BDTR,0x8000);			// disable master output
}

