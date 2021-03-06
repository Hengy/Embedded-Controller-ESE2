#include "TIM16.h"

void TIM16_Init(uint16_t freq) {
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;		// enable TIM16 clocks
	
	CLR_BITS(TIM16->CR1, 0x10);		// clear DIR bit (up-counting)
	
	TIM16->PSC = 719;	// 100KHz counter frequency
	
	TIM16->ARR = freq;
	
	SET_BITS(TIM16->CR1,0x80);			// auto-reload preload value
	
	SET_BITS(TIM16->CR1,0x01);					// enable counter
	
	// enable update interrupts
	TIM16->DIER |= TIM_DIER_UIE;
	
	// enable TIM16 interrupt in NVIC
	NVIC_EnableIRQ(TIM16_IRQn);
}

void TIM16_set(uint16_t freq) {
	if (freq) {
		TIM16->ARR = freq;		
	} else {
		TIM16_disable();
	}
}

void TIM16_enable() {
	SET_BITS(TIM16->BDTR,0x8000);			// Enable master output
}

void TIM16_disable() {
	CLR_BITS(TIM16->BDTR,0x8000);			// disable master output
}

