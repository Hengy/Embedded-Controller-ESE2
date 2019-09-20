#include "lim_sw.h"

void lim_sw_Init() {
	
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;	// Enable clock of Port B
	
	GPIO_PIN_MODE(LIM_PORT,LIML_PIN,IN);
	GPIO_PIN_MODE(LIM_PORT,LIMR_PIN,IN);
	
	GPIO_PIN_AF_MODE(LIM_PORT,LIML_PIN,0x00);				// set AF mode
	
	GPIO_PIN_AF_MODE(LIM_PORT,LIMR_PIN,0x00);				// set AF mode
	
	// SYSCFG clock
	SET_BITS(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
	
	// external interrupt
	FORCE_BITS(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI2_Msk, SYSCFG_EXTICR1_EXTI2_PB);	// PB2 external interrupt source
	FORCE_BITS(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI1_Msk, SYSCFG_EXTICR1_EXTI1_PB);	// PB1 external interrupt source
	
	CLR_BITS(EXTI->FTSR, EXTI_FTSR_FT2);	// Falling edge trigger for EXTI2
	CLR_BITS(EXTI->FTSR, EXTI_FTSR_FT1);	// Falling edge trigger for EXTI1
	
	SET_BITS(EXTI->RTSR, EXTI_RTSR_RT2);	// Rising edge trigger for EXTI2
	SET_BITS(EXTI->RTSR, EXTI_RTSR_RT1);	// Rising edge trigger for EXTI1
	
	SET_BITS(EXTI->IMR, EXTI_IMR_IM2);		// Enable interrupt
	SET_BITS(EXTI->IMR, EXTI_IMR_IM1);		// Enable interrupt
	
	NVIC_EnableIRQ(EXTI2_TSC_IRQn);		// Enable NVIC IRQ
	NVIC_EnableIRQ(EXTI1_IRQn);			// Enable NVIC IRQ
}
