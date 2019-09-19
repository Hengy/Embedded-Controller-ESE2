#include "DRV8884.h"

void DRV8884_Init(void) {
	
	// enable port clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;	// Enable clock of Port A
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;	// Enable clock of Port B
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;	// Enable clock of Port C
	
	// step
	GPIO_PIN_MODE(DRV8884_STEP_PORT, DRV8884_STEP_PIN, OUT);	// output
	GPIO_PIN_DRV_TYPE(DRV8884_STEP_PORT, DRV8884_STEP_PIN, PP);	// push-pull
	
	// dir
	GPIO_PIN_MODE(DRV8884_DIR_PORT, DRV8884_DIR_PIN, OUT);	// output
	GPIO_PIN_DRV_TYPE(DRV8884_DIR_PORT, DRV8884_DIR_PIN, PP);	// push-pull
	
	// sleep
	GPIO_PIN_MODE(DRV8884_SLP_PORT, DRV8884_SLP_PIN, OUT);	// output
	GPIO_PIN_DRV_TYPE(DRV8884_SLP_PORT, DRV8884_SLP_PIN, PP);	// push-pull
	
	// enable
	GPIO_PIN_MODE(DRV8884_EN_PORT, DRV8884_EN_PIN, OUT);	// output
	GPIO_PIN_DRV_TYPE(DRV8884_EN_PORT, DRV8884_EN_PIN, PP);	// push-pull
	
	// fault
	GPIO_PIN_MODE(DRV8884_FLT_PORT, DRV8884_FLT_PIN, IN);	// output
	
	// SYSCFG clock
	SET_BITS(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
	
	// external interrupt
	SET_BITS(SYSCFG->EXTICR[2], SYSCFG_EXTICR3_EXTI10_PA);	// PA10 external interrupt source
	
	SET_BITS(EXTI->FTSR, EXTI_FTSR_FT10);	// Falling edge trigger for EXTI3
	
	SET_BITS(EXTI->RTSR, EXTI_RTSR_RT10);	// Rising edge trigger for EXTI3
	
	SET_BITS(EXTI->IMR, EXTI_IMR_IM10);		// Enable interrupt
	
	NVIC_EnableIRQ(EXTI3_IRQn);		// Enable NVIC IRQ
	
	// TIM16
	TIM16_Init(STEP_PWMFREQSLOW);
	TIM16_enable();
	
	DRV8884_SLEEP;
	DRV8884_DISABLE;
	DRV8884_STEP_LO;
	DRV8884_DIR_CW;
}
	
void DRV8884_step(void) {
	DRV8884_STEP_HI;
	uint32_t wait = 50;
	while (wait--);
	DRV8884_STEP_LO;
}

void DRV8884_sleep(void) {
	DRV8884_SLEEP;
	DRV8884_awake = 0;
}

void DRV8884_wake(void) {
	DRV8884_WAKE;
	DRV8884_awake = 1;
}

void DRV8884_enable(void) {
	DRV8884_ENABLE;
	DRV8884_enabled = 1;
}

void DRV8884_disable(void) {
	DRV8884_DISABLE;
	DRV8884_enabled = 0;
}

uint8_t DRV8884_status(void) {
	return ((DRV8884_enabled << 1) | DRV8884_awake);
}


