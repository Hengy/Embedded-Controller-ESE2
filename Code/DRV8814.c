#include "DRV8814.h"

void DRV8814_Init(void) {
	
	// enable port clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;	// Enable clock of Port C
	
	// pin modes
	GPIO_PIN_MODE(DRV8814_EN_PORT, DRV8814_ENA_PIN, AF);
	GPIO_PIN_MODE(DRV8814_EN_PORT, DRV8814_ENB_PIN, AF);
	
	GPIO_PIN_AF_MODE(DRV8814_EN_PORT, DRV8814_ENA_PIN, 0x02);
	GPIO_PIN_AF_MODE(DRV8814_EN_PORT, DRV8814_ENB_PIN, 0x02);
	
	GPIO_PIN_MODE(DRV8814_PHS_PORT, DRV8814_PHSA_PIN, OUT);
	GPIO_PIN_MODE(DRV8814_PHS_PORT, DRV8814_PHSB_PIN, OUT);
	
	GPIO_PIN_MODE(DRV8814_RST_PORT, DRV8814_RST_PIN, OUT);
	GPIO_PIN_MODE(DRV8814_SLP_PORT, DRV8814_SLP_PIN, OUT);
	
	// pin drive types
	GPIO_PIN_DRV_TYPE(DRV8814_EN_PORT, DRV8814_ENA_PIN, PP);
	GPIO_PIN_DRV_TYPE(DRV8814_EN_PORT, DRV8814_ENB_PIN, PP);
	
	GPIO_PIN_DRV_TYPE(DRV8814_PHS_PORT, DRV8814_PHSA_PIN, PP);
	GPIO_PIN_DRV_TYPE(DRV8814_PHS_PORT, DRV8814_PHSB_PIN, PP);
	
	GPIO_PIN_DRV_TYPE(DRV8814_RST_PORT, DRV8814_RST_PIN, PP);
	GPIO_PIN_DRV_TYPE(DRV8814_SLP_PORT, DRV8814_SLP_PIN, PP);
	
	// fault
	GPIO_PIN_MODE(DRV8814_FLT_PORT, DRV8814_FLT_PIN, IN);	// input
	
	// SYSCFG clock
	SET_BITS(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
	
	// external interrupt
	SET_BITS(SYSCFG->EXTICR[3], SYSCFG_EXTICR4_EXTI12_PC);	// PC12 external interrupt source
	//SYSCFG->EXTICR[3] = 0x02;
	
	SET_BITS(EXTI->FTSR, EXTI_FTSR_FT12);	// Falling edge trigger for EXTI4
	
	SET_BITS(EXTI->RTSR, EXTI_RTSR_RT12);	// Rising edge trigger for EXTI4
	
	SET_BITS(EXTI->IMR, EXTI_IMR_IM12);		// Enable interrupt
	
	NVIC_EnableIRQ(EXTI4_IRQn);		// Enable NVIC IRQ
	
	// TIM3
	TIM3_Init(DRV8814_DC_FREQ);
//	TIM3_disable_ch3();
//	TIM3_disable_ch4();
	
	DRV8814_NORM;
	DRV8814_DIRA_FWD;
	DRV8814_DIRB_FWD;
}

void DRV8814_set_speed_left(uint8_t speed) {
	uint16_t duty = 0;
	if (speed != 0) {
		duty = ((speed*DRV8814_DC_FREQ) / 100) + 1;
	}
	TIM3_set_ch3(duty);
}

void DRV8814_set_speed_right(uint8_t speed) {
	uint16_t duty = 0;
	if (speed != 0) {
		duty = ((speed*DRV8814_DC_FREQ) / 100) + 1;
	}
	TIM3_set_ch4(duty);
}

void DRV8814_sleep(void) {
	DRV8814_SLEEP;
	DRV8814_awake = 0;
}

void DRV8814_wake(void) {
	DRV8814_WAKE;
	DRV8814_awake = 1;
}

void DRV8814_enable() {
	TIM3_enable_ch3();
	TIM3_enable_ch4();
	DRV8814_enabledA = 1;
	DRV8814_enabledB = 1;
}

void DRV8814_disable() {
	TIM3_disable_ch3();
	TIM3_disable_ch4();
	DRV8814_enabledA = 0;
	DRV8814_enabledB = 0;
}

uint8_t DRV8814_status(void) {
	return ((DRV8814_enabledA << 2) | (DRV8814_enabledB << 1) | DRV8814_awake);
}


