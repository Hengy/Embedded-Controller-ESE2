#include "RTC.h"

void RTC_Init(void) {
	
	// LSE clock
	if ((RCC->APB1ENR & RCC_APB1ENR_PWREN) == 0) {
		
		SET_BITS(RCC->APB1ENR, RCC_APB1ENR_PWREN);
		
		(void) RCC->APB1ENR;
	}
	
	if ((PWR->CR & PWR_CR_DBP) == 0) {
		
		SET_BITS(PWR->CR, PWR_CR_DBP);
		
		while ((PWR->CR & PWR_CR_DBP) == 0);
	}
	
	CLR_BITS(RCC->BDCR, (RCC_BDCR_LSEON | RCC_BDCR_LSEBYP));
	
	SET_BITS(RCC->BDCR, RCC_BDCR_BDRST);
	CLR_BITS(RCC->BDCR, RCC_BDCR_BDRST);
	
	while ((RCC->BDCR & RCC_BDCR_LSERDY) == 0) {
		
		SET_BITS(RCC->BDCR, RCC_BDCR_LSEON);
		
	}
	
	CLR_BITS(RCC->BDCR, RCC_BDCR_RTCSEL);
	SET_BITS(RCC->BDCR, RCC_BDCR_RTCSEL_0);
	
	CLR_BITS(RCC->APB1ENR, RCC_APB1ENR_PWREN);
	
	// RTC
	SET_BITS(RCC->BDCR, RCC_BDCR_RTCEN);
	
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	
	SET_BITS(RTC->ISR, RTC_ISR_INIT);
	
	while((RTC->ISR & RTC_ISR_INITF) == 0);
	
	CLR_BITS(RTC->CR, RTC_CR_FMT);
	
	RTC->PRER = ((200) << 16) | 520;
	
	//RTC->TR = 0U<<22 | 0U<<20 | 5U<<16 | 0U<<12 | 7U<<8;	// time AM/PM | H | H | M | M
	
	//RTC->DR = 1U<<22 | 9U<<20 | 0U<<16 | 4U<<12 | 1U<<8 | 2U;	// date Y | Y | M | M | D | D
	
	CLR_BITS(RTC->ISR, RTC_ISR_INIT);
	
	RTC->WPR = 0xFF;
}

uint32_t RTC_getTime(void) {
	
}


