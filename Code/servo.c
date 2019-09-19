#include "servo.h"

void Servo_Init(uint16_t freq, uint16_t degree) {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;		// enable PORT A clocks
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;		// enable TIM2 clocks
	
	GPIO_PIN_MODE(SERVOPORT,SERVOPIN,AF);					// set PE8 as AF
	
	GPIO_PIN_AF_MODE(SERVOPORT,SERVOPIN,0x01);				// set AF mode
	
	GPIO_PIN_DRV_TYPE(SERVOPORT,SERVOPIN,PP);
	
	CLR_BITS(TIM2->CR1, 0x10);					// clear DIR bit (up-counting)
	
	TIM2->PSC = 719;	// 100KHz counter frequency
	
	TIM2->ARR = freq;	// servo signal frequency
	
	FORCE_BITS(TIM2->CCMR1,0x00F0,0x60);	// PWM mode 1
	
	SET_BITS(TIM2->CR1,0x80);			// auto-reload preload value
	
	FORCE_BITS(TIM2->CCER,(TIM_CCER_CC1NP | TIM_CCER_CC1P | TIM_CCER_CC1E),(TIM_CCER_CC1NP | TIM_CCER_CC1P | TIM_CCER_CC1E));	// complementary output, polarity bit, compare enable 

	int duty = Servo_UsToVal(Servo_degreeToUs(degree));
	TIM2->CCR1 = duty;	// Set duty cycle
	
	SET_BITS(TIM2->CR1,0x01);					// enable counter
}

void Servo_set(uint16_t degree) {
	int val = Servo_UsToVal(Servo_degreeToUs(degree));
	if (val != 0) {
		TIM2->CCR1 = val;	// Set duty cycle
	}
}

void Servo_enable() {
	SET_BITS(TIM2->BDTR,0x8000);			// Enable master output
}

void Servo_disable() {
	CLR_BITS(TIM2->BDTR,0x8000);			// disable master output
}

uint16_t Servo_degreeToUs(uint16_t degree) {
	
	return ((degree*10)+600);
}

uint16_t Servo_UsToVal(uint16_t us) {
	
	uint16_t val = SERVOFREQ-((us*SERVOFREQ)/20100);
	
	if ((val >= SERVOMAX) && (val <= SERVOMIN)) {
		return val;
	} else {
		return 0;
	}
}


