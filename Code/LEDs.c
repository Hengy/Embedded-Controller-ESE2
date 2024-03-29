#include "LEDs.h"

void LED_Init(void) {
	
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  // Enable clock of Port A
	
	GPIOA->MODER &= ~(3UL << (LED_PIN*2)); // Clear mode bits
	GPIOA->MODER |=   1UL << (LED_PIN*2);  // Set mode to output
	
	GPIOA->OTYPER &= ~(1UL << LED_PIN); // Select push-pull output
}

void Heartbeat_Init(void) {
	
	RCC->AHBENR |= RCC_AHBENR_GPIODEN;  // Enable clock of Port D
	
	GPIOD->MODER &= ~(3UL << (HEARTBEAT_PIN*2)); // Clear mode bits
	GPIOD->MODER |=   1UL << (HEARTBEAT_PIN*2);  // Set mode to output
	
	GPIOD->OTYPER &= ~(1UL << HEARTBEAT_PIN); // Select push-pull output
}
