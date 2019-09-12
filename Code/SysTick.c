#include "SysTick.h"

void SysTick_Init(uint32_t ticks) {

    SysTick->CTRL = 0;            // Disable SysTick

    SysTick->LOAD = ticks - 1;    // Set reload register

    // Set interrupt priority of SysTick to least urgency (i.e., largest priority value)
    NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);

    SysTick->VAL = 0;             // Reset the SysTick counter value

    // Select processor clock: 1 = processor clock; 0 = external clock
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    // Enables SysTick interrupt, 1 = Enable, 0 = Disable
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    // Enable SysTick
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

static volatile int32_t TimeDelay;

void SysTick_Handler (void) { // SysTick interrupt service routine 
	if (TimeDelay > 0)    // Prevent it from being negative 
		TimeDelay--;        // TimeDelay is a global volatile variable 
} 	

void delay_us (uint32_t n) {  // n: specifies the delay time length
	// optionally restart the SysTick timer (if stopping when function done)
	SysTick->VAL = 0;       // Force reset of countdown timer
	TimeDelay = ((int)(n/SYSTICK_PERIOD)) + 1;      // TimeDelay must be declared as volatile
	while(TimeDelay != 0);  // Busy wait
	// optionally stop the SysTick timer until needed again when Delay called
}

void delay_ms (uint32_t n) { 
	for (int i=0; i<n; i++) {	// loop n*1000us
		// optionally restart the SysTick timer (if stopping when function done)
		SysTick->VAL = 0;       // Force reset of countdown timer
		TimeDelay = 50;      // TimeDelay must be declared as volatile
		while(TimeDelay != 0);  // Busy wait
		// optionally stop the SysTick timer until needed again when Delay called
	}
}
