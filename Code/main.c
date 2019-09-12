/* ------------------------------------------
// 	Name:		Matthew Hengeveld
//	ID:			8534331
//	Date:		1/17/2019
//
//	LEDs:
//		Green			PA5
//		HEARBEAT		PD2
//
//	-----------------------------------------*/

#include "utils.h"

#include "system_stm32f3xx.h"
#include "stm32f303xe.h"
#include "SysClock.h"
#include "SysTick.h"

#include "LEDs.h"

int main(void){

	// system clock
	System_Clock_Init(); // Switch System Clock = 72MHz
	
	// systick
	SysTick_Init(SYSTICK_US_VAL);

	LED_Init();
	
	while(1){
		
		LED_TOGGLE;
		delay_ms(500);
		
	}
	
}
