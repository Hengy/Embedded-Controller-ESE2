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
#include <string.h>

#include "system_stm32f3xx.h"
#include "stm32f303xe.h"
#include "SysClock.h"
#include "SysTick.h"

#include "LEDs.h"
#include "LCD.h"
#include "DRV8884.h"
#include "lim_sw.h"
#include "servo.h"
#include "DRV8814.h"
#include "encoders.h"
#include "TIM15.h"
#include "serial.h"
#include "RTC.h"

#define	HEARTBEAT_TIMER_VAL			3000000		// heartbeat LED timer
#define	DIR_TIMER_VAL				8000000	 	// stepper direction timer
#define ENC_SW_ITER					10			// encoder switch iterations

// stepper global variables
volatile static uint16_t step_num = 0;
volatile uint8_t step_fault = DRV8884_NO_FAULT;
volatile uint8_t step_fault_changed = 0;

// DC global variables
volatile uint8_t dc_fault = DRV8814_NO_FAULT;
volatile uint8_t dc_fault_changed = 0;

// limit switch global variables
volatile static uint16_t Step_RLim = 0;
volatile static uint16_t Step_LLim = 0;

// encoder global variables
volatile static uint16_t speed_R = 0;
volatile static uint16_t speed_L = 0;
volatile static uint8_t enc_update = 0;	// Encoder update flag

// LCD & Serial update flag
volatile static uint8_t update_flag = 0;
volatile static uint8_t send_update = 10;

// Serial UART global variables
volatile static uint8_t SerialRX_Buf[64];
volatile static uint8_t SerialRX_Buf_index = 0;
volatile static uint8_t SerialRX_cmd[64];


void TIM16_IRQHandler(void) {
	if ((TIM16->SR & TIM_SR_UIF) != 0) {
		if (step_num > 0) {
			step_num--;
									
			DRV8884_STEP_HI;
			uint32_t wait = 50;
			while (wait--);
			DRV8884_STEP_LO;
			
		}
		
		TIM16->SR &= ~TIM_SR_UIF;
	}
}

void TIM15_IRQHandler(void) {
	if ((TIM15->SR & TIM_SR_UIF) != 0) {
		
		update_flag = 1;
		
		send_update = 10;
		
		Step_LLim = 0;
		Step_RLim = 0;
		
		TIM15->SR &= ~TIM_SR_UIF;
	}
}

// R Limit switch interrupt handler
void EXTI2_TSC_IRQHandler(void) {

	Step_LLim = 1;
	
	SET_BITS(EXTI->PR, EXTI_PR_PIF2);	// clear EXTI2 flag
}

// L Limit switch interrupt handler
void EXTI1_IRQHandler(void) {

	Step_RLim = 1;
	
	SET_BITS(EXTI->PR, EXTI_PR_PIF1);	// clear EXTI1 flag
}

// Stepper motor driver fault pin
void EXTI3_IRQHandler(void) {
	if (!DRV8884_FLT_STATUS) {	// fault
		//DRV8884_disable();
		step_num = 0;
		step_fault = DRV8884_FAULT;
	} else {		// fault cleared
		DRV8884_enable();
		step_fault = DRV8884_NO_FAULT;
	}
	
	step_fault_changed = 1;
	
	SET_BITS(EXTI->PR, EXTI_PR_PIF3);	// clear EXTI3 flag
}

// DC motor driver fault pin
void EXTI4_IRQHandler(void) {
	if (!DRV8814_FLT_STATUS) {	// fault
		DRV8814_disable();
		dc_fault = DRV8814_FAULT;
	} else {		// fault cleared
		DRV8814_enable();
		dc_fault = DRV8814_NO_FAULT;
	}
	
	dc_fault_changed = 1;
	
	SET_BITS(EXTI->PR, EXTI_PR_PIF4);	// clear EXTI12 flag
}

// encoder interrupt
void TIM1_CC_IRQHandler(void) {
		
	if (TIM1->SR & TIM_SR_CC1IF) {
	
		speed_R = 1000000 / ((TIM1->CCR1*2)+1);	// save value and clear interrupt
		enc_update = 2;
		
		CLR_BITS(TIM1->CCER,TIM_CCER_CC1E);	// disable capture from counter channel 1 --default--
		SET_BITS(TIM1->CCER,TIM_CCER_CC2E);	// enable capture from counter channel 2
		
		CLR_BITS(TIM1->DIER,TIM_DIER_CC1IE);	// disable interrupt channel 1 --default--
		SET_BITS(TIM1->DIER,TIM_DIER_CC2IE);	// enable interrupt channel 2
		
		speed_L = 0;
	} 
	if (TIM1->SR & TIM_SR_CC2IF) {
		
		speed_L = 1000000 / ((TIM1->CCR2*2)+1);	// save value and clear interrupt
		enc_update = 1;
		
		SET_BITS(TIM1->CCER,TIM_CCER_CC1E);	// enable capture from counter channel 1 --default--
		CLR_BITS(TIM1->CCER,TIM_CCER_CC2E);	// disable capture from counter channel 2
		
		SET_BITS(TIM1->DIER,TIM_DIER_CC1IE);	// enable interrupt channel 1 --default--
		CLR_BITS(TIM1->DIER,TIM_DIER_CC2IE);	// disable interrupt channel 2
		
		speed_R = 0;
	}
	
	//CLR_BITS(TIM1->SR, TIM_SR_UIF);
	
}

// Serial UART Receive ISR
void USART2_IRQHandler(void) {
	SerialRX_Buf[SerialRX_Buf_index] = USART2->RDR;
	
	if (SerialRX_Buf[SerialRX_Buf_index] == 'X') {
		memcpy((void*)SerialRX_cmd, (void*)SerialRX_Buf, 64);
	}
		
	SerialRX_Buf_index++;
	if (SerialRX_Buf_index == 64) {
		SerialRX_Buf_index = 0;
	}
}

uint16_t CAM_HOME(void) {
	LCDsetCursorPosition(2, 0);
	LCDprintf("HOMING");
	delay_ms(5);
	
	DRV8884_DIR_CW;
	step_num = 1500;
	DRV8884_enable();
	DRV8884_wake();
	while (Step_RLim == 0);	// turn until right limit is hit
	
	uint16_t total_steps = 0;	// set steps count to 0
	
	DRV8884_DIR_CCW;
	DRV8884_enable();
	DRV8884_wake();
	step_num = 1500;
	while (Step_LLim == 0);
	total_steps = 1500 - step_num;
	step_num = 0;	//stop
	
	DRV8884_enable();
	DRV8884_wake();
	DRV8884_DIR_CW;
	step_num = total_steps >> 1;
	
	while (step_num);	// wait for stepper to get to middle
	
	LCDsetCursorPosition(2, 0);
	LCDprintf("      ");
	delay_ms(5);
	
	return total_steps;
}

int main(void){

	// system clock
	System_Clock_Init(); // Switch System Clock = 72MHz
	
	// systick
	SysTick_Init(SYSTICK_US_VAL);
	
	// LCD
	LCD_Init();
	LCDclear();

	// hearbeat LED
	Heartbeat_Init();
	LED_Init();
	uint32_t beat_timer = HEARTBEAT_TIMER_VAL;
	
	LCDprintf("Matt H - ESE Yr2");
	
	delay_ms(500);
	
	LCDclear();
	LCDsetCursorPosition(1, 0);
	LCDprintf("STEP");
	LCDsetCursorPosition(1, 14);
	LCDprintf("DC");
	
	// limit switches
	lim_sw_Init();
	
	// stepper motor
	uint32_t dir_count = DIR_TIMER_VAL;
	DRV8884_Init();
	DRV8884_sleep();
	DRV8884_DIR_CW;
	DRV8884_disable();
	step_fault = DRV8884_NO_FAULT;
	step_fault_changed = 0;
	
	// servo
	Servo_Init(SERVOFREQ, SERVO_FWD_DEG);
	uint8_t servo_angle = 0;
	
	// DRV8814
	DRV8814_Init();
	DRV8814_sleep();
	DRV8814_disable();
	dc_fault = DRV8814_NO_FAULT;
	dc_fault_changed = 0;
	short int dc_step = 1;
	uint16_t dc_speed = 107;
	
	dc_speed = 0;
	DRV8814_wake();
	DRV8814_enable();
	
	// encoders
	ENC_Init();
	TIM15_Init(8000000);
	
	// serial
	UART_Init(SERIAL_BAUD_DEFAULT);
	
	// RTC
	RTC_Init();
	
	delay_ms(5);
	
	TIM17_set(LCD_BL_PWMFREQ, 80);
	
	while(1){
		
		// heartbeat LED toggles
		beat_timer--;
		if ((beat_timer == (HEARTBEAT_TIMER_VAL/21)) || (beat_timer == (HEARTBEAT_TIMER_VAL/24)) || (beat_timer == (HEARTBEAT_TIMER_VAL/60))) {
			HEARTBEAT_TOGGLE;
		} else if (beat_timer == 0) {
			HEARTBEAT_TOGGLE;
			beat_timer = HEARTBEAT_TIMER_VAL;
		}
		
		if (update_flag == 1) {
			if (step_fault == DRV8884_FAULT) {
				LCDsetCursorPosition(2, 0);
				LCDprintf("FLT!");
				
				UART_printf("STP FLT!");
			} else {
				LCDsetCursorPosition(2, 0);
				LCDprintf("    ");
			}
			
			if (dc_fault == DRV8814_FAULT) {
				LCDsetCursorPosition(2, 11);
				LCDprintf("FLT!");
				
				UART_printf("STP FLT!");
			} else {
				if (enc_update == 2) {
					LCDsetCursorPosition(2, 10);
					if (speed_R < 100) {
						LCDprintf(" %uHz",speed_R);
					} else {
						LCDprintf("%uHz",speed_R);
					}
					enc_update = 0;
				}
				
			}
			update_flag = 0;
		}
	
		/*
		if (SerialRX_Buf != 0) {
			LCDsetCursorPosition(2, 0);
			LCDprintf("%c", (char)SerialRX_Buf);
			
			UART_printf("char: %c\n", (char)SerialRX_Buf);
			
			switch (SerialRX_Buf) {
				
				case 'S':	// Stepper
					step_num = 768;
					DRV8884_enable();
					DRV8884_wake();
					break;
				
				case 'D':	// DC motor
					dc_speed = 100;
					DRV8814_wake();
					DRV8814_enable();
					break;
				
				case 'A':	// Servo
					if (servo_angle > 0) {
						Servo_set(0);
						servo_angle = 40;
					} else {
						Servo_set(180);
						servo_angle = 60;
					}
					break;
					
				case 'H':	// Home
					// homing function for camera
					DRV8884_enable();
					DRV8884_wake();
					CAM_HOME();
					break;
					
				case 'U':	// Update
					UART_printf("SPD: %u\n", speed_R);
					break;
				
				case 'T':	// Time
					UART_printf("TIME: %u%u:%u%u:%u%u\n", (RTC->TR & RTC_TR_HT_Msk) >> RTC_TR_HT_Pos, (RTC->TR & RTC_TR_HU_Msk) >> RTC_TR_HU_Pos, (RTC->TR & RTC_TR_MNT_Msk) >> RTC_TR_MNT_Pos, (RTC->TR & RTC_TR_MNU_Msk) >> RTC_TR_MNU_Pos, (RTC->TR & RTC_TR_ST_Msk) >> RTC_TR_ST_Pos, (RTC->TR & RTC_TR_SU_Msk) >> RTC_TR_SU_Pos);
					break;
				
			}
			
			SerialRX_Buf = 0;
		}
		*/

//		if (step_fault_changed) {
//			if (step_fault == DRV8884_NO_FAULT) {
//				LCDsetCursorPosition(2, 0);
//				LCDprintf("Stepper: Normal ");
//			} else {
//				LCDsetCursorPosition(2, 0);
//				LCDprintf("Stepper: FAULT! ");
//			}	
//			step_fault_changed = 0;
//		}
		
		if (dc_fault_changed) {
			if (dc_fault == DRV8814_NO_FAULT) {
				LCDsetCursorPosition(1, 0);
				LCDprintf("DC: Normal ");
			} else {
				LCDsetCursorPosition(1, 0);
				LCDprintf("DC: FAULT! ");
			}	
			dc_fault_changed = 0;
		}

		// DC direction & speed
		if (dc_speed <= 105) {
			dc_speed += dc_step;
			
			if (dc_fault == DRV8814_NO_FAULT) {
				
				if (dc_speed > 100) {
					if ((speed_R > 15) && (speed_R < 105)) {
						DRV8814_set_speed_left(speed_R);
						DRV8814_set_speed_right(speed_R);
					}
				} else {
					DRV8814_set_speed_left(dc_speed);
					DRV8814_set_speed_right(dc_speed);
				}
				
			}
			delay_ms(100);
		}
		if (dc_speed == 105) {
			dc_speed = 100;
		}
	}
}
