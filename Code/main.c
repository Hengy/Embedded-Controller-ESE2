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

#define	HEARTBEAT_TIMER_VAL			300000			// heartbeat LED timer
#define ENC_SW_ITER							10					// encoder switch iterations
#define SERVODEFAULT						1600				// default servo position

// stepper global variables
volatile static uint16_t step_num = 0;
volatile uint8_t step_fault = DRV8884_NO_FAULT;
volatile uint8_t step_fault_changed = 0;
uint16_t total_LR_steps = 0;	// total steps form left limit switch to right limit switch
uint16_t current_LR_steps = 0;
volatile static uint8_t stepper_dis_cnt = 0;

// servo global variables
volatile static uint16_t servo_angle_curr = SERVODEFAULT - 100;
volatile static uint16_t servo_angle = SERVODEFAULT;

// DC global variables
volatile uint8_t dc_fault = DRV8814_NO_FAULT;
volatile uint8_t dc_fault_changed = 0;
volatile uint8_t dc_speedL = 0;
volatile uint8_t dc_speedR = 0;
volatile uint8_t dc_dirL = 1;	// 0 - backwards; 1 - forwards
volatile uint8_t dc_dirR = 1;

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
#define STATUSMSGLEN		13

// Serial UART global variables
volatile static uint8_t SerialRX_Buf[64];
volatile static uint8_t SerialRX_Buf_index = 0;
volatile static uint8_t SerialRX_cmd[64];
volatile static uint8_t cmd_recv = 0;
volatile static uint8_t cmd_recv_index = 0;

void send_status(void);

void TIM16_IRQHandler(void) {
	if ((TIM16->SR & TIM_SR_UIF) != 0) {
		if (step_num > 0) {
			step_num--;
									
			DRV8884_STEP_HI;
			uint32_t wait = 50;
			while (wait--);
			DRV8884_STEP_LO;
			
		}
		
		if (servo_angle > servo_angle_curr) {
			if ((servo_angle - servo_angle_curr) < 2) {
				servo_angle_curr = servo_angle;
			} else {
				servo_angle_curr += 2;
			}
			Servo_set_us(servo_angle_curr);
		} else if (servo_angle < servo_angle_curr) {
			if ((servo_angle_curr - servo_angle) < 2) {
				servo_angle_curr = servo_angle;
			} else {
				servo_angle_curr -= 2;
			}
			Servo_set_us(servo_angle_curr);
		}
		
		TIM16->SR &= ~TIM_SR_UIF;
	}
}

void TIM15_IRQHandler(void) {
	if ((TIM15->SR & TIM_SR_UIF) != 0) {
		
		update_flag = 1;
		
		send_update--;
		
		Step_LLim = 0;
		Step_RLim = 0;
		
		if (step_num == 0) {
			stepper_dis_cnt--;
			if (stepper_dis_cnt < 1) {
				stepper_dis_cnt = 1;
			}
			if (stepper_dis_cnt == 2) {
				DRV8884_disable();
			}
		} else {
			stepper_dis_cnt = 30;
		}
		
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
		
		cmd_recv = 1;
		if (SerialRX_Buf_index > 0) {
			cmd_recv_index = SerialRX_Buf_index - 1;
		} else {
			cmd_recv_index = 63;
		}
;
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
	
	DRV8884_disable();
	
	// done homing stepper
	
	/// home servo
	Servo_set_us(servo_angle);
	
	LCDsetCursorPosition(2, 0);
	LCDprintf("      ");
	delay_ms(5);
	
	return total_steps;
}

void send_status(void) {
	char status_msg[STATUSMSGLEN];
	
	status_msg[0] = 'X';
	status_msg[1] = 'S';
	status_msg[2] = 'T';
	status_msg[3] = servo_angle;
	status_msg[4] = LO_NIBBLE(current_LR_steps);
	status_msg[5] = HI_NIBBLE(current_LR_steps);
	status_msg[6] = speed_R;
	status_msg[7] = dc_speedR;
	status_msg[8] = dc_dirR;
	status_msg[9] = speed_L;
	status_msg[10] = dc_speedL;
	status_msg[11] = dc_dirL;
	status_msg[12] = '\n';
	
	UART_puts(status_msg);
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
	DRV8884_Init();
	DRV8884_sleep();
	DRV8884_DIR_CW;
	DRV8884_disable();
	step_fault = DRV8884_NO_FAULT;
	step_fault_changed = 0;
	
	// servo
	Servo_Init(SERVOFREQ, SERVO_FWD_DEG);
	Servo_set(servo_angle);
	
	// DRV8814
	DRV8814_Init();
	DRV8814_sleep();
	DRV8814_disable();
	dc_fault = DRV8814_NO_FAULT;
	dc_fault_changed = 0;
	//short int dc_step = 1;
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
	
	total_LR_steps = CAM_HOME();
	
	while(1){
		
		// heartbeat LED toggles
		beat_timer--;
		if ((beat_timer == (HEARTBEAT_TIMER_VAL/21)) || (beat_timer == (HEARTBEAT_TIMER_VAL/24)) || (beat_timer == (HEARTBEAT_TIMER_VAL/60))) {
			HEARTBEAT_TOGGLE;
		} else if (beat_timer == 0) {
			HEARTBEAT_TOGGLE;
			beat_timer = HEARTBEAT_TIMER_VAL;
		}
		
		if (send_update < 3) {
			send_status();
			send_update = 53;
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
		
		if (cmd_recv) {
			uint8_t cmd_recv_index_next;
			if (cmd_recv_index_next > 0) {
			cmd_recv_index_next = cmd_recv_index - 1;
		} else {
			cmd_recv_index_next = 63;
		}
			
			if (SerialRX_cmd[cmd_recv_index] == 'C') {
				switch (SerialRX_cmd[cmd_recv_index_next]) {
					
					case 'S':	//camera straight
						TIM16_Init(STEP_PWMFREQFAST);
					
						if (current_LR_steps > (total_LR_steps >> 1)) {
							DRV8884_DIR_CW;
						} else {
							DRV8884_DIR_CCW;
						}
						step_num = 1500;
						DRV8884_enable();
						DRV8884_wake();
						while ((Step_RLim == 0) && (Step_LLim == 0));	// turn until right limit is hit
					
						if (current_LR_steps > (total_LR_steps >> 1)) {
							DRV8884_DIR_CCW;
						} else {
							DRV8884_DIR_CW;
						}
						DRV8884_enable();
						DRV8884_wake();
						step_num = total_LR_steps >> 1;
						//while (step_num);
						
						servo_angle = SERVODEFAULT;
					
						break;
					
					case 'L':	//camera left
						TIM16_Init(STEP_PWMFREQMED);	
					
						if ((current_LR_steps - 40) > 0) {
							DRV8884_DIR_CCW;
							step_num = 40;
							DRV8884_enable();
							DRV8884_wake();
							current_LR_steps-= 40;
						}

						break;
					
					case 'R':	//camera right
						TIM16_Init(STEP_PWMFREQMED);	
					
						if ((current_LR_steps + 40) < total_LR_steps) {
							DRV8884_DIR_CW;
							step_num = 40;
							DRV8884_enable();
							DRV8884_wake();
							current_LR_steps+= 40;
						}
					
						break;
					
					case 'U':	//camera up
						
						servo_angle -= 60;
						if (servo_angle < SERVOMINDEG) {
							servo_angle = SERVOMINDEG;
						}
						//Servo_set(servo_angle);
						
						break;
					
					case 'D':	//camera down
						
						servo_angle += 60;
						if (servo_angle > SERVOMAXDEG) {
							servo_angle = SERVOMAXDEG;
						}
						//Servo_set(servo_angle);
					
						break;
				}
			} else if (SerialRX_cmd[cmd_recv_index] == 'H') {
				switch (SerialRX_cmd[cmd_recv_index_next]) {
					
					case 'L':	//hard left
						
						break;
					
					case 'R':	//hard right
						
						break;
					
					case 'M':	//home
						TIM16_Init(STEP_PWMFREQFAST);
						total_LR_steps = CAM_HOME();
						current_LR_steps = total_LR_steps >> 1;
						break;
				}
			} else if (SerialRX_cmd[cmd_recv_index] == 'R') {
				switch (SerialRX_cmd[cmd_recv_index_next]) {
					
					case 'S':	//robot stop
						
						dc_speedL = 0;
						dc_speedR = 0;
					
						DRV8814_set_speed_left(dc_speedL);
						DRV8814_set_speed_right(dc_speedR);
					
						break;
					
					case 'M':	//robot move
						
						if (SerialRX_cmd[cmd_recv_index-3] == 'F') {	// forward
							DRV8814_DIRL_FWD;
							dc_dirL = 1;
						} else {
							DRV8814_DIRL_BKWD;
							dc_dirL = 0;
						}
						dc_speedL = SerialRX_cmd[cmd_recv_index-2];
						if (dc_speedL > 99) {
							dc_speedL = 107;
						}
						
						DRV8814_set_speed_left(dc_speedL);
						
						if (SerialRX_cmd[cmd_recv_index-5] == 'F') {	// forward
							DRV8814_DIRR_FWD;
							dc_dirR = 1;
						} else {
							DRV8814_DIRR_BKWD;
							dc_dirR = 0;
						}
						dc_speedR = SerialRX_cmd[cmd_recv_index-4];
						if (dc_speedR > 99) {
							dc_speedR = 107;
						}
						
						DRV8814_set_speed_right(dc_speedR);

						break;
				}
			} else if (SerialRX_cmd[cmd_recv_index] == 'O') {
				if (SerialRX_cmd[cmd_recv_index_next] == 'M') {	// on screen message
					char msg[16];
					uint8_t msg_i = 0;
					uint8_t msg_len = SerialRX_cmd[cmd_recv_index-2] - 16;
					
					if ((cmd_recv_index-2) > 15) {
						for (uint8_t i = cmd_recv_index-17; i < cmd_recv_index-2; i++) {
							msg[msg_i++] = SerialRX_cmd[i];
						}
					}
					
					LCDsetCursorPosition(2, 0);
					LCDputs(msg);
				}
			} else if (SerialRX_cmd[cmd_recv_index] == 'S') {
				if (SerialRX_cmd[cmd_recv_index_next] == 'T') {	// status
					send_status();
				}
			}
			
			cmd_recv = 0;
		}

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
	}
}
