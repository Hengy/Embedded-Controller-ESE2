#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f303xe.h"
#include "utils.h"

#define SERVOPORT		A
#define SERVOPIN		15

// servo signal frequency
#define SERVOFREQ		1999	// 50Hz

// max and min servo values
#define SERVOMAX	 	1750
#define SERVOMIN 		1940

#define SERVOMAXDEG	170
#define SERVOMINDEG 55

// centre position
#define SERVO_FWD_DEG 		60

void Servo_Init(uint16_t freq, uint16_t duty);
void Servo_set(uint16_t duty);
void Servo_enable(void);
void Servo_disable(void);
uint16_t Servo_degreeToUs(uint16_t degree);
uint16_t Servo_UsToVal(uint16_t us);

#endif /* __SERVO_H */
