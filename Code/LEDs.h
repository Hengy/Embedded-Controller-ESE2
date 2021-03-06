#ifndef __LEDS_H
#define __LEDS_H

#include "stm32f303xe.h"

#define LED_PORT	A
#define LED_PIN		5

#define	HEARTBEAT_PORT	D
#define HEARTBEAT_PIN	2

#define LED_HI				SET_BITS(GPIO(LED_PORT)->ODR, (0x01) << LED_PIN)
#define LED_LO				CLR_BITS(GPIO(LED_PORT)->ODR, (0x01) << LED_PIN)	
#define LED_TOGGLE			FLIP_BITS(GPIO(LED_PORT)->ODR, (0x01) << LED_PIN)

#define HEARTBEAT_HI		SET_BITS(GPIO(HEARTBEAT_PORT)->ODR, (0x01) << HEARTBEAT_PIN)
#define HEARTBEAT_LO		CLR_BITS(GPIO(HEARTBEAT_PORT)->ODR, (0x01) << HEARTBEAT_PIN)	
#define HEARTBEAT_TOGGLE	FLIP_BITS(GPIO(HEARTBEAT_PORT)->ODR, (0x01) << HEARTBEAT_PIN)

void LED_Init(void);

void Heartbeat_Init(void);

#endif /* __LEDS_H */

