#ifndef __DRV8884_H
#define __DRV8884_H

#include "stm32f30xe.h"
#include "utils.h"
#include "SysTick.h"
#include "TIM16.h"

#define DRV8884_STEP_PORT	B
#define DRV8884_STEP_PIN	5

#define DRV8884_DIR_PORT	B
#define DRV8884_DIR_PIN		4

#define DRV8884_SLP_PORT	C
#define DRV8884_SLP_PIN		4

#define DRV8884_EN_PORT		B
#define DRV8884_EN_PIN		3

#define DRV8884_FLT_PORT	A
#define DRV8884_FLT_PIN		10

#define DRV8884_STEP_HI		SET_BITS(GPIO(DRV8884_STEP_PORT)->ODR, (0x01) << DRV8884_STEP_PIN)	// high
#define DRV8884_STEP_LO		CLR_BITS(GPIO(DRV8884_STEP_PORT)->ODR, (0x01) << DRV8884_STEP_PIN)	// low

#define DRV8884_DIR_CW		SET_BITS(GPIO(DRV8884_DIR_PORT)->ODR, (0x01) << DRV8884_DIR_PIN)	// clockwise
#define DRV8884_DIR_CCW		CLR_BITS(GPIO(DRV8884_DIR_PORT)->ODR, (0x01) << DRV8884_DIR_PIN)	// counter-clockwise

#define DRV8884_WAKE		SET_BITS(GPIO(DRV8884_SLP_PORT)->ODR, (0x01) << DRV8884_SLP_PIN)
#define DRV8884_SLEEP		CLR_BITS(GPIO(DRV8884_SLP_PORT)->ODR, (0x01) << DRV8884_SLP_PIN)

#define DRV8884_ENABLE		SET_BITS(GPIO(DRV8884_EN_PORT)->ODR, (0x01) << DRV8884_EN_PIN)
#define DRV8884_DISABLE		CLR_BITS(GPIO(DRV8884_EN_PORT)->ODR, (0x01) << DRV8884_EN_PIN)

#define DRV8884_NO_FAULT	1
#define DRV8884_FAULT		0

#define DRV8884_FLT_STATUS		(READ_BITS(DRV8884_FLT_PORT, (0x01) << DRV8884_FLT_PIN)) ? 1:0	// 0: fault; 1: normal

static uint8_t DRV8884_enabled = 0;	// 0: disabled; 1: enabled
static uint8_t DRV8884_awake = 0;	// 0: sleeping; 1: awake

void DRV8884_Init(void);
void DRV8884_step(void);
void DRV8884_sleep(void);
void DRV8884_wake(void);
void DRV8884_enable(void);
void DRV8884_disable(void);
uint8_t DRV8884_status(void);

#endif /* __DRV8884_H */

