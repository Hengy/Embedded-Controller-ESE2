#ifndef __DRV8814_H
#define __DRV8814_H

#include "stm32f30xe.h"
#include "utils.h"
#include "SysTick.h"
#include "TIM3.h"

#define DRV8814_EN_PORT		C
#define DRV8814_ENA_PIN		8
#define DRV8814_ENB_PIN		9

#define DRV8814_PHS_PORT	C
#define DRV8814_PHSA_PIN	5
#define DRV8814_PHSB_PIN	6

#define DRV8814_RST_PORT	C
#define DRV8814_RST_PIN		10

#define DRV8814_SLP_PORT	C
#define DRV8814_SLP_PIN		11

#define DRV8814_FLT_PORT	C
#define DRV8814_FLT_PIN		12

#define DRV8814_DIRA_BKWD	SET_BITS(GPIO(DRV8814_PHS_PORT)->ODR, (0x01) << DRV8814_PHSA_PIN)	// forward
#define DRV8814_DIRA_FWD	CLR_BITS(GPIO(DRV8814_PHS_PORT)->ODR, (0x01) << DRV8814_PHSA_PIN)	// backward

#define DRV8814_DIRB_BKWD	SET_BITS(GPIO(DRV8814_PHS_PORT)->ODR, (0x01) << DRV8814_PHSB_PIN)	// forward
#define DRV8814_DIRB_FWD	CLR_BITS(GPIO(DRV8814_PHS_PORT)->ODR, (0x01) << DRV8814_PHSB_PIN)	// backward

#define DRV8814_DIRL_FWD	DRV8814_DIRA_FWD
#define DRV8814_DIRL_BKWD	DRV8814_DIRA_BKWD

#define DRV8814_DIRR_FWD	DRV8814_DIRB_FWD
#define DRV8814_DIRR_BKWD	DRV8814_DIRB_BKWD

#define DRV8814_WAKE		SET_BITS(GPIO(DRV8814_SLP_PORT)->ODR, (0x01) << DRV8814_SLP_PIN)
#define DRV8814_SLEEP		CLR_BITS(GPIO(DRV8814_SLP_PORT)->ODR, (0x01) << DRV8814_SLP_PIN)

#define DRV8814_RST			CLR_BITS(GPIO(DRV8814_RST_PORT)->ODR, (0x01) << DRV8814_RST_PIN)
#define DRV8814_NORM		SET_BITS(GPIO(DRV8814_RST_PORT)->ODR, (0x01) << DRV8814_RST_PIN)

#define DRV8814_NO_FAULT	1
#define DRV8814_FAULT		0

#define DRV8814_FLT_STATUS		(READ_BITS(DRV8814_FLT_PORT, (0x01) << DRV8814_FLT_PIN)) ? 1:0	// 0: fault; 1: normal

static uint8_t DRV8814_enabledA = 0;	// Motor A 0: disabled; 1: enabled
static uint8_t DRV8814_enabledB = 0;	// Motor B 0: disabled; 1: enabled
static uint8_t DRV8814_awake = 0;	// 0: sleeping; 1: awake

void DRV8814_Init(void);
void DRV8814_set_speed_left(uint8_t speed);
void DRV8814_set_speed_right(uint8_t speed);
void DRV8814_sleep(void);
void DRV8814_wake(void);
void DRV8814_enable(void);
void DRV8814_disable(void);
uint8_t DRV8814_status(void);

#endif /* __DRV8814_H */

