#ifndef __ENCODERS_H
#define __ENCODERS_H

// ------------ USES TIMER 1 ------------ 

#include "stm32f30xe.h"
#include "utils.h"

#define ENC_PORT		A
#define ENC_L_PIN		8
#define ENC_R_PIN		9

void ENC_Init(void);

#endif /* __ENCODERS_H */

