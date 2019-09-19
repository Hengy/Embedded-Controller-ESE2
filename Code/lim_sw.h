#ifndef __LIM_SW_H
#define __LIM_SW_H

#include "stm32f30xe.h"
#include "utils.h"

#define LIM_PORT		B
#define LIML_PIN		1
#define LIMR_PIN		2

#define GET_LIML		READ_BITS(LIM_PORT, (0x01) << LIML_PIN)
#define GET_LIMR		READ_BITS(LIM_PORT, (0x01) << LIMR_PIN)

void lim_sw_Init(void);

#endif /* __LIM_SW_H */

