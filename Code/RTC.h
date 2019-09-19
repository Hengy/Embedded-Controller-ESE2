#ifndef __RTC_H
#define __RTC_H

#include "stm32f30xe.h"
#include "utils.h"


void RTC_Init(void);
uint32_t RTC_getTime(void);

#endif /* __RTC_H */

