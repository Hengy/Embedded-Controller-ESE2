#ifndef __UTILS_H
#define __UTILS_H

#include "stm32f30xe.h"

/* --------------------------------------
/
/	SYMBOLIC NAMES
/
/---------------------------------------*/

// drive type
#define PP				0		// push-pull
#define OD				1		// open-drain

// mode type
#define	IN				0		// input
#define OUT				1		// output
#define AF				2		// alternate function
#define ANALOG			3		// analog in

/* --------------------------------------
/
/	MACROS
/
/---------------------------------------*/

// port macro
#define GPIO(port)						GPIOx(port)
#define GPIOx(port)						GPIO ## port

// Bit manipulation macros
#define SET_BITS(port,bits)				((port) |= (bits))
#define	CLR_BITS(port,bits)				((port) &= (~(bits)))
#define FLIP_BITS(port,bits)			((port) ^= (bits))
#define FORCE_BITS(port,mask,value)		( (port) = ((port) & (~(mask))) | ((value) & (mask)) )

#define LO_NIBBLE(data)					((data) & 0x0F)
#define HI_NIBBLE(data)					(((data) >> 4) & 0x0F)

#define READ_BITS(port, bits)			((GPIO(port)->IDR) & (bits))

// GPIO setup macros
#define GPIO_PIN_DRV_TYPE(port,pin,type)			FORCE_BITS(GPIO(port)->OTYPER, 1 << (pin), (type) << (pin))
#define GPIO_PIN_MODE(port,pin,mode)				FORCE_BITS(GPIO(port)->MODER, 3 << ((pin) * 2), (mode) << ((pin) * 2) )

#define GPIO_PIN_AF_MODE(port,pin,mode)				if (((pin) / 8) > 0) { \
														FORCE_BITS(GPIO(port)->AFR[1], 0x0F << ((pin % 8) * 4), (mode) << ((pin % 8) * 4) ); \
													} else { \
														FORCE_BITS(GPIO(port)->AFR[0], 0x0F << ((pin) * 4), (mode) << ((pin) * 4) ); }

#endif /* __UTILS_H */

