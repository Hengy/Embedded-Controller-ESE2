#ifndef __LCD_H
#define __LCD_H

#include "HD44780.h"

#include "stm32f303xe.h"
#include "utils.h"
#include "SysTick.h"
#include <stdio.h>
#include <stdarg.h>
#include "TIM17.h"

/* --------------------------------------
/
/	LCD MACROS & SYMBOLS
/
/---------------------------------------*/

#define LCD_E_PORT			B
#define LCD_RS_PORT			A
#define LCD_BITS_PORT		C

// LCD pins
#define	LCD_E				0
#define LCD_RS				4
#define LCD_BITS			0x000F
#define LCD_BIT0			0
#define LCD_BIT1			1
#define LCD_BIT2			2
#define LCD_BIT3			3

#define LCD_E_HI			SET_BITS(GPIO(LCD_E_PORT)->ODR, (0x01) << LCD_E)	// set high
#define LCD_E_LO			CLR_BITS(GPIO(LCD_E_PORT)->ODR, (0x01) << LCD_E)	// set low

#define LCD_RS_D			SET_BITS(GPIO(LCD_RS_PORT)->ODR, (0x01) << LCD_RS)	// set high
#define LCD_RS_I			CLR_BITS(GPIO(LCD_RS_PORT)->ODR, (0x01) << LCD_RS)	// set low

#define LCD_BUS(data)		FORCE_BITS(GPIO(LCD_BITS_PORT)->ODR, LCD_BITS, (data))

#define LCD_MAX_BUFSZ		17

#define LCD_WAIT			80

void LCD_Init(void);
void LCDwriteData(uint8_t);
void LCDwriteCMD(uint8_t);
void LCDputc(char c);
void LCDputs(char *buf);
void LCDprintf(char *fmt,...);
void LCDsetCursorPosition(uint8_t row, uint8_t pos);
void LCDclear(void);

#endif /* __LCD_H */

