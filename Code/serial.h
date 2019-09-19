#ifndef __UART_H
#define __UART_H

#include "stm32f30xe.h"
#include "utils.h"
#include <stdio.h>
#include <stdarg.h>

// Pins
#define SERIAL_PORT		A
#define SERIAL_TX		2
#define SERIAL_RX		3
#define SERIAL_RTS		1
#define SERIAL_CTS		0

#define SERIAL_BAUD_DEFAULT		115200

#define SERIAL_MAX_BUFSZ		32

void UART_Init(uint32_t baud);

void UART_putc(char c);

void UART_puts(char *buf);

void UART_printf(char *fmt,...);

#endif /* __UART_H */
