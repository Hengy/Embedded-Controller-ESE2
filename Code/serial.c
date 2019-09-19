#include "serial.h"

void UART_Init(uint32_t baud) {
	
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;	// enable PORT A clocks
	
	GPIO_PIN_MODE(SERIAL_PORT,SERIAL_TX,AF);					// set as AF	
	GPIO_PIN_AF_MODE(SERIAL_PORT,SERIAL_TX,0x07);				// set AF mode
	
	GPIO_PIN_MODE(SERIAL_PORT,SERIAL_RX,AF);					// set as AF	
	GPIO_PIN_AF_MODE(SERIAL_PORT,SERIAL_RX,0x07);				// set AF mode
	
	GPIO_PIN_MODE(SERIAL_PORT,SERIAL_RTS,AF);					// set as AF	
	GPIO_PIN_AF_MODE(SERIAL_PORT,SERIAL_RTS,0x07);				// set AF mode
	
	GPIO_PIN_MODE(SERIAL_PORT,SERIAL_CTS,AF);					// set as AF	
	GPIO_PIN_AF_MODE(SERIAL_PORT,SERIAL_CTS,0x07);				// set AF mode
	
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // Enable USART 2 clock		

	// 8 data bits; no flow control; no parity; 1 start, 1 stop bit**	
	
	// Disable USART
	CLR_BITS(USART2->CR1, USART_CR1_UE);
	
	// Configure word length to 8 bit
	CLR_BITS(USART2->CR1, USART_CR1_M);   // M: 00 = 8 data bits
	
	// swap RX/TX
	CLR_BITS(USART2->CR2, USART_CR2_SWAP);   // SWAP RX & TX
	
	// Configure oversampling mode 
	CLR_BITS(USART2->CR1, USART_CR1_OVER8);  // 0 = oversampling by 16, 1 = oversampling by 8

	CLR_BITS(USART2->CR2, USART_CR2_STOP);    // 0 stop bits
	
	CLR_BITS(USART2->CR1, USART_CR1_PCE);	// no parity
                                    
	// USARTDIV = 72MHz/115200 = 625 = 0x0271
	//USART2->BRR  = (72000000/baud); // Set baudrate
	USART2->BRR  = 0x0271; // Set baudrate 115200

	SET_BITS(USART2->CR1, (USART_CR1_RE | USART_CR1_TE));  	// Transmitter and Receiver enable
	
	// interrupts
	SET_BITS(USART2->CR1, USART_CR1_RXNEIE);	// Receive not empty interrupt
	
	SET_BITS(USART2->CR1, USART_CR1_UE); // USART enable                 
	
	while ( (USART2->ISR & USART_ISR_TEACK) == 0); // Verify that the USART is ready for reception
	while ( (USART2->ISR & USART_ISR_REACK) == 0); // Verify that the USART is ready for transmission

	//NVIC_SetPriority(USART2_IRQn, 0);		// priority
	NVIC_EnableIRQ(USART2_IRQn);
}

void UART_putc(char c) {
	while ( (USART2->ISR & USART_ISR_TXE) == 0);
	USART2->TDR = c;
}

void UART_puts(char *buf) {
	while(*buf) {
		UART_putc(*buf++);
	}
}

void UART_printf(char *fmt, ...) {
	va_list args;	// init args
	
	char buf[SERIAL_MAX_BUFSZ];	// setup buffer
	
	va_start(args,fmt);	
	vsnprintf(buf,SERIAL_MAX_BUFSZ,fmt,args);
	va_end(args);
	
	UART_puts(buf);	// print string
}
