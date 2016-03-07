


// Create definitions for UART
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))
#define CPU_16MHz       0x00
#define CPU_8MHz        0x01
#define CPU_4MHz        0x02
#define CPU_2MHz        0x03
#define CPU_1MHz        0x04
#define CPU_500kHz      0x05
#define CPU_250kHz      0x06
#define CPU_125kHz      0x07
#define CPU_62kHz       0x08
#define DIVISOR 1

#define F_CPU 16000000L
#define BAUDRATE 9600
#define BAUD_PRESCALLER 103

// Libraries:
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdlib.h>
#include "uart.h"


/**SERIAL COMMUNICATION AND ASSOCIATED FUNCTIONS**/
void uart_init(void){

	UBRR1H = (uint8_t)(BAUD_PRESCALLER>>8);
	UBRR1L = (uint8_t)(BAUD_PRESCALLER);
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	UCSR1C = ((1<<UCSZ10)|(1<<UCSZ11));
}

// Transmit a byte
void uart_putchar(unsigned char data)
{
 while (!(UCSR1A & (1 << UDRE1)));
 UDR1 = data;
}

// Receive a byte
unsigned char uart_getchar(void)
{
       while(!(UCSR1A & (1<<RXC1))); // Check to see if line is empty
       DDRD |= (1<<6);
       PORTD ^= (1<<6);
       return UDR1;
}

//send a string of characters
void printStr(char* data){

	uint8_t i=0;
	//send each character until we hit the end of the string
	while(data[i] != '\0'){
		uart_putchar(data[i]);
		i++;
	}
}

/*
// Transmit Interrupt
ISR(USART1_UDRE_vect)
{

}

// Receive Interrupt
ISR(USART1_RX_vect)
{

}

*/