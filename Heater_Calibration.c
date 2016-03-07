/*
	Author: Martijus
*/

//#Defines:
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))
#define CPU_16MHz 0x00
#define F_CPU 16000000L 

// Libraries:
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "temp.h"
#include "Timer.h"
#include "ADC.h"

volatile unsigned int count_ms = 0;
void Counter_init (void);
ISR(TIMER1_COMPA_vect){
	count_ms ++;
}

// Testing:
#define LED_ON		(PORTD	|= (1<< PD6))	// Toggle Teensy LED on.
#define LED_OFF		(PORTD	&= ~(1<<PD6))	// Toggle Teensy LED off.

// Heating:
#define Heat_ON		(PORTB	|= (1<< PB2))	// Toggle H-bridge channel 1 on.
#define Heat_OFF	(PORTB	&= ~(1<<PB2))	// Toggle H-bridge channel 1 off.

// Fan:
#define Fan_ON		(PORTB	|= (1<< PB3))	// Toggle H-bridge channel 2 on.
#define Fan_OFF		(PORTB	&= ~(1<<PB3))	// Toggle H-bridge channel 2 off.

//Function declarations:
//void init(void);
void port_init(void);
void init(void);

// Global Variables:
uint8_t input;
uint8_t denTime=20;
uint8_t denTemp=40;
uint8_t annTime;
uint8_t annTemp;
uint8_t eloTime;
uint8_t eloTemp;
char passData[19];

int main(void)
{
	int isTime =0;
	int next = 0;
	double readTemp=0;
	double lowerRange = 0;
	double upperRange = 0;
	init();
	
	while(next != 1){
		lowerRange = denTemp - 1;
		upperRange = denTemp + 1;
		readTemp = getTemp();
		printStr("Denaturing temp is "); //send App
		uart_putchar(denTemp);
		uart_putchar('\n');
		_delay_ms(1000);
		printStr("readTemp is "); //send App
		uart_putchar(readTemp);
		uart_putchar('\n');
		_delay_ms(1000);
		
		if (readTemp < lowerRange)
		{
			PORTB = Heat_ON;
			PORTB = Fan_OFF;
			PORTD = LED_OFF;
		}
		
		if (readTemp > lowerRange && readTemp < upperRange)
		{
			PORTB = Heat_OFF;
			PORTB = Fan_OFF;
			PORTD = LED_ON;
		}
		
		
		if (readTemp > upperRange)
		{
			PORTB = Heat_OFF;
			PORTB = Fan_ON;
			PORTD = LED_OFF;
		}
	}
	
	
	
	
}


void init(void){
	
	CPU_PRESCALE(CPU_16MHz);
	adc_init();
	Counter_init();
	port_init();
	uart_init();
	sei();
}



void port_init(void){
	
	DDRB |= ((1<<PB1)|(1<<PB2)|(1<<PB3)) ; // Sets the data direction for Port B PB1 (Relay Control), PB2 is Heating PB3 is fan as an output.
	DDRF |= (0<<PF0) | (0<<PF1); // Sets the data direction for port F, PF0 (ADC in), PF1 (Low Battery Signal) as an input.
	DDRD |= (1<<6); // Sets the data direction for port D , PD6 (Teensy LED) as an output.
	
	PORTB = 0b00000000; // Sets all PB pin values low.
	PORTF = 0b00000000; // Sets all PF pins low.
	PORTD = 0b00000000; // Sets all PD pins low.
}

