/* Relay_test.c
 *  Author: martijus
 Connect the low voltage detection input to F1 no the teensy
 Connect the relay to B1 on the teensy
 Run code.
 */ 
//***** Includes: *****//
#include <avr/io.h>
// These definitions must be before util/delay.h
#define CPU_16MHz 0x00
#define F_CPU CPU_16MHz
#include <util/delay.h>		// for _delay_ms function



// Battery Protection:
#define Batt_low	(PINF	|= (1<<PF1))	// Check if Pin F1 is high, then low battery detected.
#define Relay_on	(PORTB  |= (1<<PB1))	// Toggle Relay on.
#define Relay_off	(PORTB  &= ~(1<<PB1))	// Toggle Relay off.
#define isRelay_on	(PINB	|= (1<<PB1))	// Check if Pin B1 is high, then Relay is on (e.g. H&C is in cut-off)
#define isRelay_off	(PINB	&= ~(1<<PB1))	// Check if Pin B1 is low, then Relay is off (e.g. H&C is on)
// Testing:
#define LED_ON		(PORTD	|= (1<< PD6))	// Toggle Teensy LED on.
#define LED_OFF		(PORTD	&= ~(1<<PD6))	// Toggle Teensy LED off.




// Main Funiction Declerations:
void port_init(void);	// Set data direction for ports and initial values
void isBatLow(void);	// Check for low battery signal from battery protection and handles low battery protocol.


//*
int main(void){
	port_init();	// Initialize Ports & default PIN states
	while (1)
	{
		isBatLow();	// Check for low battery voltage signal & if low, execute low battery protocol
	}
}
//*/

void port_init(void){
	DDRB |= (1<<PB1); // Sets the data direction for  PB1 == Relay output.
	DDRF |= (0<<PF1); // Sets the data direction for  PF1 == Low Bat input.
	DDRD |= (1<<PD6); // Sets the data direction for PD6 == LED teensy output.
	
	PORTB = 0b00000000; // Sets all PB pin values low.
	PORTF = 0b00000000; // Sets all PF pins low.
	PORTD = 0b00000000; // Sets all PD pins low.
}

void isBatLow(void){
	if (PINF == Batt_low)		// Low battery level detected.
	{
		if (PINB == isRelay_off)// Check to see if relay is already on
		{
			PORTB = Relay_on;	// When the relay turns on, the power to the heating & Cooling block is disconnected.
			PORTD = LED_ON;		// for testing function
			_delay_ms(30000); // for testing function
		}
	}
	
	if (PINF != Batt_low)		// Low battery level NOT detected.
	{
	PORTB = Relay_off;		// When there is no low battery detected the power to the heating & cooling block is connected.
	PORTD = LED_OFF;		// for testing function
	}
}