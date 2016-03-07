/* SD_PCR.c
 *  Author: martijus
 
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "ADC.h"
#include "Timer.h"

/*/////////////////////////////////////////////////////////////////////////
/////////// Begin Definitions: ////////////////////////////////////////////
*//////////////////////////////////////////////////////////////////////////
// Testing:
#define LED_ON		(PORTD	|= (1<< PD6))	// Toggle Teensy LED on.
#define LED_OFF		(PORTD	&= ~(1<<PD6))	// Toggle Teensy LED off.

// Battery Protection:
#define Batt_low	(PINF	|= (1<<PF1))	// Check if Pin F1 is high, then low battery detected.
#define Relay_on	(PORTB  |= (1<<PB1))	// Toggle Relay on.
#define Relay_off	(PORTB  &= ~(1<<PB1))	// Toggle Relay off.
#define isRelay_on	(PINB	|= (1<<PB1))	// Check if Pin B1 is high, then Relay is on (e.g. H&C is in cut-off)
#define isRelay_off	(PINB	&= ~(1<<PB1))	// Check if Pin B1 is low, then Relay is off (e.g. H&C is on)

// Teensy:
/* Optional clock speeds. **default is 2 MHz**
If your Teensy is modified to run at 3.3 volts, 8 MHz is the maximum.
*/
#define CPU_16MHz       0x00
#define CPU_8MHz        0x01
#define CPU_4MHz        0x02
#define CPU_2MHz        0x03
#define CPU_1MHz        0x04
#define CPU_500kHz      0x05
#define CPU_250kHz      0x06
#define CPU_125kHz      0x07
#define CPU_62kHz       0x08

// UART:
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n)) //teensy Hz setting
#define baud 9600

/*/////////////////////////////////////////////////////////////////////////
/////////// END Definitions: ////////////////////////////////////////////
*//////////////////////////////////////////////////////////////////////////

//Global Variables:
volatile unsigned int count_ms = 0; // for ISR

ISR(TIMER1_COMPA_vect){
	count_ms ++;
}

// Main Funiction Declerations:
void port_init(void);	// Set data direction for ports and initial values
void isBatLow(void);	// Check for low battery signal from battery protection and handles low battery protocol.


/*///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////	Begin Main	/////////////////////////////////////////////
*////////////////////////////////////////////////////////////////////////////////////////////
int main(void)
{
	//Set up UART
	CPU_PRESCALE(CPU_2MHz);
	
	
	
	// Initializations:
	port_init();
	Counter_init();
	adc_init();
	uart_init(baud);
	
	sei(); // Enable interrupts
    while(1)
    {
		isBatLow();
        //TODO:: Please write your application code 
    }
}
/*///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////		END Main 	/////////////////////////////////////////
*////////////////////////////////////////////////////////////////////////////////////////////

/*///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////		Main Functions	/////////////////////////////////////
*////////////////////////////////////////////////////////////////////////////////////////////

void port_init(void){
	DDRB = 0b11111111; // Sets the data direction for port B as an output.
	DDRF = 0b00000000; // Sets the data direction for port F as an input.
	DDRF = 0b00000000; // Sets the data direction for port D as an input.
	
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
			//PORTD = LED_ON;		// for testing function
		}
	}
	
	if (PINF != Batt_low)		// Low battery level NOT detected.
	{
		PORTB = Relay_off;		// When there is no low battery detected the power to the heating & cooling block is connected.
		//PORTD = LED_OFF;		// for testing function
	}
}