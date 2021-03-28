/*
TITLE	: UART Communication: ATmega 2560 & ESP32
DATE	: 2019/11/12
AUTHOR	: e-Yantra Team

AIM: To send data on UART#0 of ATMega 2560 to ESP32
*/


// #define F_CPU 16000000UL		// Define Crystal Frequency of eYFi-Mega Board
#define F_CPU 14745600
#include <avr/io.h>				// Standard AVR IO Library
#include <util/delay.h>			// Standard AVR Delay Library
#include <avr/interrupt.h>		// Standard AVR Interrupt Library
#include "uart.h"				// Third Party UART Library
#include "atmega_to_esp32_uart.h"

#define PIN_LED_GREEN	PH5		// Macro for Pin Number of Green Led
#define MESSAGE			"Tx from ATmega 2560> Hello ESP32!\n" // Message to be sent on UART #0

volatile unsigned int count = 0;	// Used in ISR of Timer2 to store ms elasped
unsigned int seconds = 0;			// Stores seconds elasped
char rx_byte;

void init_timer2(void){
	cli();	// Turn off global interrupts

	//Setup Timer2 to fire every 1ms
	TCCR2B = 0x00;        						// Cut off Clock Source to disbale Timer2 while we set it up
	TCNT2  = 130;         						// Reset Timer Count to 130 out of 255
	TIFR2  &= ~(1 << TOV2);        				// Timer2 INT Flag Reg: Clear Timer Overflow Flag
	TIMSK2 |= (1 << TOIE2);        				// Timer2 INT Reg: Timer2 Overflow Interrupt Enable
	TCCR2A = 0x00;        						// Timer2 Control Reg A: Wave Gen Mode normal
	TCCR2B |= (1 << CS22) | (1 << CS20);        // Timer2 Control Reg B: Timer Prescaler set to 128 and Start Timer2

	sei();	// Turn on global interrupts
}


//Timer2 Overflow Interrupt Vector
ISR(TIMER2_OVF_vect) {
  count++;	// increment after 1 ms               
  
  // increment seconds variable after 1000 ms
  if(count > 999){
	seconds++;
	
	uart3_puts(MESSAGE);    // Send data on UART #0 after 1 second

    count = 0;          
  }
  
  TCNT2 = 130;           	// Reset Timer to 130 out of 255
  TIFR2  &= ~(1 << TOV2);	// Timer2 INT Flag Reg: Clear Timer Overflow Flag
};


void init_led(void){
	DDRH    |= (1 << PIN_LED_GREEN);    
	PORTH   |= (1 << PIN_LED_GREEN);    
}

void led_greenOn(void){
	PORTH &= ~(1 << PIN_LED_GREEN);

}

void led_greenOff(void){
	PORTH |= (1 << PIN_LED_GREEN);
}


char uart0_readByte(void){

	uint16_t rx;
	uint8_t rx_status, rx_data;

	rx = uart3_getc();
	rx_status = (uint8_t)(rx >> 8);
	rx = rx << 8;
	rx_data = (uint8_t)(rx >> 8);

	if(rx_status == 0 && rx_data != 0){
		return rx_data;
	} else {
		return -1;
	}
	// return rx;

}

void uart0_SendByte(char rx_byte){

	uart3_putc(rx_byte);

}
void uart_init1(void)
{
	// uart0_init(UART_BAUD_SELECT(115200, F_CPU));
	// uart0_flush();
	uart3_init(UART_BAUD_SELECT(115200, F_CPU));
	uart3_flush();
}

void uart_send_string(char * Str)
{
	uart3_puts(Str);    // Send data on UART #0 after 1 second
}

// int main(void) {
// 	init_led();
// 	init_timer2();


// 	uart_init();
	

// 	while(1){

// 		rx_byte = uart0_readByte();

// 		if(rx_byte != -1){
// 			uart0_putc(rx_byte);
// 			// led_greenOn();
// 		}

// 		// Turn On green led if seconds elasped are even else turn it off
// 		if((seconds % 2) == 0){
// 			led_greenOn();
			
// 		} else {
// 			led_greenOff();
// 		}

// 	}

// 	return 0;	
// }
