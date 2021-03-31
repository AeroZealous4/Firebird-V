/*! \mainpage Experiment: 8_Timer_Overflow_Interrupt
 *
 * @author     e-Yantra Team
 * @date       2020/02/12
 *
 * \subsection Aim
 * To interface the Buzzer already present on Firebird V robot and using Timer 4 Overflow Interrupt, perform a
 * periodic task of turning the Buzzer ON and OFF with time period of 1 second.
 *
 * \subsection Connections
 * Buzzer 				:  PC3 				<br>
 *
 * \subsection Macro Definitions
 * Macros for Buzzer:   <br>
 *		buzzer_ddr_reg			:  DDRC				<br>
 *		buzzer_port_reg			:  PORTC			<br>
 *		buzzer_pin				:  PC3				<br>
 *
 * Macros for Timer:	<br>
 *		TCCR4A_reg				:  TCCR4A		// Timer / Counter Control Register 4A			<br>
 *		TCCR4B_reg				:  TCCR4B		// Timer / Counter Control Register 4B			<br>
 *		TCNT4H_reg				:  TCNT4H		// Timer / Counter 4 High Byte register			<br>
 *		TCNT4L_reg				:  TCNT4L		// Timer / Counter 4 Low Byte register			<br>
 *		TIMSK4_reg				:  TIMSK4		// Timer / Counter Interrupt Mask register 4	<br>
 *
 *		COMA1_bit				:  COM4A1			<br>
 *		COMA0_bit				:  COM4A0			<br>
 *		COMB1_bit				:  COM4B1			<br>
 *		COMB0_bit				:  COM4B0			<br>
 *		WGM3_bit				:  WGM43			<br>
 *		WGM2_bit				:  WGM42			<br>
 *		WGM1_bit				:  WGM41			<br>
 *		WGM0_bit				:  WGM40			<br>
 *		CS2_bit					:  CS42				<br>
 *		CS1_bit					:  CS41				<br>
 *		CS0_bit					:  CS40				<br>
 */


//---------------------------------- HEADER FILES -------------------------------------------------------

#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>					// Standard AVR Delay Library


//---------------------------------- FUNCTIONS ----------------------------------------------------------

/**
 * @brief      Makes **ONLY** 'buzzer_pin' as output and initially sets it to low
 */
void buzzer_pin_config (void) {

	// Make 'buzzer_pin' as output
	buzzer_ddr_reg	|= ( 1 << buzzer_pin );
	
	// Set 'buzzer_pin' to low initially to turn it OFF
	buzzer_port_reg &= ~( 1 << buzzer_pin );
}


/**
 * @brief      Initializes Timer 4 in Normal mode for the up (incremental) counting, where the counter simply
 *				overruns when it passes its maximum 16-bit value (MAX = 0xFFFF) and then restarts from the BOTTOM (0x0000)
 */
void timer4_init (void) {
	
	// First stopping the Timer / Counter 4 by resetting the Clock Select bits 2, 1 and 0
	// to configure it in the desired Compare Output and Waveform Generation mode
	TCCR4B_reg	&= ~( ( 1 << CS2_bit ) | ( 1 << CS1_bit ) | ( 1 << CS0_bit ) );
	
	// Selecting the Normal mode ( Mode 0 ) of Operation for Timer / Counter 4 by setting appropriate values of
	// Waveform Generation Mode Bits 3, 2 in TCCR4B_reg and Bits 1, 0 in TCCR4A_reg ( Refer Table 17-2 in ATmega2560 datasheet Pg. No. 145 )
	TCCR4B_reg	&= ~( ( 1 << WGM3_bit ) | ( 1 << WGM2_bit ) );
	TCCR4A_reg	&= ~( ( 1 << WGM1_bit ) | ( 1 << WGM0_bit ) );
	
	// Selecting the Normal port operation for Timer / Counter 4 non-PWM by setting appropriate values of
	// Compare Output Mode Bits 1, 0 for Channels A, B and C in TCCR4A_reg ( Refer Table 17-3 in ATmega2560 datasheet Pg. No. 155 )
	TCCR4A_reg	&= ~( ( 1 << COMA1_bit ) | ( 1 << COMA0_bit ) | ( 1 << COMB1_bit ) | ( 1 << COMB0_bit ) | ( 1 << COMC1_bit ) | ( 1 << COMC0_bit ) );
	
	// Selecting the Clock Source for Timer / Counter 4 with division factor of 1024 from prescaler by setting appropriate values of
	// Clock Select bit 2, 1, 0 in TCCR4B_reg ( Refer Table 17-6 in ATmega2560 datasheet Pg. No. 157 )
	TCCR4B_reg	|= ( ( 1 << CS2_bit ) | ( 1 << CS0_bit ) );
	TCCR4B_reg	&= ~( ( 1 << CS1_bit ) );
	
	/*
	
	Q: Why do we select the division factor of 1024 such that the Timer / Counter 4 uses the clock of F_CPU / 1024 ?
	
	Ans: The crystal frequency or system clock ( F_CPU ) connected to ATmega2560 on Firebird V robot is 14.7456 MHz or 14745600 Hz.
		 Timer 4 is of 16-bit meaning it will count from 0 up to maximum of 65535, in total 65536 clock pulses. So, the maximum delay a timer
		 can produce = 65536 x ( 1 / 14745600 ) = 4.44 milliseconds. Since we want to create a delay of minimum 1 second, we have to increase the time
		 period of one clock pulse, which means the clock provided to Timer 4 should be decreased. This can be achieved with the help of prescaler.
		 So, if we choose the division factor of 1024 from prescaler, the clock for Timer 4 = ( F_CPU / 1024 ) = ( 14745600 / 1024 ) = 14400 Hz.
		 Hence, the maximum delay Timer 4 can now produce = 65536 x ( 1 / 14400 ) = 4.5511 seconds. This is enough for the application at our hand since
		 we just have to create delay of 1 second.
	
	*/
	
	// Load the Timer / Counter 4 registers with the value from where the Timer will start counting and will reach its maximum value i.e. 65535 ( 0xFFFF )
	// such that it will take approximately 1 second for Timer 4 to reach the maximum and overflow.
	TCNT4H_reg	= 0xC7;
	TCNT4L_reg	= 0xBF;
}


/**
 * @brief      This ISR will be automatically invoked when the Timer 4 gets overflowed.
 */
ISR( TIMER4_OVF_vect ) {
	
	// Reload the Timer / Counter 4 registers with the value from where the Timer will start counting and will reach its maximum value i.e. 65535 ( 0xFFFF )
	// such that it will take approximately 1 second for Timer 4 to reach the maximum and overflow.
	TCNT4H_reg	= 0xC7;
	TCNT4L_reg	= 0xBF;
	
	// Toggle the 'buzzer_pin' to high to low and low to high each time ISR is called
	buzzer_port_reg ^= ( 1 << buzzer_pin );
	
}


//---------------------------------- MAIN ----------------------------------------------------------------
/**
 * @brief      Main Function
 *
 * @details    First Initializes the Buzzer present on Firebird-V by calling buzzer_pin_config() function,
 * 			   and Timer 4 by calling timer4_init() function. Timer 4 Overflow Interrupt is also enabled.
 *			   Timer 4 is initialized to start counting and overflow in approximately 1 second.
 *			   The ISR will be called automatically and inside ISR, toggle ' buzzer_pin ' to set it high and low and so on,
 *			   i.e. beeping the Buzzer every second.
 */
int main(void) {
		
	buzzer_pin_config();				// Initialize the Buzzer
	timer4_init();						// Initialize the Timer
	
	// Clear the global interrupt flag bit ( I-bit ) in SREG before enabling Timer 4 Overflow Interrupt
	cli();
	
	TIMSK4_reg	|= ( 1 << TOIE_bit );	// Set the TOIE_bit to enable the Interrupt for Timer 4 Overflow
	
	// Set the global interrupt flag bit ( I-bit ) in SREG after enabling Timer 4 Overflow Interrupt
	sei();
	
	while (1);
}
//---------------------------------- END ------------------------------------------------------------------
