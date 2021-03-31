/*! \mainpage Experiment: 1_Buzzer_Beep
 *
 * @author     e-Yantra Team
 * @date       2020/02/12
 *
 * \subsection Aim
 * To interface the Buzzer already present on Firebird V robot and turn it ON and OFF at an interval of a second.
 *
 * \subsection Connections
 * Buzzer 				:  PC3 				<br>
 *
 * \subsection Macro Definitions
 * buzzer_ddr_reg		:  DDRC				<br>
 * buzzer_port_reg		:  PORTC			<br>
 * buzzer_pin			:  PC3				<br>
 * 
 */


//---------------------------------- HEADER FILES -----------------------------------------------------

#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>					// Standard AVR Delay Library


//---------------------------------- FUNCTIONS ----------------------------------------------------------

//-----------------------------CONFIGURATION FUNCTIONS --------------------------------------------------

/**
 * @brief      Function to make **ONLY** 'buzzer_pin' as output and initially set it to low
 */
void buzzer_pin_config (void) {

	// Make 'buzzer_pin' as output
	buzzer_ddr_reg	|= ( 1 << buzzer_pin );
	
	// Set 'buzzer_pin' to low initially to turn it OFF
	buzzer_port_reg &= ~( 1 << buzzer_pin );
}

//----------------------------- BUZZER RELATED FUNCTIONS -----------------------------------------------

/**
 * @brief      Function to set **ONLY** 'buzzer_pin' to high, hence turn ON the Buzzer
 */
void buzzer_on (void) {

	// Set 'buzzer_pin' to high to turn it ON
	buzzer_port_reg |= ( 1 << buzzer_pin );
}


/**
 * @brief      Function to set **ONLY** 'buzzer_pin' to low, hence turn OFF the Buzzer
 */
void buzzer_off (void) {

	// Set 'buzzer_pin' to low to turn it OFF
	buzzer_port_reg &= ~( 1 << buzzer_pin );
}


//---------------------------------- MAIN ----------------------------------------------------------------
/**
 * @brief      Main Function
 *
 * @details    First Initializes the Buzzer present on Firebird-V by calling buzzer_pin_config() function,
 * 			   then calls the buzzer_on() and buzzer_off() functions to turn the Buzzer ON and OFF at an interval of a second.
 */
int main(void) {
		
	buzzer_pin_config();				// Initialize the Buzzer
	
	while (1) {

		buzzer_on();					// Turn ON the Buzzer
		_delay_ms(1000);				// Delay of 1 second
		
		buzzer_off();					// Turn OFF the Buzzer
		_delay_ms(1000);				// Delay of 1 second
	}
}
//---------------------------------- END ------------------------------------------------------------------
