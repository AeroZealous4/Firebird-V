/*! \mainpage Experiment: 2_I-O Interfacing
 *
 * @author     e-Yantra Team
 * @date       2020/02/12
 *
 * \subsection Aim
 * To interface the Buzzer, Bar Graph LED and the Interrupt Switch already present on Firebird V
 * robot and turn the Buzzer and the Bar Graph LED when the Interrupt Switch is pressed.
 *
 * \subsection Connections
 * Interrupt Switch				:	PE7 				<br>
 * Buzzer 						:	PC3					<br>
 * Bar Graph LED 				:	PORTJ				<br>
 * 
 * \subsection Macro Definitions
 * buzzer_ddr_reg				: 	DDRC				<br>
 * buzzer_port_reg				:  	PORTC				<br>
 * buzzer_pin					:  	PC3					<br>
 * 
 * interrupt_sw_ddr_reg 		:  	DDRE				<br>
 * interrupt_sw_port_reg		:  	PORTE				<br>
 * interrupt_sw_pin				:  	PE7					<br>
 * 	 
 * bar_graph_led_ddr_reg		:  	DDRJ				<br>
 * bar_graph_led_port_reg		:  	PORTJ				<br>
 * bar_graph_led_1_pin			:	PJ0					<br>
 * bar_graph_led_2_pin			:	PJ1					<br>
 * bar_graph_led_3_pin			:	PJ2					<br>
 * bar_graph_led_4_pin			:	PJ3					<br>
 * bar_graph_led_5_pin			:	PJ4					<br>
 * bar_graph_led_6_pin			:	PJ5					<br>
 * bar_graph_led_7_pin			:	PJ6					<br>
 * bar_graph_led_8_pin			:	PJ7					<br>
 * 
 * Note: Make sure that Jumper J3 is connected on Firebird V for Bargraph LEDs
 */




//---------------------------------- HEADER FILES -----------------------------------------------------

#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>					// Standard AVR Delay Library  




//---------------------------------- FUNCTIONS ---------------------------------------------------------

//-----------------------------CONFIGURATION FUNCTIONS --------------------------------------------------

/**
 * @brief      Function to make **ONLY** 'buzzer_pin' as output and initially set it to low
 */
void buzzer_pin_config (void) {

	// Make 'buzzer_pin' as output
	buzzer_ddr_reg	|= ( 1 << buzzer_pin );
	
	// Set 'buzzer_pin' to low initially
	buzzer_port_reg &= ~( 1 << buzzer_pin );
}

/**
 * @brief      Function to make **ONLY** Interrupt Switch pin as input and pull it up internally
 */
void interrupt_switch_config (void) {

	// Make **ONLY** Interrupt Switch pin as input
	interrupt_sw_ddr_reg &= ~( 1 << interrupt_sw_pin );

	// Make **ONLY** Interrupt Switch pin internally pull-up	
	interrupt_sw_port_reg |= ( 1 << interrupt_sw_pin );	
}

/**
 * @brief      Function to configure the pins of ATmega2560 to which the Bar Graph LED is connected
 */
void LED_bargraph_config (void) {
	bar_graph_led_ddr_reg = 0xFF;  		// Bargraph LED port as output
	bar_graph_led_port_reg = 0x00; 		// LEDs initially off
}

//----------------------------- BUZZER RELATED FUNCTIONS -----------------------------------------------

/**
 * @brief      Function to set **ONLY** 'buzzer_pin' to high, hence turn ON the Buzzer
 */
void buzzer_on (void) {

	// Set 'buzzer_pin' to high
	buzzer_port_reg |= ( 1 << buzzer_pin );
}


/**
 * @brief      Function to set **ONLY** 'buzzer_pin' to low, hence turn OFF the Buzzer
 */
void buzzer_off (void) {

	// Set 'buzzer_pin' to low
	buzzer_port_reg &= ~( 1 << buzzer_pin );
}

//----------------------------- BARGRAPH LED RELATED FUNCTIONS -----------------------------------------

/**
 * @brief      Function to set LED pins to high, hence turn on all LEDs
 */
void bargraph_led_on(void){
	// Turn on all LEDs
	bar_graph_led_port_reg = 0xFF;
}

/**
 * @brief      Function to set LED pins to low, hence turn off all LEDs
 */
void bargraph_led_off(void){
	// Turn off all LEDs
	bar_graph_led_port_reg = 0x00;
}

//---------------------------------- MAIN ----------------------------------------------------------------

/**
 * @brief      Main Function
 *
 * @details    First initializes the necessary devices (Buzzer, Interrupt Switch and Bar Graph LED) 
 * 			   present on Firebird-V, then inside an infinite while 
 * 			   loop checks if the Interrupt Switch is pressed or not. If the switch is pressed it turns 
 * 			   ON the Bar Graph LEDs and the Buzzer till the switch is pressed else it turns OFF both 
 * 			   the devices. 
 */
int main(void) {
	
	// Initialize the necessary devices (Buzzer, Interrupt Switch and Bar Graph LED) required for the experiment.
	buzzer_pin_config();
	interrupt_switch_config();
	LED_bargraph_config();
	
	while (1) {
		
		// If the Interrupt Switch is NOT pressed
		if ((interrupt_sw_pin_reg & (1 << interrupt_sw_pin)) == (1 << interrupt_sw_pin) ) { 
			bargraph_led_off(); 			//Turn off bargraph LEDs
			buzzer_off(); 					//Turn off buzzer
		} else {
			bargraph_led_on(); 				//Turn on bargraph LEDs
			buzzer_on(); 					//Turn on buzzer
		}
	}
}

//---------------------------------- END ------------------------------------------------------------------
