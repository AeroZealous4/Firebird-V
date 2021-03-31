/*! \mainpage Experiment: 10_Sensor_Switching
 *
 * @author     e-Yantra Team
 * @date       2020/02/12
 *
 * \subsection Aim
 * To demonstrate the switching ON and OFF of MOSFET switches in order to turn ON and OFF the active part of the sensors.
 *
 * \subsection Connections
 * Sharp Sensors 2, 3, 4 and White-line sensors control pin		:	PG2 				<br>
 * Sharp Sensors 1, 5 control pin								:	PH2 				<br>
 * IR Proximity Sensors 1 to 8 control pin						:	PH3 				<br>
 *
 * \subsection Macro Definitions
 * sharp_234_wl_ctrl_ddr_reg		:	DDRG				<br>
 * sharp_234_wl_ctrl_port_reg		:	PORTG				<br>
 * sharp_234_wl_ctrl_pin			:	PG2					<br>
 * sharp_15_ctrl_ddr_reg			:	DDRH				<br>
 * sharp_15_ctrl_port_reg			:	PORTH				<br>
 * sharp_15_ctrl_pin				:	PH2					<br>
 * ir_prox_ctrl_ddr_reg				:	DDRH				<br>
 * ir_prox_ctrl_port_reg			:	PORTH				<br>
 * ir_prox_ctrl_pin					:	PH3					<br>
 *
 * \subsection Note
 * 1. Make sure that the pins J1-2, J1-3, J1-4 of Jumper J1 are open on the Main Board.					<br>
 * 2. Logic 1 to any of the sensor control pin turns OFF the power to that particular sensor.			<br>
 * 3. Logic 0 to any of the sensor control pin turns ON the power to that particular sensor.			<br>
 * 4. To use IR Proximity Sensor as directional light intensity sensor,									<br>
 *	  turn OFF IR Proximity Sensor. It will turn OFF the IR LEDs.										<br>
 * 
 */


//---------------------------------- HEADER FILES -----------------------------------------------------

#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>					// Standard AVR Delay Library


//---------------------------------- FUNCTIONS ----------------------------------------------------------

//----------------------------- CONFIGURATION FUNCTIONS -------------------------------------------------

/**
 * @brief      Function to configure the sensor control pins, i.e. of MOSFET switches as output
 */
void sensors_control_pins_config (void) {

	// Configure the control pin of 3 White Line Sensors
	wl_pwr_ctrl_ddr_reg			|= ( 1 << wl_pwr_ctrl_pin );
	
	// Configure the control pin of all Sharp Sensors
	sharp_pwr_ctrl_ddr_reg		|= ( 1 << sharp_pwr_ctrl_pin );
	
	// Configure the control pin of all 8 IR Proximity Sensors
	ir_prox_pwr_ctrl_ddr_reg	|= ( 1 << ir_prox_pwr_ctrl_pin );
}


/**
 * @brief      Function to turn ON 3 White Line Sensors
 */
void turn_on_wl_sensors (void) {

	// Turn ON the control pin of 3 White Line Sensors
	wl_pwr_ctrl_port_reg	&= ~( 1 << wl_pwr_ctrl_pin );
}


/**
 * @brief      Function to turn OFF 3 White Line Sensors
 */
void turn_off_wl_sensors (void) {

	// Turn OFF the control pin of 3 White Line Sensors
	wl_pwr_ctrl_port_reg	|= ( 1 << wl_pwr_ctrl_pin );
}


/**
 * @brief      Function to turn ON all Sharp Sensors
 */
void turn_on_sharp_sensors (void) {

	// Turn ON the control pin of all Sharp Sensors
	sharp_pwr_ctrl_port_reg		&= ~( 1 << sharp_pwr_ctrl_pin );
}


/**
 * @brief      Function to turn OFF all Sharp Sensors
 */
void turn_off_sharp_sensors (void) {
	
	// Turn OFF the control pin of all Sharp Sensors
	sharp_pwr_ctrl_port_reg		|= ( 1 << sharp_pwr_ctrl_pin );
}


/**
 * @brief      Function to turn ON all IR Proximity Sensors
 */
void turn_on_ir_proxy_sensors (void) {

	// Turn ON the control pin of all IR Proximity Sensors
	ir_prox_pwr_ctrl_port_reg	&= ~( 1 << ir_prox_pwr_ctrl_pin );
}


/**
 * @brief      Function to turn OFF all IR Proximity Sensors
 */
void turn_off_ir_proxy_sensors (void) {

	// Turn OFF the control pin of all IR Proximity Sensors
	ir_prox_pwr_ctrl_port_reg	|= ( 1 << ir_prox_pwr_ctrl_pin );
}


/**
 * @brief      Function to turn ON all the sensors by turning ON the respective MOSFET switches
 */
void turn_on_all_sensors (void) {
	
	// Turn ON the control pin of 3 White Line Sensors
	turn_on_wl_sensors();
	
	// Turn ON the control pin of all Sharp Sensors
	turn_on_sharp_sensors();
	
	// Turn ON the control pin of all IR Proximity Sensors
	turn_on_ir_proxy_sensors();
}


/**
 * @brief      Function to turn OFF all the sensors by turning OFF the respective MOSFET switches
 */
void turn_off_all_sensors (void) {
	
	// Turn OFF the control pin of 3 White Line Sensors
	turn_off_wl_sensors();
	
	// Turn OFF the control pin of all Sharp Sensors
	turn_off_sharp_sensors();
	
	// Turn OFF the control pin of all IR Proximity Sensors
	turn_off_ir_proxy_sensors();
}


//---------------------------------- MAIN ----------------------------------------------------------------
/**
 * @brief      Main Function
 *
 * @details    First Initializes the sensor control pins, i.e. of MOSFET switches as output by calling sensors_control_pins_config()
 *			   function and then calls different functions to turn ON and OFF the White Line, Sharp and IR proximity Sensors with certain delay.
 */
int main(void) {
		
	sensors_control_pins_config();		// Initialize the sensor control pins, i.e. of MOSFET switches as output
	
	while (1) {

		turn_off_all_sensors();			// Turn OFF all the sensors by turning OFF the respective MOSFET switches
		_delay_ms(1000);
		
		turn_on_wl_sensors();			// Turn ON 3 White Line Sensors
		_delay_ms(1000);
		
		turn_off_wl_sensors();			// Turn OFF 3 White Line Sensors
		_delay_ms(1000);
		
		turn_on_sharp_sensors();		// Turn ON Sharp Sensors
		_delay_ms(1000);
		
		turn_off_sharp_sensors();		// Turn OFF Sharp Sensors
		_delay_ms(1000);
		
		turn_on_ir_proxy_sensors();		// Turn ON IR Proximity Sensors
		_delay_ms(1000);
		
		turn_off_ir_proxy_sensors();	// Turn OFF IR Proximity Sensors
		_delay_ms(1000);
		
		turn_on_all_sensors();			// Turn ON all the sensors by turning ON the respective MOSFET switches
		_delay_ms(1000);
		
	}
}
//---------------------------------- END ------------------------------------------------------------------
