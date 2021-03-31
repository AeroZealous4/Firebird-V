/*! \mainpage Experiment: 3_Motion_Control_Simple
 *
 * @author     e-Yantra Team
 * @date       2020/02/12
 *
 * \subsection Aim
 *  This experiment demonstrates simple motion control.
 *
 * \subsection Connections
 * Motors Connections: 
 * Motors are connected to the Microcontroller through L293D Motor Driver IC. <br>
 *		 Motors Pin	  	Microcontroller Pin      			<br>
 *			  RB  	--> 	PA3								<br>
 *			  RF  	--> 	PA2								<br>
 *			  LF  	--> 	PA1								<br>
 *			  LB 	--> 	PA0								<br>
 *
 * PWM/Enable connections:
 * PWM Pins of the Microcontroller are connected to the L293D Motor Driver IC.
 *		   PWM/Enable Pin	  Microcontroller Pin      		<br>
 *		  Left Motor  	--> 	PL4	(OC5A)					<br>
 *		  Right Motor  	--> 	PL3	(OC5B)					<br>
 *
 * \subsection Macro Definitions
 *		motors_dir_ddr_reg		:  DDRA				<br>
 *		motors_dir_port_reg		:  PORTA			<br>
 *		motors_pwm_ddr_reg		:  DDRL				<br>
 *		motors_pwm_port_reg		:  PORTL			<br>
 *
 *		motors_RB_pin			:  PA3				<br>
 *		motors_RF_pin			:  PA2				<br>
 *		motors_LF_pin			:  PA1				<br>
 *		motors_LB_pin			:  PA0				<br>
 *		motors_pwm_R_pin		:  PL4				<br>
 *		motors_pwm_L_pin		:  PL3				<br>
 */


//---------------------------------- HEADER FILES -----------------------------------------------------

#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>					// Standard AVR Delay Library

//---------------------------------- FUNCTIONS ----------------------------------------------------------

//-----------------------------CONFIGURATION FUNCTIONS --------------------------------------------------

/**
 * @brief      Function to configure motor pins
 */
void motors_pin_config(void) {
	motors_dir_ddr_reg |= (1 << motors_RB_pin) | (1 << motors_RF_pin) | (1 << motors_LF_pin) | (1 << motors_LB_pin) ;			// motor pin as output
	motors_dir_port_reg &=  ~( (1 << motors_RB_pin) | (1 << motors_RF_pin) | (1 << motors_LF_pin) | (1 << motors_LB_pin) );		// stop motor intially
}

/**
 * @brief      Function to configure left and right channel pins of the L293D Motor Driver IC for PWM
 */
void pwm_pin_config(void){
	motors_pwm_ddr_reg |= (1 << motors_pwm_R_pin) | (1 << motors_pwm_L_pin);	// left and right channel pin as output
	motors_pwm_port_reg |= (1 << motors_pwm_R_pin) | (1 << motors_pwm_L_pin);	// turn on left and right channel
}

//----------------------------- MOTION RELATED FUNCTIONS ----------------------------------------------

/**
 * @brief      Function to make Firebird-V move forward.
 */
void forward (void) //both wheels forward
{
  	motors_dir_port_reg &=  ~( (1 << motors_RB_pin) | (1 << motors_LB_pin) );	// Make LB and RB LOW
	motors_dir_port_reg |= (1 << motors_RF_pin) | (1 << motors_LF_pin) ;		// Make LF and RF HIGH
}

/**
 * @brief      Function to make Firebird-V move backward.
 */
void back (void) //both wheels backward
{
  	motors_dir_port_reg &=  ~( (1 << motors_RF_pin) | (1 << motors_LF_pin) );	// Make LF and RF LOW
	motors_dir_port_reg |= ((1 << motors_RB_pin) | (1 << motors_LB_pin)) ;		// Make LB and RB HIGH
}

/**
 * @brief      Function to make Firebird-V rotate left.
 */
void left (void) //Left wheel backward, Right wheel forward
{
  	motors_dir_port_reg &=  ~( (1 << motors_RB_pin) | (1 << motors_LF_pin) );	// Make LF and RB LOW
	motors_dir_port_reg |= (1 << motors_RF_pin) | (1 << motors_LB_pin) ;		// Make LB and RF HIGH
}

/**
 * @brief      Function to make Firebird-V rotate right.
 */
void right (void) //Left wheel forward, Right wheel backward
{
  	motors_dir_port_reg &=  ~( (1 << motors_LB_pin) | (1 << motors_RF_pin) );	// Make LB and RF LOW
	motors_dir_port_reg |= (1 << motors_LF_pin) | (1 << motors_RB_pin) ;		// Make LF and RB HIGH
}

/**
 * @brief      Function to make Firebird-V rotate soft left.
 */
void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motors_dir_port_reg &=  ~( (1 << motors_LB_pin) | (1 << motors_RF_pin) | (1 << motors_LF_pin));	// Make LF, LB and RF LOW
	motors_dir_port_reg |= (1 << motors_RF_pin) ;	// Make RF HIGH
}

/**
 * @brief      Function to make Firebird-V rotate soft right.
 */
void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 	motors_dir_port_reg &=  ~( (1 << motors_LB_pin) | (1 << motors_RF_pin) | (1 << motors_RB_pin));	// Make LB, RF and RB LOW
	motors_dir_port_reg |= (1 << motors_LF_pin) ;	// Make LF HIGH
}

/**
 * @brief      Function to make Firebird-V rotate backward left.
 */
void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 	motors_dir_port_reg &=  ~( (1 << motors_LF_pin) | (1 << motors_RF_pin) | (1 << motors_RB_pin));	// Make LF, RF and RB LOW
	motors_dir_port_reg |= (1 << motors_LB_pin) ;	// Make LB HIGH
}

/**
 * @brief      Function to make Firebird-V rotate backward right.
 */
void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motors_dir_port_reg &=  ~( (1 << motors_LF_pin) | (1 << motors_RF_pin) | (1 << motors_LB_pin));	// Make LF, RF and LB LOW
	motors_dir_port_reg |= (1 << motors_RB_pin) ;	// Make RB HIGH
}

/**
 * @brief      Function to make Firebird-V stop.
 */
void stop (void)
{
  	motors_dir_port_reg &=  ~( (1 << motors_LF_pin) | (1 << motors_RF_pin) | (1 << motors_LB_pin) | (1 << motors_RB_pin));	// Make LF, RF, LB and RB LOW
}


//---------------------------------- MAIN ----------------------------------------------------------------
/**
 * @brief      Main Function
 *
 * @details    First Initializes the Motor Driver pins on Firebird-V.
 * 			   then call different functions to move the robot using different motions.
 */
int main(void)
{
	motors_pin_config();
	pwm_pin_config();
	while(1)
	{
	
		forward();						//both wheels forward
		_delay_ms(1000);

		stop();
		_delay_ms(500);
	
		back();							//both wheels backward
		_delay_ms(1000);

		stop();
		_delay_ms(500);
	
		left();							//Left wheel backward, Right wheel forward
		_delay_ms(1000);
	
		stop();
		_delay_ms(500);
	
		right();						//Left wheel forward, Right wheel backward
		_delay_ms(1000);

		stop();
		_delay_ms(500);

		soft_left();					//Left wheel stationary, Right wheel forward
		_delay_ms(1000);
	
		stop();
		_delay_ms(500);

		soft_right();					//Left wheel forward, Right wheel is stationary
		_delay_ms(1000);

		stop();
		_delay_ms(500);

		soft_left_2();					//Left wheel backward, right wheel stationary
		_delay_ms(1000);

		stop();
		_delay_ms(500);

		soft_right_2();					//Left wheel stationary, Right wheel backward
		_delay_ms(1000);

		stop();
		_delay_ms(1000);
	}
}
//---------------------------------- END ------------------------------------------------------------------