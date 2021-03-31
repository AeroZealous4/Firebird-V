/*! \mainpage 6_Velocity_Control_using_PWM
 *
 * @author     e-Yantra Team
 * @date       2020/02/27
 *
 * \subsection Aim
 * To increase and decrease the speed of motors using FAST PWM Mode.
 *
 * \subsection Connections
 * 
 * Motors Connections: 
 * Motors are connected to the Microcontroller through L293D Motor Driver IC. <br>
 *		 Motors Pin	  	Microcontroller Pin      	<br>
 *			  RB  	--> 	PA3						<br>
 *			  RF  	--> 	PA2						<br>
 *			  LF  	--> 	PA1						<br>
 *			  LB 	--> 	PA0						<br>
 * 
 * PWM Pins of the Microcontroller are connected to the L293D Motor Driver IC.
 *		   PWM Pin	  		Microcontroller Pin      	<br>
 *		  Left Motor  	--> 	PL4						<br>
 *		  Right Motor  	--> 	PL3						<br>
 *
 * \subsection Macro Definitions
 * Macros for Motors:	<br>
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
 *
 * Macros for Timer:	<br>
 *		TCCR5A_reg				:  TCCR5A		// Timer / Counter Control Register 5A			<br>
 *		TCCR5B_reg				:  TCCR5B		// Timer / Counter Control Register 5B			<br>
 *		TCNT5H_reg				:  TCNT5H		// Timer / Counter 5 High Byte register			<br>
 *		TCNT5L_reg				:  TCNT5L		// Timer / Counter 5 Low Byte register			<br>
 *		OCR5AH_reg				:  OCR5AH		// Output Compare Register 5 A High Byte		<br>
 *		OCR5AL_reg				:  OCR5AL		// Output Compare Register 5 A Low Byte			<br>
 *		OCR5BH_reg				:  OCR5BH		// Output Compare Register 5 B High Byte		<br>
 *		OCR5BL_reg				:  OCR5BL		// Output Compare Register 5 B Low Byte			<br>
 *		
 *		COMA1_bit				:  COM5A1			<br>
 *		COMA0_bit				:  COM5A0			<br>
 *		COMB1_bit				:  COM5B1			<br>
 *		COMB0_bit				:  COM5B0			<br>
 *		WGM3_bit				:  WGM53			<br>
 *		WGM2_bit				:  WGM52			<br>
 *		WGM1_bit				:  WGM51			<br>
 *		WGM0_bit				:  WGM50			<br>
 *		CS2_bit					:  CS52				<br>
 *		CS1_bit					:  CS51				<br>
 *		CS0_bit					:  CS50				<br>
 *
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
	motors_dir_port_reg &=  ~( (1 << motors_RB_pin) | (1 << motors_RF_pin) | (1 << motors_LF_pin) | (1 << motors_LB_pin) );		// stop motor initially
}

/**
 * @brief      Function to configure left and right channel pins of the L293D Motor Driver IC for PWM
 */
void pwm_pin_config(void){
	motors_pwm_ddr_reg |= (1 << motors_pwm_R_pin) | (1 << motors_pwm_L_pin);	// left and right channel pin as output
	motors_pwm_port_reg |= (1 << motors_pwm_R_pin) | (1 << motors_pwm_L_pin);	// turn on left and right channel
}

/**
 * @brief      Function to initialize Timer 5 in FAST PWM mode for speed control of motors of Firebird-V
 *
 */
void timer5_init() {
	TCCR5B_reg = 0x00;	//Stop
	
	TCNT5H_reg = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L_reg = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	
	OCR5AH_reg = 0x00;	//Output compare register high value for Left Motor
	OCR5AL_reg = 0xFF;	//Output compare register low value for Left Motor
	
	OCR5BH_reg = 0x00;	//Output compare register high value for Right Motor
	OCR5BL_reg = 0xFF;	//Output compare register low value for Right Motor
	
	// Clear on Compare
	TCCR5A_reg |= (1 << COMA1_bit) | (1 << COMB1_bit);
	TCCR5A_reg &= ~( (1 << COMA0_bit) | (1 << COMB0_bit));

	// Configure for FAST PWM
	TCCR5A_reg |= (1 << WGM0_bit);
	TCCR5A_reg &= ~(1 << WGM1_bit);
	TCCR5B_reg |= (1 << WGM2_bit);
	TCCR5B_reg &= ~(1 << WGM3_bit);

	// Set Prescalar to 64
	TCCR5B_reg |= (1 << CS1_bit) | (1 << CS0_bit);
	TCCR5B_reg &= ~(1 << CS2_bit);
}

//----------------------------- VELOCITY FUNCTION ----------------------------------------------

/**
 * @brief      Function to control the speed of both the motors of Firebird-V
 *
 * @param[in]  left_motor   Left motor speed 0 to 255
 * @param[in]  right_motor  Right motor speed 0 to 255
 */
void velocity (unsigned char left_motor, unsigned char right_motor) {
	OCR5AL_reg = left_motor;
	OCR5BL_reg = right_motor;
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

//---------------------------------- MAIN ----------------------------------------------------------------

/**
 * @brief      Main function
 *
 * @details    First initialized motors, lcd and the bar graph LED after that makes the Firebird-V move motors_move_motors_move_forward using
 * 			   Fast PWM mode first, then Phase correct PWM mode and at the end Phase and Frequency Correct Mode
 *
 * @return     0
 */
int main() {
	
	motors_pin_config();
	pwm_pin_config();
	timer5_init();
	
	while (1) {
		forward();
		//Smaller the value lesser will be the velocity.Try different values between 0 to 255
		velocity (150, 150);		// Move forward
		_delay_ms(1000);

		velocity (100, 255);		// Left turn
		_delay_ms(1000);
		
		velocity (255, 100);		// Right turn
		_delay_ms(1000);
		
		back(); 
		velocity (150, 150);		// Move backward
		_delay_ms(1000);

		velocity (100, 255);		// Backward right
		_delay_ms(1000);
		
		velocity (255, 100);		// Backward left
		_delay_ms(1000);
		
		velocity (0, 0);			// Stop
		_delay_ms(1000);
	}
}
//---------------------------------- END ------------------------------------------------------------------
