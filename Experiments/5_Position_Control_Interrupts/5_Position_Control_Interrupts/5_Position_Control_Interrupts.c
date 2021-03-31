/*! \mainpage Experiment: 5_Position_Control_Interrupt
 *
 * @author     e-Yantra Team
 * @date       2020/02/12
 *
 * \subsection Aim
 * To interface position encoders and demonstrate use of it to traverse the robot by specified distance and rotate by specific angle.
 *
 * \subsection Connections
 * Motors Connections: 
 * Motors are connected to the Microcontroller through L293D Motor Driver IC. <br>
 *		 Motors Pin	  	Microcontroller Pin      	<br>
 *			  RB  	--> 	PA3						<br>
 *			  RF  	--> 	PA2						<br>
 *			  LF  	--> 	PA1						<br>
 *			  LB 	--> 	PA0						<br>
 * 
 * PWM Pins of the Microcontroller are connected to the L293D Motor Driver IC.
 *		   PWM Pin	  		Microcontroller Pin      		<br>
 *		  Left Motor  	--> 	PL4	(OC5A)					<br>
 *		  Right Motor  	--> 	PL3	(OC5B)					<br> 
 * 
 * Position Encoders Connections:
 *		   Encode Pin	  			Microcontroller Pin			<br>
 *		  Left Motor Encoder  	--> 	PE4 (INT4)				<br>
 *		  Right Motor Encoder 	--> 	PE5 (INT5)				<br>
 *
 * \subsection Macro Definitions
 * Macros for Motors:	<br>			
 *		motors_dir_ddr_reg			:  DDRA				<br>
 *		motors_dir_port_reg			:  PORTA			<br>
 *		motors_pwm_ddr_reg			:  DDRL				<br>
 *		motors_pwm_port_reg			:  PORTL			<br>
 *		position_encoder_ddr_reg	:  DDRE				<br>
 *		position_encoder_port_reg	:  PORTE			<br>
 *
 *		motors_RB_pin				:  PA3				<br>
 *		motors_RF_pin				:  PA2				<br>
 *		motors_LF_pin				:  PA1				<br>
 *		motors_LB_pin				:  PA0				<br>
 *		motors_pwm_R_pin			:  PL4				<br>
 *		motors_pwm_L_pin			:  PL3				<br>
 *		left_encoder_pin			:  PE4				<br>
 *		right_encoder_pin			:  PE5				<br>
 *		
 * Macros for Position Encoder Interrupt: <br>
 *		EIMSK_reg					:  EIMSK			<br>
 *		EICRB_reg					:  EICRB			<br>
 *
 *		interrupt_left_encoder_pin	:  INT4				<br>
 *		interrupt_right_encoder_pin	:  INT5				<br>
 *		interrupt_ISC_right_bit1	:  ISC51			<br>
 *		interrupt_ISC_right_bit0	:  ISC50			<br>
 *		interrupt_ISC_left_bit1		:  ISC41			<br>
 *		interrupt_ISC_left_bit0		:  ISC40			<br>
 *
 * Note: Make sure that optimization: -O0
 *
 */


//---------------------------------- HEADER FILES -----------------------------------------------------

#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>					// Standard AVR Delay Library

//---------------------------------- GLOBAL VARIABLES --------------------------------------------------

#define angle_resolution 4.090			//resolution used for angle rotation
#define distance_resolution 5.338		//resolution used for distance traversal

unsigned long int ShaftCountLeft = 0; 	//to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; 	//to keep track of right position encoder			
unsigned int Degrees; 					//to accept angle in degrees for turning

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
 * @brief      Function to configure left and right encoder pins
 */
void position_encoder_pin_config (void)
{
 	position_encoder_ddr_reg  &= ~(1 << left_encoder_pin | 1 << right_encoder_pin);  	//Set the direction of the encoder pins as input
 	position_encoder_port_reg |= (1 << left_encoder_pin | 1 << right_encoder_pin); 		//Enable internal pull-up for encoder pins
}

/**
 * @brief      Function to configure external interrupt for encoder pins
 */
void position_encoder_interrupt_config (void)
{
 	// all interrupts have to be disabled before configuring interrupts
	cli();	// Disable Interrupts Globally
	
	// Turn ON INT4 and INT5 (alternative function of PE4 and PE5 i.e Left and Right Encoder Pin)
	EIMSK_reg |= (1 << interrupt_left_encoder_pin | 1 << interrupt_right_encoder_pin);

	// Falling Edge detection on INT4 and INT5 pins
	EICRB_reg |= (1 << interrupt_ISC_left_bit1 | 1 << interrupt_ISC_right_bit1);
	EICRB_reg &= ~(1 << interrupt_ISC_left_bit0 | 1 << interrupt_ISC_right_bit0);

	sei();	// Enable Interrupts Globally
}

//----------------------------- INTERRUPT SERVICE ROUTINES ----------------------------------------------

/**
 * @brief      ISR for right position encoder
 */
ISR(INT5_vect)  
{
	ShaftCountRight++;  //increment right shaft position count
}

/**
 * @brief      ISR for left position encoder
 */
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
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

//----------------------------- ENCODER RELATED FUNCTIONS ----------------------------------------------

/**
 * @brief      Function to rotate Firebird-V by specified degrees
 * @param[in]  Degrees   Rotation angle 0 to 360
 */
void angle_rotate(unsigned int Degrees)
{
	 float ReqdShaftCount = 0;
	 unsigned long int ReqdShaftCountInt = 0;

	 ReqdShaftCount = (float) Degrees/ angle_resolution; // division by resolution to get shaft count
	 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	 ShaftCountRight = 0; 
	 ShaftCountLeft = 0; 

	 while (1)
	 {
		  if((ShaftCountRight >= ReqdShaftCountInt) || (ShaftCountLeft >= ReqdShaftCountInt))
			break;
	 }
	 stop(); //Stop robot
}

/**
 * @brief      Function to move Firebird-V by specified distance
 * @param[in]  DistanceInMM   Distance in mm 0 to 65535
 */
void linear_distance_mm(unsigned int DistanceInMM)
{
	 float ReqdShaftCount = 0;
	 unsigned long int ReqdShaftCountInt = 0;

	 ReqdShaftCount = DistanceInMM / distance_resolution; // division by resolution to get shaft count
	 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
	 ShaftCountRight = 0;
	 ShaftCountLeft = 0;
	 while(1)
	 {
		  if((ShaftCountRight >= ReqdShaftCountInt) || (ShaftCountLeft >= ReqdShaftCountInt))
			  break;
	 } 
	 stop(); //Stop robot
}

/**
 * @brief      Function to move forward Firebird-V by specified distance
 * @param[in]  DistanceInMM   Distance in mm 0 to 65535
 */
void forward_mm(unsigned int DistanceInMM)
{
	 forward();
	 linear_distance_mm(DistanceInMM);
}

/**
 * @brief      Function to move backward Firebird-V by specified distance
 * @param[in]  DistanceInMM   Distance in mm 0 to 65535
 */
void back_mm(unsigned int DistanceInMM)
{
	 back();
	 linear_distance_mm(DistanceInMM);
}

/**
 * @brief      Function to rotate Firebird-V left by specified degrees
 * @param[in]  Degrees   Rotation angle 0 to 360
 */
void left_degrees(unsigned int Degrees) 
{
	 // 88 pulses for 360 degrees rotation 4.090 degrees per count
	 left(); //Turn left
	 angle_rotate(Degrees);
}

/**
 * @brief      Function to rotate Firebird-V right by specified degrees
 * @param[in]  Degrees   Rotation angle 0 to 360
 */
void right_degrees(unsigned int Degrees)
{
	 // 88 pulses for 360 degrees rotation 4.090 degrees per count
	 right(); //Turn right
	 angle_rotate(Degrees);
}

/**
 * @brief      Function to rotate Firebird-V left by specified degrees
 * @param[in]  Degrees   Rotation angle 0 to 360
 */
void soft_left_degrees(unsigned int Degrees)
{
	 // 176 pulses for 360 degrees rotation 2.045 degrees per count
	 soft_left(); //Turn soft left
	 Degrees=Degrees*2;
	 angle_rotate(Degrees);
}

/**
 * @brief      Function to rotate Firebird-V right by specified degrees
 * @param[in]  Degrees   Rotation angle 0 to 360
 */
void soft_right_degrees(unsigned int Degrees)
{
	 // 176 pulses for 360 degrees rotation 2.045 degrees per count
	 soft_right();  //Turn soft right
	 Degrees=Degrees*2;
	 angle_rotate(Degrees);
}

/**
 * @brief      Function to rotate Firebird-V left by specified degrees
 * @param[in]  Degrees   Rotation angle 0 to 360
 */
void soft_left_2_degrees(unsigned int Degrees)
{
	 // 176 pulses for 360 degrees rotation 2.045 degrees per count
	 soft_left_2(); //Turn reverse soft left
	 Degrees=Degrees*2;
	 angle_rotate(Degrees);
}

/**
 * @brief      Function to rotate Firebird-V right by specified degrees
 * @param[in]  Degrees   Rotation angle 0 to 360
 */
void soft_right_2_degrees(unsigned int Degrees)
{
	 // 176 pulses for 360 degrees rotation 2.045 degrees per count
	 soft_right_2();  //Turn reverse soft right
	 Degrees=Degrees*2;
	 angle_rotate(Degrees);
}

//---------------------------------- MAIN ----------------------------------------------------------------
/**
 * @brief      Main Function
 *
 * @details    First Initializes the Motor and Position Encoder present on Firebird-V.
 * 			   then call different functions to move the robot by specified distance or rotate the robot by specific angle.
 */
int main(void)
{
	motors_pin_config();
	pwm_pin_config();
	position_encoder_pin_config();
	position_encoder_interrupt_config();

	while(1)
	{
		forward_mm(100); 			//Moves robot forward 100mm
		_delay_ms(500);			
		
		back_mm(100);   			//Moves robot backward 100mm			
		_delay_ms(500);
		
		left_degrees(90); 			//Rotate robot left by 90 degrees
		_delay_ms(500);
		
		right_degrees(90); 			//Rotate robot right by 90 degrees
		_delay_ms(500);
		
		soft_left_degrees(90); 		//Rotate (soft turn) by 90 degrees
		_delay_ms(500);
		
		soft_right_degrees(90);		//Rotate (soft turn) by 90 degrees
		_delay_ms(500);

		soft_left_2_degrees(90); 	//Rotate (soft turn) by 90 degrees
		_delay_ms(500);
		
		soft_right_2_degrees(90);	//Rotate (soft turn) by 90 degrees
		_delay_ms(500);
	}
}
//---------------------------------- END ------------------------------------------------------------------
