/*! \mainpage Experiment: 7_Servo_Motor_Control_using_PWM
 *
 * @author     e-Yantra Team
 * @date       2020/02/12
 *
 * \subsection Aim
 * To interface Servo motor to Firebird V and control using 10 bit fast PWM mode.
 *
 * \subsection Connections
 *		Servo Motors Pin	Microcontroller Pin      	<br>
 *			  Servo 1  	--> 	PB5 (OC1A)					<br>
 *			  Servo 2  	--> 	PB6	(OC1B)					<br>
 *			  Servo 3  	--> 	PB7	(OC1C)					<br>
 * 
 * \subsection Macro Definitions
 * Macros for Servo Motors:	<br>
 *		servo_ddr_reg			:  DDRB				<br>
 *		servo_port_reg			:  PORTB			<br>
 *
 *		servo_1_pin				:  PB5				<br>
 *		servo_2_pin				:  PB6				<br>
 *		servo_3_pin				:  PB7				<br>
 * 
 * Macros for Timer:	<br>
 *		TCCR1A_reg				:  TCCR1A		// Timer / Counter Control Register 1A			<br>
 *		TCCR1B_reg				:  TCCR1B		// Timer / Counter Control Register 1B			<br>
 *		TCNT1H_reg				:  TCNT1H		// Timer / Counter 1 High Byte register			<br>
 *		TCNT1L_reg				:  TCNT1L		// Timer / Counter 1 Low Byte register			<br>
 *		OCR1AH_reg				:  OCR1AH		// Output Compare Register 1 A High Byte		<br>
 *		OCR1AL_reg				:  OCR1AL		// Output Compare Register 1 A Low Byte			<br>
 *		OCR1BH_reg				:  OCR1BH		// Output Compare Register 1 B High Byte		<br>
 *		OCR1BL_reg				:  OCR1BL		// Output Compare Register 1 B Low Byte			<br>
 *		OCR1CH_reg				:  OCR1CH		// Output Compare Register 1 C High Byte		<br>
 *		OCR1CL_reg				:  OCR1CL		// Output Compare Register 1 C Low Byte			<br>
 *
 *		COMA1_bit				:  COM1A1			<br>
 *		COMA0_bit				:  COM1A0			<br>
 *		COMB1_bit				:  COM1B1			<br>
 *		COMB0_bit				:  COM1B0			<br>
  *		COMC1_bit				:  COM1C1			<br>
  *		COMC0_bit				:  COM1C0			<br>
 *		WGM3_bit				:  WGM13			<br>
 *		WGM2_bit				:  WGM12			<br>
 *		WGM1_bit				:  WGM11			<br>
 *		WGM0_bit				:  WGM10			<br>
 *		CS2_bit					:  CS12				<br>
 *		CS1_bit					:  CS11				<br>
 *		CS0_bit					:  CS10				<br>
 *
 */


//---------------------------------- HEADER FILES -----------------------------------------------------

#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>					// Standard AVR Delay Library

//---------------------------------- FUNCTIONS ----------------------------------------------------------

//-----------------------------CONFIGURATION FUNCTIONS --------------------------------------------------

/**
 * @brief      Function to configure servo 1 pin
 */
void servo1_pin_config (void)
{
 	servo_ddr_reg  |= (1 << servo_1_pin);  		//making Servo 1 pin output
 	servo_port_reg |= (1 << servo_1_pin); 		//setting servo 1 pin to logic 1
}

/**
 * @brief      Function to configure servo 2 pin
 */
void servo2_pin_config (void)
{
 	servo_ddr_reg  |= (1 << servo_2_pin);  		//making Servo 2 pin output
 	servo_port_reg |= (1 << servo_2_pin); 		//setting servo 2 pin to logic 1
}

/**
 * @brief      Function to configure servo 3 pin
 */
void servo3_pin_config (void)
{
 	servo_ddr_reg  |= (1 << servo_3_pin);  		//making Servo 3 pin output
 	servo_port_reg |= (1 << servo_3_pin); 		//setting servo 3 pin to logic 1
}

/**
 * @brief      Function to configure timer 1 with 10 bit fast pwm (52.25Hz)
 */
void timer1_init(void)
{
 	TCCR1B_reg &= ~(1 << CS2_bit | 1 << CS1_bit | 1 << CS0_bit); 			//stop the timer
 	
	TCNT1H_reg = 0xFC; 			//Counter high value to which OCR1xH value is to be compared with
 	TCNT1L_reg = 0x01;			//Counter low value to which OCR1xH value is to be compared with
 	OCR1AH_reg = 0x03;			//Output compare Register high value for servo 1
 	OCR1AL_reg = 0xFF;			//Output Compare Register low Value For servo 1
 	OCR1BH_reg = 0x03;			//Output compare Register high value for servo 2
 	OCR1BL_reg = 0xFF;			//Output Compare Register low Value For servo 2
 	OCR1CH_reg = 0x03;			//Output compare Register high value for servo 3
 	OCR1CL_reg = 0xFF;			//Output Compare Register low Value For servo 3

	//{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0} For Overriding normal port functionality to OCRnA outputs.
	//{WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode
 	TCCR1A_reg |= (1 << COMA1_bit | 1 << COMB1_bit | 1 << COMC1_bit | 1 << WGM1_bit | 1 << WGM0_bit); 
	TCCR1A_reg &= ~(1 << COMA0_bit | 1 << COMB0_bit | 1 << COMC0_bit);

	//WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
 	TCCR1B_reg |= (1 << WGM2_bit | 1 << CS2_bit); 
	TCCR1B_reg &= ~(1 << CS1_bit | CS0_bit);
}

//----------------------------- SERVO FUNCTIONS --------------------------------------------------

/**
 * @brief      Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
 */
void servo_1(unsigned char degrees)  
{
 	float PositionPanServo = 0;
  	PositionPanServo = ((float)degrees / 1.86) + 35.0;
 	OCR1AH_reg = 0x00;
 	OCR1AL_reg = (unsigned char) PositionPanServo;
}

/**
 * @brief      Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
 */
void servo_2(unsigned char degrees)  
{
 	float PositionPanServo = 0;
  	PositionPanServo = ((float)degrees / 1.86) + 35.0;
 	OCR1BH_reg = 0x00;
 	OCR1BL_reg = (unsigned char) PositionPanServo;
}

/**
 * @brief      Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
 */
void servo_3(unsigned char degrees)  
{
 	float PositionPanServo = 0;
  	PositionPanServo = ((float)degrees / 1.86) + 35.0;
 	OCR1CH_reg = 0x00;
 	OCR1CL_reg = (unsigned char) PositionPanServo;
}

//----------------------------- SERVO FREE FUNCTIONS ----------------------------------------------

/**
 * @brief      Function to free the Servo 1
 * 
 * @details    Servo_free functions unlocks the servo motors from the any angle and make them free by giving 100% duty cycle at the PWM. 
 * 			   This function can be used to reduce the power consumption of the motor if it is holding load against the gravity. 
 */
void servo_1_free (void) //makes servo 1 free rotating
{
 	OCR1AH_reg = 0x03; 
 	OCR1AL_reg = 0xFF; 			//Servo 1 off
}

/**
 * @brief      Function to free the Servo 2
 * 
 * @details    Servo_free functions unlocks the servo motors from the any angle and make them free by giving 100% duty cycle at the PWM. 
 * 			   This function can be used to reduce the power consumption of the motor if it is holding load against the gravity. 
 */
void servo_2_free (void) //makes servo 2 free rotating
{
 	OCR1BH_reg = 0x03; 
 	OCR1BL_reg = 0xFF; 			//Servo 2 off
}

/**
 * @brief      Function to free the Servo 3
 * 
 * @details    Servo_free functions unlocks the servo motors from the any angle and make them free by giving 100% duty cycle at the PWM. 
 * 			   This function can be used to reduce the power consumption of the motor if it is holding load against the gravity. 
 */
void servo_3_free (void) //makes servo 3 free rotating
{
 	OCR1CH_reg = 0x03; 
 	OCR1CL_reg = 0xFF; 			//Servo 3 off
}

//---------------------------------- MAIN ----------------------------------------------------------------
/**
 * @brief      Main Function
 *
 * @details    First Initializes the Three Servos on Firebird-V,
 * 			   then rotate three servos by 90 degrees.
 */
int main(void)
{
	unsigned char i = 0;
	servo1_pin_config();
	servo2_pin_config();
	servo3_pin_config();
	timer1_init();
	
	for (i = 0; i <90; i++)
	{
		servo_1(i);
		_delay_ms(30);
		servo_2(i);
		_delay_ms(30);
		servo_3(i);
		_delay_ms(30);
	}

	_delay_ms(2000);
	
	servo_1_free(); 
	servo_2_free();
	servo_3_free();
	
	while(1);
}
//---------------------------------- END ------------------------------------------------------------------
