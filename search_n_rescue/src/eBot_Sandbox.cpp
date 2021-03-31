/*
 * eBot_Sandbox.cpp
 *
 *  Created on: 21-Mar-2021
 *      Author: Akshay Pampatwar (183079007)
 */


//---------------------------------- INCLUDES -----------------------------------
// #define DEBUG 1	//Uncomment for debug msg from esp32 to laptop
#include "eBot_Sandbox.h"
#include <stdio.h>

// //------------------------------ GLOBAL VARIABLES -------------------------------

char str[] = "Hello, I am a Firebird-------- V\n";

/**
 * @brief Initializes all peripherals reuired for project
 */
void init_all_peripherals(void)
{
	adc_port_config();					// Initialize the ADC port
	adc_init();							// Initialize the ADC
	lcd_port_config();					// Initialize the LCD port
	lcd_init();							// Initialize the LCD
	init_color_sensor();				//Initialize colour sensor
	filter_clear();	//
	//Related to motors
	motors_pin_config();
	pwm_pin_config();
	position_encoder_pin_config();
	position_encoder_interrupt_config();

	timer5_init(); //For PWM

	Comm_ESP32_init(); //Init timer3 for time keeping
	//Uart Initialization
	uart_init1();	//UART3 of atmega
}

void Task_1B(void)
{
	forward_wls(1); //Goes to next node
	sprintf(str,"1st Node");
	uart_send_string(str);
	forward_wls(1); //Goes to next node i.e. Mid point of required node
	sprintf(str,"Mid Node");
	uart_send_string(str);
	velocity(Cruise_Vel0,Cruise_Vel0);
	left_degrees(90); //Left turn by 90Degrees
	sprintf(str,"90 Degree Left turn");
	uart_send_string(str);
	velocity(Cruise_Vel,Cruise_Vel);
	forward_mm(180);
	sprintf(str,"Center of block reached");
	uart_send_string(str);

	char Type_Inj = Sense_Color();
	uart3_SendByte(Type_Inj); //Scan and send a required colour
	lcd_clear();
	lcd_wr_char_EE(Type_Inj);

	if(Type_Inj == 'R')
		Set_Major();
	else if(Type_Inj == 'G')
		Set_Minor();
	else
		Set_NoInjury();

	back_mm(180);
	sprintf(str,"Back on line");
	uart_send_string(str);

	left_turn_wls();
	sprintf(str,"Left Turn");
	uart_send_string(str);

	forward_wls(1); //Goes to next node
	sprintf(str,"1st Node");
	uart_send_string(str);

	forward_wls(1); //Goes to next node
	sprintf(str,"Start Location");
	uart_send_string(str);
	sprintf(str,"Finished");
	lcd_string_EE(str);
}

/**
 * @brief  Checks latest msg coming from ESP32 and trigger respective task
 */
/**
 * @brief      Executes the logic to achieve the aim of Project
 */
void Controller(void)
{
	// int return_code;

	init_all_peripherals();
	calibrate();
	Update_Command();

	if(Is_Command())
	{
		Cmd_Accepted(); //Send ack that cmd is accepted
		Task_1B();		//Complete task related to 1B
		Task_Complete(); 	
	}


	while ((1));
	{
		/* code */
	}
	
/*

	#ifdef DEBUG
		sprintf(str,"Cal Done, Min Max\n");
		uart_send_string(str);
		sprintf(str,"%d %d, %d %d,%d %d\n",leftMin,leftMax,centerMin,centerMax,rightMin,rightMax);
		uart_send_string(str);
	#endif
*/
}
