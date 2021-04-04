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

	Comm_ESP32_init(); //Init timer3 for time keeping and buzzer
	//Uart Initialization
	uart_init1();	//UART3 of atmega
}

void Task_1B(void)
{
	forward_wls(1); //Goes to next node
	sprintf(str,"1st Node\n");
	uart_send_string(str);
	forward_wls(1); //Goes to next node i.e. Mid point of required node
	sprintf(str,"Mid Node\n");
	uart_send_string(str);
	velocity(Cruise_Vel0,Cruise_Vel0);
	left_degrees(90); //Left turn by 90Degrees
	sprintf(str,"90 Degree Left turn\n");
	uart_send_string(str);
	velocity(Cruise_Vel,Cruise_Vel);
	forward_mm(180);
	sprintf(str,"Center of block reached\n");
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
	sprintf(str,"Back on line\n");
	uart_send_string(str);

	left_turn_wls();
	sprintf(str,"Left Turn\n");
	uart_send_string(str);

	forward_wls(1); //Goes to next node
	sprintf(str,"1st Node\n");
	uart_send_string(str);

	forward_wls(1); //Goes to next node
	sprintf(str,"Start Location\n");
	uart_send_string(str);
	sprintf(str,"Finished\n");
	lcd_string_EE(str);
}

/**
 * @brief  Scan arena and executes any task if encountered before that
 */
bool Task_2B(void)
{
	// bool Is_Finished = false;
	static bool In_Path = false; // If bot is traveling to desired loc then true 
	static int Current_Node = 0 ;
	static int Nxt_Node = 0 ;
	static int Dest_Node = 66;

	if( In_Path == false)
	{
		if(Inj_at_Plot(Next_Plot_to_Scan()) == '0')
		{
			Dest_Node = Min_Dist_Node_Fr_Plot ( Next_Plot_to_Scan(),Current_Node);
			dijkstra(Dest_Node);	//Updates path to travel to Dest node
			In_Path = true;
		}
		else
		{
			Plot_Scan_Compl(); //Current plot is already scanned
			In_Path = false;
		}
		
	}
	else if( In_Path == true)
	{
		Nxt_Node = Next_Node(Current_Node);

		if( ~(Nxt_Node == -1) )
		{
			turn_head( Next_Dir(Current_Node, Nxt_Node) );	//Rotate to desired direction
			forward_wls(1);	//Move to next node
			Current_Node = Nxt_Node;
			In_Path = true;
		}
		else if(Next_Plot_to_Scan()!=66)//Plot reached
		{
			turn_head_to_plot(Get_Dir_Plot());

			forward_mm(180);
				// sprintf(str,"Center of block no: %d reached\n",Next_Plot_to_Scan());
				// uart_send_string(str);

			char Type_Inj = Sense_Color();	//Scan Injury

			if(Type_Inj == 'R')
				sprintf(str,"Block no: %d with Major Injury\n",Next_Plot_to_Scan());
			else if(Type_Inj == 'G')
				sprintf(str,"Block no: %d with Minor Injury\n",Next_Plot_to_Scan());
			else
				sprintf(str,"Block no: %d with No Injury\n",Next_Plot_to_Scan());
			
			Plot_Scan_Compl();

			back_mm(180);

			In_Path = false;
		}
		else if(Next_Plot_to_Scan()==66)//Goal reached
		{
			Plot_Scan_Compl();
			In_Path = false;
			return true;
		}
	}
	// Is_Finished = true;
	return false;
}
/**
 * @brief      Executes the logic to achieve the aim of Project
 */
void Controller(void)
{
	// int return_code;

	init_all_peripherals();
	calibrate();
	// char next = Next_Dir(4, 5); 

	sprintf(str,"Cal Done\n");
	uart_send_string(str);

	Update_Command();

	Task_1B();		//Complete task related to 1B
	// int i=1; 
	// while (1)
	// {
	// 		/* code */
	// 	Update_Command();
	// 	// _delay_ms(100);	//Delay
	// 	if(Is_Command())
	// 	{
	// 		Cmd_Accepted(); //Send ack that cmd is accepted
	// 		Task_1B();		//Complete task related to 1B
	// 		_delay_ms(1000); //1 Sec delay
	// 		Task_Complete(); //Sends ack to ESP32	
	// 		// i = 0;
	// 		break;
	// 	}
	// 	// if(Task_2B())
	// 	// 	break;
	// }

//Operation Search and Rescue finished successfully
	lcd_clear();
	sprintf(str,"Scan Finished\n");
	lcd_string_EE(str);
	while(1);
	
/*

	#ifdef DEBUG
		sprintf(str,"Cal Done, Min Max\n");
		uart_send_string(str);
		sprintf(str,"%d %d, %d %d,%d %d\n",leftMin,leftMax,centerMin,centerMax,rightMin,rightMax);
		uart_send_string(str);
	#endif
*/
}
