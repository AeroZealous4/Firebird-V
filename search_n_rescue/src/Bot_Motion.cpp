/*
 * eBot_Sandbox.cpp
 *
 *  Created on: 11-Jan-2021
 *      Author: TAs of CS 684 Spring 2020
 */


//---------------------------------- INCLUDES -----------------------------------

// #include "eBot_Sandbox.h"
#include <stdlib.h>
#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>					// Standard AVR Delay Library
#include <stdio.h>						// Standard C Library for standard input output
#include <math.h> 						//included to support power function
#include <string.h>
#include "ADC_Sensor_Display_on_LCD.h"

// #include "shortest_path.h"

#define turn_delay 2000
#define wl_sen_th 21	//Line Exists, sen_read > wl_sen_th
#define wl_sen_th_l 12	//Line Does not exists  sen_read < wl_sen_th_l
#define ir_sen_th 200
//------------------------------ GLOBAL VARIABLES -------------------------------

// To store 8-bit data of left, center and right white line sensors
unsigned char left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data;

// To store 8-bit data of 5th IR proximity sensors
unsigned char ir_prox_5_sensor_data;

// To store the eBot's current and Goal location
typedef struct
{
	int x, y;
}tuple;
tuple curr_loc = {4,0}, goal_loc = {4,5};

// To store the direction in which eBot is currently facing
// char dir_flag = "N";
int dir_flag = 1; //1: N, 2: E, 3: S, 4: W, 0: "0"
int curr_node = 0;


//---------------------------------- FUNCTIONS ----------------------------------


/*
*
* Function Name: Update_Read
* Input: void
* Output: void
* Logic: Updates global sensor variable data with latest reading
* Example Call: 
*
*/
//
void Update_Read(void)
{
	left_wl_sensor_data = ADC_Conversion(left_wl_sensor_channel);
	center_wl_sensor_data = ADC_Conversion(center_wl_sensor_channel);
	right_wl_sensor_data =	ADC_Conversion(rigth_wl_sensor_channel);
	// ir_prox_5_sensor_data = convert_analog_channel_data(4);
}

/*
*
* Function Name: PID
* Output: void
* Logic: Uses PID to trace line properly
* Example Call: 
*
*/
void PID()
{
	float k_p = 1.5,k_d = 0.3;
	static float error_d = 0 ; 
	Update_Read();
	//Location of line w.r.t. body of robot (-1 to 1), if it is on center then 0
	float line_loc = (-left_wl_sensor_data + right_wl_sensor_data )/(left_wl_sensor_data + center_wl_sensor_data + right_wl_sensor_data) ;

	error_d -= line_loc*255;
	error_d *= k_d;
	//line_loc is nothing but a error 
	int error = (int) line_loc*255*k_p;

	error += (int) error_d; 

	int R = 175 +error;
	int L = 175 - error;

	if(R>255)
		R =255;
	if(R<0)
		R=0;
	if(L>255)
		L =255;
	if(L<0)
		L=0;

	velocity(L, R); //Differential velocity
	// set_motor_velocities();
}
/*
*
* Function Name: forward_wls
* Input: node
* Output: void
* Logic: Uses white line sensors to go forward by the number of nodes specified
* Example Call: forward_wls(2); //Goes forward by two nodes
*
*/
//
void forward_wls(unsigned char node)
{
	forward ();
	for (int i = 0; i < node; i++)
	{
		while(1)
		{
			PID();
			if(left_wl_sensor_data > wl_sen_th && center_wl_sensor_data > wl_sen_th && right_wl_sensor_data > wl_sen_th)
				{
					stop();
					break; //Next node reached
				}
		}
	}
}

/*
*
* Function Name: left_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn left until black line is encountered
* Example Call: left_turn_wls();
*
*/
void left_turn_wls(void)
{
	velocity (125, 125);	
	right_degrees(90);
	// left();
	// _delay_ms(turn_delay);	//Tune the delay
	// while(1)
	// 	{
	// 		if(left_wl_sensor_data > wl_sen_th && center_wl_sensor_data > wl_sen_th && right_wl_sensor_data > wl_sen_th)
	// 			{
	// 				stop();
	// 				break; //Next node reached
	// 			}
	// 	}

}

/*
*
* Function Name: right_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn right until black line is encountered
* Example Call: right_turn_wls();
*
*/
void right_turn_wls(void)
{
	velocity (125, 125);	
	right_degrees(90);
	// right();
	// _delay_ms(turn_delay);	//Tune the delay
	// while(1)
	// 	{
	// 		if(left_wl_sensor_data > wl_sen_th && center_wl_sensor_data > wl_sen_th && right_wl_sensor_data > wl_sen_th)
	// 			{
	// 				stop();
	// 				break; //Next node reached
	// 			}
	// 	}
}

/*
*
* Function Name: turn_head(const char * Head)
* Input: Desired heading
* Output: void
* Logic: Compares current heading and desired heading and rotate accordingly
*/
void turn_head(int Head)
{
	int H;
	H = Head;
	if(H==dir_flag || H==0)
		return;

	switch(dir_flag)
	{
		case 1://N
				switch(H)
				{
					case(2): //E
								right_turn_wls();
								break;
					case 3: //S
								right_turn_wls();
							  	right_turn_wls();
							  	break;
					case 4: //WW
								left_turn_wls();
							  	break;
				}
				break;
		case 2:	//E
				switch(H)
				{
					case 1: //N
								left_turn_wls();
								break;
					case 3: //S
								right_turn_wls();
							  	break;
					case 4: //W
								right_turn_wls();
							  	right_turn_wls();
							  	break;
				}
				break;
		case 3:	//S
				switch(H)
				{
					case 4: //W
								right_turn_wls();
								break;
					case 1: //N
								right_turn_wls();
							  	right_turn_wls();
							  	break;
					case 2: //E
								left_turn_wls();
							  	break;
				}
				break;
		case 4:	//W
				switch(H)
				{
					case 1: //N
								right_turn_wls();
								break;
					case 2: //E
								right_turn_wls();
							  	right_turn_wls();
							  	break;
					case 3: 
								left_turn_wls();
							  	break;
				}
				break;
	}	

	dir_flag = H;
}
/**
 * @brief      Executes the logic to achieve the aim of Lab 4
 */
void traverse_line_to_goal(void)
{
	int Head;
	dijkstra(dest);
	forward_wls(1); //Goes to zero th node
	curr_node = 0;

	while(curr_node != dest)
	{
		Head = Next_Dir(curr_node,parent[curr_node]);
		turn_head(Head);
		Update_Read();

		if(ir_prox_5_sensor_data<200)
			Adj_Update(curr_node,parent[curr_node]);
		else
		{
			forward_wls(1); //Goes to next node
			curr_node = parent[curr_node];
		}
	}
}

