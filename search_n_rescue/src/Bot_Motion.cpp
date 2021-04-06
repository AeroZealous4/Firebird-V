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
#include "Position_Control_Interrupts.h"
#include "atmega_to_esp32_uart.h"
#include "lcd.h"
#include "Color_Sensor.h"
#include "Bot_Motion.h"
//------------------------------ GLOBAL VARIABLES -------------------------------

// To store 8-bit data of left, center and right white line sensors
unsigned char left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data;
// Calibearated sensor data
unsigned char leftValue,centerValue,rightValue;
// To store 8-bit data of 5th IR proximity sensors
unsigned char ir_prox_5_sensor_data;

char str_temp1[] = "Hello, I am a Firebird-------- V\n";
unsigned int count_t = 0;

int leftMin = 4095;
int leftMax = 0;

int rightMin = 4095;
int rightMax = 0;

int centerMin = 4095;
int centerMax = 0;

// To store the eBot's current and Goal location
typedef struct
{
	int x, y;
}tuple;
tuple curr_loc = {4,0}, goal_loc = {4,5};
//Current Plot number
// int curr_dir = 0;
int curr_plot = 0;
// To store the direction in which eBot is currently facing
//N=0,E=1,S=2,W=3, Null = 5
char curr_dir = 'N';
int curr_node = 0;


//---------------------------------- FUNCTIONS ----------------------------------
unsigned char getSensorPercent(int value, int min, int max);

//Returns current heading
//Output: char : 'N','E','S','W','0';
char Get_Curr_Head(void)
{
	return curr_dir;
}
//Returns current Node number
//Output: int
int Get_Curr_Node(void)
{
	return curr_node;
}

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
		right_wl_sensor_data =	ADC_Conversion(right_wl_sensor_channel);
	// ir_prox_5_sensor_data = convert_analog_channel_data(4);

		leftValue = getSensorPercent(left_wl_sensor_data, leftMin, leftMax);
		rightValue = getSensorPercent(right_wl_sensor_data, rightMin, rightMax);
		centerValue = getSensorPercent(center_wl_sensor_data, centerMin, centerMax);
	#ifdef DEBUG
		if(++count_t>1500)
		{
		sprintf(str_temp1,"Abs: %d %d %d \n",left_wl_sensor_data,center_wl_sensor_data,right_wl_sensor_data);
		uart_send_string(str_temp1);
		sprintf(str_temp1,"Cal: %d %d %d \n",leftValue,centerValue,rightValue);
		uart_send_string(str_temp1);
		if(count_t > 1502)
			count_t = 0;
		}
	#endif

	
}
void MinMax(void)
{
	// put the robot down on the field; push  This code will overwrite 
	// each sensor’s min & max from above with your
	// real-world values
	Update_Read();

	if(left_wl_sensor_data > leftMax)
		leftMax = left_wl_sensor_data;
	if(left_wl_sensor_data < leftMin)
		leftMin = left_wl_sensor_data;
	if(center_wl_sensor_data > centerMax)
		centerMax = center_wl_sensor_data;
	if(center_wl_sensor_data < centerMin)
		centerMin = center_wl_sensor_data;

	if(right_wl_sensor_data  > rightMax)
		rightMax = right_wl_sensor_data; 
	if(right_wl_sensor_data  < rightMin)
		rightMin = right_wl_sensor_data; 
}
void calibrate(void) 
{
	//Ref: https://renegaderobotics.org/line-tracker-field-calibration-code/
	// int t = 
	Angle_Rotated(1);
	right();
	velocity(125,125);
   	while(Angle_Rotated(0)<90)
	   MinMax();

	// t = 
	Angle_Rotated(1);
   	left();

	while(Angle_Rotated(0)<180)
	   MinMax();
	
	// t =
	Angle_Rotated(1);
   	right();

	while(Angle_Rotated(0)<90)
	   MinMax();

	stop();
}
// create function used by auton that converts incoming sensor
// data to a 0-to-100 scale so you’re using %s instead of raw
// numbers
unsigned char getSensorPercent(int value, int min, int max) {
   int return_val = ((value - min) * 100)/(max - min);

   // Checks to see if for some reason the % value is 
   // negative or > 100 if so, cap it at 0 or 100 and sends 
   // that back to the auton code
   // Accounts for pre-auton calibration being slightly off
   // from true min & max

   if(return_val < 0)
      return 0;
   if(return_val > 100)
      return 100;

   // otherwise send the return_val calculated above back to
   // the auton routine
   return (unsigned char) return_val;
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
	float k_p = 0.5,k_d = 0.002;
	static float error_d = 0;//line_old=0.0 ;
	Update_Read();
	//Print value

	//Location of line w.r.t. body of robot (-1 to 1), if it is on center then 0
	// float line_loc = ((float)(-left_wl_sensor_data + right_wl_sensor_data ))/((float)(left_wl_sensor_data + center_wl_sensor_data + right_wl_sensor_data + 1.0)) ;
	float line_loc = ((float)(-leftValue+ rightValue ))/((float)(leftValue + centerValue + rightValue + 1.0)) ;

	// if(left_wl_sensor_data < wl_sen_th_l && center_wl_sensor_data < wl_sen_th_l && right_wl_sensor_data < wl_sen_th_l)
	// 	line_loc = line_old;
	// else
	// 	line_old = line_loc;

	#ifdef DEBUG
		if(count_t > 1500)
		{
		// printf("\n\tW Line loc:%f",line_loc);
		sprintf(str_temp1,"\tW Line loc: %d\t",(int) (line_loc *100));
		uart_send_string(str_temp1);
		}
	#endif

	error_d -= line_loc*255.0;
	error_d *= k_d;
	//line_loc is nothing but a error 
	int error = (int) (line_loc*255.0*k_p);
	int error_f;

	error_f =error + (int) error_d;
	error_d = line_loc*255.0;


	int R = Cruise_Vel - error_f;
	int L = Cruise_Vel + error_f;

	if(R>255)
		R =255;
	if(R<0)
		R=0;
	if(L>255)
		L =255;
	if(L<0)
		L=0;

	// char str[] ="asdfkjbasfghisud";
	// sprintf(str,"%d %d",L,R);
	// lcd_string(1, 1,str);
	#ifdef DEBUG
		if(count_t > 1500)
		{
			sprintf(str_temp1,"L,R:%d %d\n",L,R);
			uart_send_string(str_temp1);
			count_t = 0;
		}
	#endif
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
bool onleft, onright,oncenter;
unsigned char state,state_inv;
void forward_wls1(unsigned char node)
{

	unsigned char invert = 0;
    // int flag = 0;
	int l_flag = 0;
    for (int i = 0; i < node; i++)
	{
    	// flag = 0;
		while(center_wl_sensor_data > wl_sen_th && (left_wl_sensor_data > wl_sen_th || right_wl_sensor_data > wl_sen_th) == 0 )
		{
			Update_Read();
			onleft = (left_wl_sensor_data < wl_sen_th_l);
			onright = (right_wl_sensor_data < wl_sen_th_l);
			oncenter = (center_wl_sensor_data < wl_sen_th_l);

			if(invert == 1)
			{
				onleft ^= invert;
				onright ^= invert;
				oncenter ^= invert;
			}


			state = onleft<<2 | oncenter<<1 | onright;
			// printf("ON: %d, %d, %d: %d\n",onleft, oncenter, onright, state);
			switch (state) {
				case 0:
					// forward_mm(95);
					forward();
					velocity(125,175);
					// flag = 1;
					break;
				case 1:
					// LB; CB; RW; Left is just inside the line
					soft_left();
					l_flag = 1;
					velocity(0,125);
					_delay_ms(50);
					break;
				case 2:
						forward();
						velocity(170,170);
						_delay_ms(50);
						break;
					
				case 3:
					// LB; CW; RW; Center is out of line; Left on line
					left();
					l_flag = 1;
					velocity(125,175);
					_delay_ms(50);
					break;
				case 4:
					// LW; CB; RB; Right is just inside the line
					soft_right();
					l_flag = 0;
					velocity(125,0);
					_delay_ms(50);
					break;
				case 5:
					// LW; CB; RW; Just center is on the line; Move forward
					if(invert == 1)
					{
						forward();
						velocity(128,128);
						_delay_ms(100);
					}
					else
					{
						
						forward();
						velocity(175,175);
						_delay_ms(100);
					}
					break;
				case 6:
					// LW; CW; RB; Center is out of line; Right on line
					right();
					l_flag = 0;
					velocity(175,125);
					_delay_ms(50);
					break;
				case 7:
					// LW; CW; RW; All sensors out of line
					// If moving right previously, then move left and vice versa
					if(invert == 1)
					{
						if(l_flag == 0){
							right();
							velocity(10,10);
							_delay_ms(60);
						}
						else{
							left();
							velocity(10,10);
							_delay_ms(60);
						}
						break;
					}
					else{
					    if(l_flag == 0){
						right();
						velocity(125,125);
						_delay_ms(5);
					    }
					    else{
						left();
						velocity(125,125);
						_delay_ms(5);
					    }
					}
					break;
				default:
					break;
			}
		}
	}
	forward_mm(95);
   stop();
}

void forward_wls(unsigned char node)
{
	forward ();
	for (int i = 0; i < node; i++)
	{
		while(1)
		{
			PID();
			// if(center_wl_sensor_data > wl_sen_th && (left_wl_sensor_data > wl_sen_th || right_wl_sensor_data > wl_sen_th))
			if(center_wl_sensor_data > wl_sen_th_cal && (left_wl_sensor_data > wl_sen_th_cal || right_wl_sensor_data > wl_sen_th_cal))
				{
					stop();
					break; //Next node reached
				}
		}
		velocity(Cruise_Vel,Cruise_Vel);
		forward_mm(95);
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
	velocity(Cruise_Vel0, Cruise_Vel0);	
	left_degrees(65);
	velocity (Cruise_Vel0, Cruise_Vel0);
	left();
	// _delay_ms(turn_delay);	//Tune the delay
	while(1)
		{
			Update_Read();
			if(center_wl_sensor_data > wl_sen_th)
				{
					stop();
					break; //Next node reached
				}
		}
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
	velocity (Cruise_Vel0, Cruise_Vel0);	
	right_degrees(65);
	velocity (Cruise_Vel0, Cruise_Vel0);
	right();
	// _delay_ms(turn_delay);	//Tune the delay
	while(1)
		{
			Update_Read();
			if(center_wl_sensor_data > wl_sen_th)
				{
					stop();
					break; //Next node reached
				}
		}
}

/*
*
* Function Name: turn_head(const char * Head)
* Input: char Head: desired direction
*	Obsolete: const char * : Pointer pointing to const char string, Desired direction
* Output: void
* Logic: Compares current heading and desired heading and rotate accordingly
*/
void turn_head(char Head)
{
//	char H = 'N';
//	H = Head[0];
	char H = '0';
	H =Head;
	if(H==curr_dir || H=='0') //5 = '0'
		return;

	switch(curr_dir)
	{
		case 'N':
				switch(H)
				{
					case 'E':
							  right_turn_wls();
								break;
					case 'S':
							right_degrees(90); //Left turn by 90Degreesright_turn_wls();
							right_turn_wls();
							  break;
					case 'W':
						left_turn_wls();
							  break;
				}
				break;
		case 'E':
				switch(H)
				{
					case 'N':
						left_turn_wls();
								break;
					case 'S':
							right_turn_wls();
							  break;
					case 'W':
							right_degrees(90); //right_turn_wls();
							  right_turn_wls();
							  break;
				}
				break;
		case 'S':
				switch(H)
				{
					case 'W':
							right_turn_wls();
								break;
					case 'N':
							right_degrees(90); //right_turn_wls();
							  right_turn_wls();
							  break;
					case 'E':
						left_turn_wls();
							  break;
				}
				break;
		case 'W':
			switch(H){
					case 'N':
						right_turn_wls();
								break;
					case 'E':
							right_degrees(90); //right_turn_wls();
							right_turn_wls();
							  break;
					case 'S':
						left_turn_wls();
							  break;
				}
				break;
	}

	curr_dir = H;
}

/*
*
* Function Name: turn_head_to_plot(char Head)
* Input: char Head: desired direction which is center of a plot
* Obsolete: const char * : Pointer pointing to const char string, Desired direction
* Output: void
* Logic: Compares current heading and desired heading and rotate accordingly
*/
void turn_head_to_plot(char Head)
{
//	char H = 'N';
//	H = Head[0];
	char H = '0';
	H =Head;
	if(H==curr_dir || H=='0') //5 = '0'
		return;

	switch(curr_dir)
	{
		case 'N':
				switch(H)
				{
					case 'E':
							  right_degrees(90);//right_turn_wls();
								break;
					case 'S':
							right_degrees(90); //Left turn by 90Degreesright_turn_wls();
							right_degrees(90); //right_turn_wls();
							  break;
					case 'W':
							left_degrees(90);
							  break;
				}
				break;
		case 'E':
				switch(H)
				{
					case 'N':
						left_degrees(90);//left_turn_wls();
								break;
					case 'S':
							right_degrees(90); //right_turn_wls();
							  break;
					case 'W':
							right_degrees(90); //right_turn_wls();
							right_degrees(90); //  right_turn_wls();
							  break;
				}
				break;
		case 'S':
				switch(H)
				{
					case 'W':
							right_degrees(90); //right_turn_wls();
								break;
					case 'N':
							right_degrees(90); //right_turn_wls();
							right_degrees(90); //  right_turn_wls();
							  break;
					case 'E':
						left_degrees(90);//left_turn_wls();
							  break;
				}
				break;
		case 'W':
			switch(H){
					case 'N':
						right_degrees(90); //right_turn_wls();
								break;
					case 'E':
							right_degrees(90); //right_turn_wls();
							right_degrees(90); //right_turn_wls();
							  break;
					case 'S':
						left_degrees(90);//left_turn_wls();
							  break;
				}
				break;
	}

	curr_dir = H;
}