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
//Sets curr node to new value
//Input: int
void Set_Curr_Node(int temp)
{
	curr_node = temp;
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
	float k_p = 0.60,k_d = 0.05; //k_p = 0.50,k_d = 0.002; Working k_p value
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
//Debug function for cal debris
// void Deb_WL_Debris(void)
// {
// 	Update_Read();
// 	sprintf(str_temp1,"Cal:%d %d %d\n",leftValue,centerValue,rightValue);
// 	uart_send_string(str_temp1);

// 	sprintf(str_temp1,"WL:%d %d %d\n",left_wl_sensor_data,center_wl_sensor_data,right_wl_sensor_data);
// 	uart_send_string(str_temp1);

// 	_delay_ms(3000);
// }

bool debris_detection(void)
{
	bool debris_detected = false;
	bool onleft, onright, oncenter;
	unsigned char state;
	static int Pol_Count = 0;



	// for(int i=0; i<200;i++)
	// {

	//Update_Read();

//	onleft = (left_wl_sensor_data == 7) || (left_wl_sensor_data == 6);
	onleft = (left_wl_sensor_data <= 9);	
	oncenter = (center_wl_sensor_data <= 9);
//	onright = (right_wl_sensor_data == 7);
	onright = (right_wl_sensor_data <= 9);// || (right_wl_sensor_data == 8);
//	onleft = (left_wl_sensor_data < wl_sen_th_l_cal);
//	oncenter = (center_wl_sensor_data < wl_sen_th_l_cal);
//	onright = (right_wl_sensor_data < wl_sen_th_l_cal);

	state = onleft << 2 | oncenter << 1 | onright;
	if( state == 7 )
	{
		// lcd_numeric_value(2,1,left_wl_sensor_data,3);
		// lcd_numeric_value(2,5,center_wl_sensor_data,3);
		// lcd_numeric_value(2,9,right_wl_sensor_data,3);
		// if(curr_loc.x == 7 && curr_loc.y == 0){
		// 	_delay_ms(1000);
		// }
		// lcd_clear();
		if(Pol_Count++ >= 275)
		{

			debris_detected = true;
			// break;
		}
		else
		{
			debris_detected = false;
		}

	}
	else{
		Pol_Count = 0;
		debris_detected = false;
	}

	// }


	return debris_detected;
}
bool Debris_flag = false;
void forward_wls(unsigned char node)
{
	
	forward ();
	for (int i = 0; i < node; i++)
	{
		while(1)
		{
			// int debris_det = 0;
			
			PID();
			// if(center_wl_sensor_data > wl_sen_th && (left_wl_sensor_data > wl_sen_th || right_wl_sensor_data > wl_sen_th))
			if(center_wl_sensor_data > wl_sen_th_cal && (left_wl_sensor_data > wl_sen_th_cal || right_wl_sensor_data > wl_sen_th_cal))
				{
					stop();
					Debris_flag = false;
					break; //Next node reached
				}
			if(debris_detection())
			{
				stop();
				Debris_flag = true;
				break; //Debris Detected
			}
			
			// else if( (center_wl_sensor_data < wl_sen_th_l_cal) && (left_wl_sensor_data < wl_sen_th_l_cal ) && (right_wl_sensor_data < wl_sen_th_l_cal) )
			// {
			// 	if(debris_det++ >= 1)
			// 	{
			// 		stop();
			// 		Debris_flag = true;
			// 		break; //Debris Detected
			// 	}
			// }
			// else
			// debris_det = 0;
		}
		velocity(Cruise_Vel,Cruise_Vel);
		forward_mm(95);
	}
	
}

bool Is_Debris(void)
{
    if(Debris_flag)
    {

        #ifdef DEBUG
            sprintf(str_temp,"Debris Detected$");
            uart_send_string(str_temp);
        #endif 
        Debris_flag = false;
        return true;
    }
    return false;
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
	left_degrees(50);//65
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
	right_degrees(50);	//65
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