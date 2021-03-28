/*
 * eBot_Sandbox.cpp
 *
 *  Created on: 11-Jan-2021
 *      Author: TAs of CS 684 Spring 2020
 */


//---------------------------------- INCLUDES -----------------------------------
#define DEBUG_ESS

// #include "eBot_Sandbox.h"
#include "ADC_Sensor_Display_on_LCD.h"
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
//#include "shortest_path.h"

//--------------------------------------------------------------------------------------
#define V 50
#define dest 49 //Goal node
int parent[V];
int graph1[V][V] = {	{0,0,0,0,5,5,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,10,0,0,0,0,0,0,10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,10,0,10,0,0,0,0,0,0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,10,0,10,0,0,0,0,0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{5,0,0,10,0,10,0,0,0,0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{5,0,0,0,0,0,10,0,0,0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,10,0,10,0,0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,10,0,10,0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,10,0,0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,10,0,0,0,0,0,0,0,0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,10,0,0,0,0,0,0,10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,10,0,0,0,0,0,0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,10,0,0,0,0,0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,10,0,0,0,0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,10,0,0,0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,10,0,0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,10,0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,10, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0,5},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0,5},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0},
						{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,5,5, 0, 0, 0, 0}

					};


int minDistance(int dist[],bool sptSet[]);
void dijkstra(int graph[V][V], int src);
void Adj_Update(int i, int j);
int Next_Dir(int i, int j);

//---------------------------------------------------------------------------------------
#ifdef NON_MATLAB_PARSING
	#define turn_delay 350//450
	#define fwd_delay 500//100:400//225
	#define wl_sen_th 200	//Line Exists, sen_read > wl_sen_th
	#define wl_sen_th_l 20	//Line Does not exists  sen_read < wl_sen_th_l
	#define ir_sen_th 200
#else
	#define turn_delay 25
	#define fwd_delay 250
	#define wl_sen_th 21	//Line Exists, sen_read > wl_sen_th
	#define wl_sen_th_l 12	//Line Does not exists  sen_read < wl_sen_th_l
	#define ir_sen_th 200
#endif

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
//N=0,E=1,S=2,W=3, Null = 5
int dir_flag = 0;//'N';
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
	right_wl_sensor_data =	ADC_Conversion(right_wl_sensor_channel);
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
	float k_p = 0.5,k_d = 0.03;
	static float error_d = 0,line_old=0.0 ;
	Update_Read();
	//Print value

	//Location of line w.r.t. body of robot (-1 to 1), if it is on center then 0
	float line_loc = ((float)(-left_wl_sensor_data + right_wl_sensor_data ))/((float)(left_wl_sensor_data + center_wl_sensor_data + right_wl_sensor_data + 1.0)) ;

	if(left_wl_sensor_data < wl_sen_th_l && center_wl_sensor_data < wl_sen_th_l && right_wl_sensor_data < wl_sen_th_l)
		line_loc = line_old;
	else
		line_old = line_loc;

	#ifdef DEBUG
		printf("\n\tW Line loc:%f",line_loc);
	#endif

	error_d -= line_loc*255.0;
	error_d *= k_d;
	//line_loc is nothing but a error 
	int error = (int) (line_loc*255.0*k_p);
	int error_f;

	error_f =error + (int) error_d;
	error_d = line_loc*255.0;

	#ifdef DEBUG
		printf("\n\tW Error:%d\t%d ",error,error_f);
	#endif

	velocity(90+error_f, 90-error_f); //Differential velocity
	forward();
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
	for (int i = 0; i < (int) node; i++)
	{
		while(1)
		{
			PID();
			#ifdef DEBUG
				printf("\n\tW Sensors:%3d\t%3d\t%3d",left_wl_sensor_data,center_wl_sensor_data,right_wl_sensor_data);
			#endif
			if(left_wl_sensor_data > wl_sen_th && center_wl_sensor_data > wl_sen_th && right_wl_sensor_data > wl_sen_th)
				{
					#ifdef DEBUG
						printf("\n\tW Node Reached: Centering Started");
					#endif
					_delay_ms(fwd_delay);	//Tune the delay

					stop();
					set_motor_velocities();
					break; //Next node reached
				}
		}

//		#ifdef DEBUG
//			printf("\n\tNode: %d",node);
//		#endif
	}
#ifdef DEBUG_ESS
	printf("\n\tOne forward travel complete-----------------------------");
#endif
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
#ifdef DEBUG
	printf("\n\tLeft Turn");
#endif
	left();
	set_motor_velocities();
	_delay_ms(turn_delay);	//Tune the delay


	while(1)
		{
			Update_Read();
			if(left_wl_sensor_data < wl_sen_th && center_wl_sensor_data > wl_sen_th && right_wl_sensor_data < wl_sen_th)
				{
					stop();
					set_motor_velocities();
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
#ifdef DEBUG
	printf("\n\tRight Turn");
#endif
	right();
	set_motor_velocities();
	_delay_ms(turn_delay);	//Tune the delay

//	stop();
//	set_motor_velocities();
//	_delay_ms(4000);	//Tune the delay

//	right();
	while(1)
		{
			Update_Read();
			if(left_wl_sensor_data < wl_sen_th && center_wl_sensor_data > wl_sen_th && right_wl_sensor_data < wl_sen_th)
				{
					stop();
					set_motor_velocities();
					break; //Next node reached
				}
		}
}

/*
*
* Function Name: turn_head(const char * Head)
* Input: const char * : Poointer pointing to const char string
* Output: void
* Logic: Compares current heading and desired heading and rotate accordingly
*/
void turn_head(int Head)
{
//	char H = 'N';
//	H = Head[0];
	int H = 5;
	H =Head;
	if(H==dir_flag || H==5) //5 = '0'
		return;

	switch(dir_flag)
	{
		case 0://'N':
				switch(H)
				{
					case 1://'E':
							  right_turn_wls();
								break;
					case 2://'S':
							right_turn_wls();
							  right_turn_wls();
							  break;
					case 3://'W':
						left_turn_wls();
							  break;
				}
				break;
		case 1://'E':
				switch(H)
				{
					case 0://'N':
						left_turn_wls();
								break;
					case 2://'S':
							right_turn_wls();
							  break;
					case 3://'W':
								right_turn_wls();
							  right_turn_wls();
							  break;
				}
				break;
		case 2://'S':
				switch(H)
				{
					case 3://'W':
							right_turn_wls();
								break;
					case 0://'N':
							right_turn_wls();
							  right_turn_wls();
							  break;
					case 1://'E':
						left_turn_wls();
							  break;
				}
				break;
		case 3://'W':
			switch(H){
					case 0://'N':
						right_turn_wls();
								break;
					case 1://'E':
						right_turn_wls();
							  right_turn_wls();
							  break;
					case 2://'S':
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
	int  Head;
	dijkstra(graph1, dest);
	forward_wls(1); //Goes to zero th node

	curr_node = 0;
//	_delay_ms(4000);	//Tune the delay
//	forward();
//	set_motor_velocities();
//	_delay_ms(fwd_delay);	//Tune the delay
//	stop();
//	set_motor_velocities();

//	right_turn_wls();

//	while(1);
	while(curr_node != dest)
	{
		Head = Next_Dir(curr_node,parent[curr_node]);
		Update_Read();

		if(ir_prox_5_sensor_data<200)
			Adj_Update(curr_node,parent[curr_node]);
		else
		{
			#ifdef DEBUG_ESS
				printf("\n\t Current Node: %d",curr_node);
			#endif
				curr_node = parent[curr_node];
			#ifdef DEBUG_ESS
				printf("\n\t Next Node: %d",curr_node);
			#endif
			turn_head(Head);
			forward_wls(1); //Goes to next node
//			curr_node = parent[curr_node];
//			#ifdef DEBUG_ESS
//				printf("\n\t Next Node: %d",curr_node);
//			#endif
		}
	}
}



///------------------------------------------------------
// A utility function to find the
// vertex with minimum distance
// value, from the set of vertices
// not yet included in shortest
// path tree
int minDistance(int dist[],
				bool sptSet[])
{

	// Initialize min value
	int min = INT_MAX, min_index;

	for (int v = 0; v < V; v++)
		if (sptSet[v] == false &&
				dist[v] <= min)
			min = dist[v], min_index = v;

	return min_index;
}

// Function to print shortest
// path from source to j
// using parent array
// void printPath(int parent[], int j)
// {

// 	// Base Case : If j is source
// 	if (parent[j] == - 1)
// 		return;

// 	printPath(parent, parent[j]);

// 	printf("%d ", j);
// }
void printPath( int j)
{

	// Base Case : If j is source
	if (parent[j] == - 1)
		return;

	printPath(parent[j]);

	printf("%d ", j);
}

// A utility function to print
// the constructed distance
// array
// int printSolution(int dist[], int n,
// 					int parent[])
// {
// 	int src = 0;
// 	printf("Vertex\t Distance\tPath");
// 	for (int i = 1; i < V; i++)
// 	{
// 		printf("\n%d -> %d \t\t %d\t\t%d ",
// 					src, i, dist[i], src);
// 		printPath(parent, i);
// 	}
// }

// Funtion that implements Dijkstra's
// single source shortest path
// algorithm for a graph represented
// using adjacency matrix representation
void dijkstra(int graph[V][V], int src)
{

	// The output array. dist[i]
	// will hold the shortest
	// distance from src to i
	int dist[V];

	// sptSet[i] will true if vertex
	// i is included / in shortest
	// path tree or shortest distance
	// from src to i is finalized
	bool sptSet[V];

	// Parent array to store
	// shortest path tree
	// int parent[V];

	// Initialize all distances as
	// INFINITE and stpSet[] as false
	for (int i = 0; i < V; i++)
	{
		parent[0] = -1;
		dist[i] = INT_MAX;
		sptSet[i] = false;
	}

	// Distance of source vertex
	// from itself is always 0
	dist[src] = 0;

	// Find shortest path
	// for all vertices
	for (int count = 0; count < V - 1; count++)
	{
		// Pick the minimum distance
		// vertex from the set of
		// vertices not yet processed.
		// u is always equal to src
		// in first iteration.
		int u = minDistance(dist, sptSet);

		// Mark the picked vertex
		// as processed
		sptSet[u] = true;

		// Update dist value of the
		// adjacent vertices of the
		// picked vertex.
		for (int v = 0; v < V; v++)

			// Update dist[v] only if is
			// not in sptSet, there is
			// an edge from u to v, and
			// total weight of path from
			// src to v through u is smaller
			// than current value of
			// dist[v]
			if (!sptSet[v] && graph[u][v] &&
				dist[u] + graph[u][v] < dist[v])
			{
				parent[v] = u;
				dist[v] = dist[u] + graph[u][v];
			}
	}

	// print the constructed
	// distance array
	// printSolution(dist, V, parent);
}

// A utility function to update adj mat of graph and parents vector
// Input: (int i, int j): Obstacle between i and jth node (Order of i,j does not matter)
// Output: None
// Description: Fn takes undirected edge which is cut off due to obstacle and removes it
//				from Adj matrix of graph and updates new shortest path i.e. parent vector
void Adj_Update(int i, int j)
{
	if( j < 50 && i< 50 && j>=0 && i>=0)
	{
		graph1[i][j] = 0;
		graph1[j][i] = 0;

		dijkstra(graph1, dest);
	}
}

// A utility function to gives next direction for movement as
// Input: (int i, int j): Location of jth node w.r.t. ith node
// Output: (unsigned char): N, E, S, W as output, "0" if no dir is possible
// Description: Takes current node i and j next node as input and gives direction in which
//				Bot should move as a output
int Next_Dir(int i, int j)
{
	int x_i,y_i,x_j,y_j; //Coordinates
	// char * N ="N",S="S",W="W",E="E",o="0";

	if(i == 0 || i== 49 || j== 0 || j== 49)
	{
		if( (i==0 && j==4) || (i==5 && j==0) )
			return 3;//"W";
		else if( (i==0 && j==5) || (i==4 && j==0) )
			return 1;//"E";
		else if( (i==44 && j==49) || (i==49 && j==45) )
			return 1;//"E";
		else if( (i==45 && j==49) || (i==49 && j==44) )
			return 3;//"W";
		else
			return 5;//"0";
	}

	i--;
	j--;
	x_i = i%8;
	y_i = i/8;
	x_j = j%8;
	y_j = j/8;

	if( ((x_j - x_i)==1 || (x_j - x_i)== -1) && ((y_j - y_i)== 1 || (y_j - y_i)== -1 ) )
	{
		// printf("(%d,%d) (%d,%d)",x_i,y_i,x_j,y_j );
		// printf("\nWrong direction");
		return 5;//"0";
	}
	if( (x_j - x_i)==1 )
		return 1;//"E";
	else if((x_j - x_i)== -1 )
		return 3;//"W";
	else if((y_j - y_i)== 1 )
		return 0;//"N";
	else if((y_j - y_i)== -1 )
		return 2;//"S";
	else
		return 5;//"0";
}
