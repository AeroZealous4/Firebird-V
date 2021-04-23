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
#include "Comm_ESP32.h"
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

//--------------------------SACHIN GLOBAL VARIABLES------------------------------------
#define GOAL_X 4
#define GOAL_Y 8


// To store the eBot's current and Goal location
typedef struct
{
	int x, y;
}tuple;

tuple curr_loc = {8,4}, goal_loc = {0,0}, medical_camp_loc = {GOAL_X,GOAL_Y};

// To store the direction in which eBot is currently facing
char dir_flag = 'n';

//To store coordinate updates
int x,y;

//To store reconstructed path
unsigned char reconstructed_path[200];


// North, South, East, West direction vector
int dr[] = {-1,1,0,0};
int dc[] = {0,0,1,-1};

// To store the status of map
char grid_map[9][9] = {
		{'*','*','*','*','*','*','*','*','*'},
		{'*','0','*','0','*','0','*','0','*'},
		{'*','*','*','*','*','*','*','*','*'},
		{'*','0','*','0','*','0','*','0','*'},
		{'*','*','*','*','*','*','*','*','*'},
		{'*','0','*','0','*','0','*','0','*'},
		{'*','*','*','*','*','*','*','*','*'},
		{'*','0','*','0','*','0','*','0','*'},
		{'*','*','*','*','*','*','*','*','*'}
};

char read_color[9][9] = {
		{'n','y','n','y','n','y','n','y','n'},
		{'y','n','y','n','y','n','y','n','y'},
		{'n','y','n','y','n','y','n','y','n'},
		{'y','n','y','n','y','n','y','n','y'},
		{'n','y','n','y','n','y','n','y','n'},
		{'y','n','y','n','y','n','y','n','y'},
		{'n','y','n','y','n','y','n','y','n'},
		{'y','n','y','n','y','n','y','n','y'},
		{'n','y','n','y','n','y','n','y','n'}
};

unsigned char node_numbering[9][9] = {
		{1,2,3,4,5,6,7,8,9},
		{10,11,12,13,14,15,16,17,18},
		{19,20,21,22,23,24,25,26,27},
		{28,29,30,31,32,33,34,35,36},
		{37,38,39,40,41,42,43,44,45},
		{46,47,48,49,50,51,52,53,54},
		{55,56,57,58,59,60,61,62,63},
		{64,65,66,67,68,69,70,71,72},
		{73,74,75,76,77,78,79,80,81}
};

// Prev matrix for row and column to store the parent nodes for backtracking

unsigned char prev[9][9];


unsigned char visited[9][9];


//Node coordinates of each node
struct node_coordinate
{
     unsigned char row;
     unsigned char col;
};

struct node_coordinate node_c[81];

struct Injury
{	
     char inj_type;
     unsigned char row;
     unsigned char col;
     unsigned char plot_no;	
};

struct Injury injury_c[16];

///////////////////////////////////////////////////////////
//Plot midpoint coordinates
struct plot_midpoints
{
	unsigned char midpoints[4];
};

struct plot_midpoints plot_m[16];


///////////////////////////////////////////////////////////
//struct for each node which keeps track of plots that can be serviced from it.
struct servicing                    //
{
	unsigned char plots_covered[2];
};

struct servicing service_m[81];




//---------------------------------- FUNCTIONS --------------------------------------------------------
unsigned char getSensorPercent(int value, int min, int max);


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
		
	//	lcd_numeric_value(2,1,left_wl_sensor_data,1);
	//    lcd_numeric_value(2,5,center_wl_sensor_data,1);
	//    lcd_numeric_value(2,9,right_wl_sensor_data,1);
	 //   _delay_ms(10);
	   // lcd_clear();
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
   return (unsigned char)return_val;
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
					break; //Next node reached
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






//---------------------------------- SACHIN FUNCTIONS ----------------------------------


bool debris_detection(void)
{
	bool debris_detected;
	bool onleft, onright, oncenter;
	unsigned char state;
	Update_Read();
//	onleft = (left_wl_sensor_data == 7) || (left_wl_sensor_data == 6);
	onleft = (left_wl_sensor_data == 8) || (left_wl_sensor_data == 7) || (left_wl_sensor_data == 9);	
	oncenter = (center_wl_sensor_data == 7 || center_wl_sensor_data == 8);
//	onright = (right_wl_sensor_data == 7);
	onright = (right_wl_sensor_data == 7) || (right_wl_sensor_data == 8);
//	onleft = (left_wl_sensor_data < wl_sen_th_l_cal);
//	oncenter = (center_wl_sensor_data < wl_sen_th_l_cal);
//	onright = (right_wl_sensor_data < wl_sen_th_l_cal);

	state = onleft << 2 | oncenter << 1 | onright;


	if( state == 7 )
	{
		lcd_numeric_value(2,1,left_wl_sensor_data,3);
		lcd_numeric_value(2,5,center_wl_sensor_data,3);
		lcd_numeric_value(2,9,right_wl_sensor_data,3);
		_delay_ms(3000);
		lcd_clear();
		debris_detected = true;
	}
	else{
		debris_detected = false;
	}
	return debris_detected;
}


//Turning 90 degrees left using position encoder
void left_90(void)
{
	left_degrees(90);
	velocity(Cruise_Vel0, Cruise_Vel0);
	
}

//Turning 90 degrees right using position encoder
void right_90(void)
{
	right_degrees(90);
	velocity(Cruise_Vel0, Cruise_Vel0);
}








//Function to get the plot injury
char print_color(void) //int plot_no)
{
	char Type_Inj,inj;
	Type_Inj = Sense_Color();
	
	if(Type_Inj == 'R')
	{	
		inj = 'M';
		return inj;
	}
	else if(Type_Inj == 'G')
	{
		inj = 'm';
		return inj;
	}
	else{
		inj = 'n';
		return inj;                 
	}
	
}

// Updates the injury struct for fetching requests later
void update_injury(char inj_t, unsigned char row, unsigned char col, unsigned char plot_no)
{
	injury_c[plot_no].inj_type = inj_t;
	injury_c[plot_no].row = row;
	injury_c[plot_no].col = col;
	injury_c[plot_no].plot_no = plot_no;
}


// A linked list (LL) node to store a queue entry
struct QNode {
    int key;
    struct QNode* next;
};

// The queue, front stores the front node of LL and rear stores the
// last node of LL
struct Queue {
    struct QNode *front, *rear;
};


// A utility function to create a new linked list node.
struct QNode* newNode(int k)
{
    struct QNode* temp = (struct QNode*)malloc(sizeof(struct QNode));
    temp->key = k;
    temp->next = NULL;
    return temp;
}

// A utility function to create an empty queue
struct Queue* createQueue()
{
    struct Queue* q = (struct Queue*)malloc(sizeof(struct Queue));
    q->front = q->rear = NULL;
    return q;
}



// The function to add a key k to q
void enQueue(struct Queue* q, int k)
{
    // Create a new LL node
    struct QNode* temp = newNode(k);
  
    // If queue is empty, then new node is front and rear both
    if (q->rear == NULL) {
        q->front = q->rear = temp;
        return;
    }
  
    // Add the new node at the end of queue and change rear
    q->rear->next = temp;
    q->rear = temp;
}


// Function to remove a key from given queue q
int deQueue(struct Queue* q)
{
    // If queue is empty, return NULL.
    if (q->front == NULL)
        return -1;
  
    // Store previous front and move front one node ahead
    struct QNode* temp = q->front;
  
    q->front = q->front->next;
  
    // If front becomes NULL, then change rear also as NULL
    if (q->front == NULL)
        q->rear = NULL;
    int v = temp->key;
    
    free(temp);
    return v;
}


bool check_connected(void){
	int next_r;
	int next_c;
	int present_r;
	int present_c;
	bool connected;

	for(int k = 0; k < 200; k++){
		if(reconstructed_path[k+1] == 0){
			connected = true;
			break;
		}
		next_r = node_c[reconstructed_path[k+1]-1].row;
		next_c = node_c[reconstructed_path[k+1]-1].col;
		present_r = node_c[reconstructed_path[k]-1].row;
		present_c = node_c[reconstructed_path[k]-1].col;
		if((next_r-present_r) > 1 || (next_r - present_r < -1) ||
				(next_c - present_c) > 1 || (next_c - present_c < -1)){
			connected = false;
			break;
		}
	}
	return connected;
}

void path_planning(int source_node[2], int end_node[2])
{
	struct Queue* r_queue = createQueue();
	struct Queue* c_queue = createQueue();
	int R = 9;
	int C = 9;
	unsigned char sr = source_node[0]; // row coordinate of start position
	unsigned char sc = source_node[1]; // column coordinate of start position
	unsigned char er = end_node[0]; // row coordinate of end position
	unsigned char ec = end_node[1]; // column coordinate of end position
	unsigned char rr = 0;   // For storing current row location under observation
	unsigned char cc = 0;   // For storing current column location under observation
	
	for(int i=0; i<R; i++){
		for(int j=0; j<C; j++){
			visited[i][j] = 0;
			prev[i][j] = 0;
		}
       }


	
//	lcd_numeric_value(1, 1, sr, 1);
//	lcd_numeric_value(1, 3, sc, 1);
//	lcd_numeric_value(2, 1, er, 1);
//	lcd_numeric_value(2, 3, ec, 1);


//Variables used to track number of steps taken
	unsigned char move_count = 0;
	unsigned char nodes_left_in_layer = 1; // 1 is for the start node
	unsigned char nodes_in_next_layer = 0;


// Variable used to track whether 'E' character ever gets reached during BFS
	bool reached_end;

// R*C matrix of false values which is used to track whether node at position
// (i,j) has been visited

	unsigned char r = 0;
	unsigned char c = 0;


        //mark the start location and end location in the grid map
	grid_map[sr][sc] = 's';
	grid_map[er][ec] = 'e';
        	 
	//enqueue the starting location coordinates
	enQueue(r_queue,sr);
	enQueue(c_queue,sc);	
		
	visited[sr][sc] = 1;
	while(r_queue->front != NULL){
		r = deQueue(r_queue);
		c = deQueue(c_queue);
		
		if(grid_map[r][c] == 'e')
		{
			reached_end = true;
			break;
		}
		for(int i = 0; i < 4; i++)
		{
			rr = r + dr[i];
			cc = c + dc[i];

			if((rr < 0) || (cc < 0)) continue;
			if((rr >= R) || (cc >= C)) continue;

			if(visited[rr][cc] == 1){
			 	continue;
			}
			if(grid_map[rr][cc] == '0') continue;
		
			enQueue(r_queue,rr);
			enQueue(c_queue,cc);
			visited[rr][cc] = 1;
			nodes_in_next_layer++;			
			prev[rr][cc] = node_numbering[r][c];
		}
		nodes_left_in_layer--;
		if(nodes_left_in_layer == 0){
			nodes_left_in_layer = nodes_in_next_layer;
			nodes_in_next_layer = 0;
			move_count++;
		}
	}
	
          
	//Clear the reconstructed_path array
	for(int i= 0; i < 200; i++)
	{
		reconstructed_path[i] = 0;
	}
	

//////////////////////////////////////////////////////////////
//BACKTRACKING SHORTEST PATH FOUND
	unsigned char k = 0; //variable to keep track of reconstructed path
	unsigned char i = er;
	unsigned char j = ec;
	unsigned char temp_i;
// Reconstruct the path back to start node from the end node
	while(grid_map[i][j] != 's')
	{
		reconstructed_path[k] = node_numbering[i][j];
		temp_i = node_c[prev[i][j]-1].row;
		j = node_c[prev[i][j]-1].col;
		k = k + 1;
		i = temp_i;
	}

	
/*
	for(int m = 0; m < k; m++)
	{
		printf("%d ",reconstructed_path[m]);
	}
*/
//////////////////////////////////////////////////////////////

// Reverse the reconstructed_path array to get the path to follow from
// start position
    unsigned char revIndex = 0;
    unsigned char arrIndex = k - 1;
    while(revIndex < arrIndex)
    {
        /* Copy value from original array to reverse array */
        unsigned char temp = reconstructed_path[revIndex];
        reconstructed_path[revIndex] = reconstructed_path[arrIndex];
        reconstructed_path[arrIndex] = temp;

        revIndex++;
        arrIndex--;
    }


    // Change the grid map values back to original
	grid_map[sr][sc] = '*';
	grid_map[er][ec] = '*';

	//if no path found return -1 in the first index
	if(!reached_end){
		reconstructed_path[0] = -1;
	}


    //Reinitialize visited array for next bfs search.
	for(int i = 0; i < R; i++){
		for(int j = 0; j < C; j++)
		{
			visited[i][j] = 0;
			prev[i][j] = 0;
		}
	}
	free(r_queue);
	free(c_queue);

}

int Get_Dist(void){
    	int dist = 0;
    	for(int i = 0; i < 200; i++){
    		if(reconstructed_path[i] == 0){
    			break;
    		}
    		dist = dist + 1;
    	}	
        return dist;
}



unsigned char Fetch_request(char inj){
	float dist[16] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
	unsigned char x = curr_loc.x;
	unsigned char y = curr_loc.y;
	unsigned char min_index = 0;
	for(int i = 0; i < 16; i++)
	{
		if(injury_c[i].inj_type == inj){
			dist[i] = sqrt((x - injury_c[i].row)*(x - injury_c[i].row) + (y - injury_c[i].col)*(y - injury_c[i].col));
			if(dist[i] < dist[min_index]){
				min_index = i;
			}
		}
	}
	
	if(dist[min_index] == 100){
		return -1;
	}
	
	return injury_c[min_index].plot_no;
}
// Function to entertain any request coming from server
bool Req_coming(void){
	int source_node[2] = {8,4};
	int goal_node[2] = {7,8};
	bool debris_detected;
	int next_node;
	int next_node_c[2];
	int row_diff;
	int col_diff;
	int flag;
	bool connected;
	char type_inj;
	unsigned char Req_plot_no = 0;
	float Vel_N_N = 0.27;
 
	
	Update_Command();
	if(Is_Command())
	{
		if(Is_Scan()){
			Req_plot_no = Scan_Plot_No();
			source_node[0] = curr_loc.x;
			source_node[1] = curr_loc.y;
			goal_node[0] = node_c[plot_m[Req_plot_no-1].midpoints[0]].row;
			goal_node[1] = node_c[plot_m[Req_plot_no-1].midpoints[1]].col;
			path_planning(source_node,goal_node);
			if( (int) (Get_Dist()/Vel_N_N) < Complete_In() ){
				Cmd_Accepted();
			}
			else{
				Cmd_Ignore();
				return true;
			}			
		}
		
		else if(Is_Major()){
		    type_inj = 'M';
			Req_plot_no = Fetch_request('M');
			if( Req_plot_no == -1)
			{
				Cmd_Ignore();
				return true;//false;
			}				
			else
			{	
				source_node[0] = curr_loc.x;
				source_node[1] = curr_loc.y;
				goal_node[0] = node_c[plot_m[Req_plot_no-1].midpoints[0]].row;
				goal_node[1] = node_c[plot_m[Req_plot_no-1].midpoints[1]].col;
				path_planning(source_node,goal_node);

				if( (int) (Get_Dist()/Vel_N_N) < Complete_In() )
				{ 
					Cmd_Accepted(); //Send ack that cmd is accepted
				}
				else{
					Cmd_Ignore();
					return true;
				}

			}
		}
		
		else if(Is_Minor()){
		    type_inj = 'm';
			Req_plot_no = Fetch_request('m');
			if(Req_plot_no == -1){
				Cmd_Ignore();
				return true;//false;
			}				
			else
			{
				source_node[0] = curr_loc.x;
				source_node[1] = curr_loc.y;
				goal_node[0] = node_c[plot_m[Req_plot_no-1].midpoints[0]].row;
				goal_node[1] = node_c[plot_m[Req_plot_no-1].midpoints[1]].col;
				path_planning(source_node,goal_node);			
				if( (int) (Get_Dist()/Vel_N_N) < Complete_In() )
				{
					Cmd_Accepted(); //Send ack that cmd is accepted
				}
				else{
					Cmd_Ignore();
					return true;
				}

			}
			
		}
		
		unsigned char scan_comp = 0;
		for(int i = 0; i < 4; i++){
			
			if(scan_comp == 1){
				break;
			}
			goal_node[0] = node_c[plot_m[Req_plot_no].midpoints[i]].row;
			goal_node[1] = node_c[plot_m[Req_plot_no].midpoints[i]].col;

		
		        //Move to the destination node and complete the request
			while(1){
				if(((curr_loc.x == goal_node[0]) && (curr_loc.y == goal_node[1]))){
					scan_comp = 1;
					break;
				}
				
				path_planning(source_node,goal_node);
				connected = check_connected();
				if(!connected){
					grid_map[goal_node[0]][goal_node[1]] = '0';
					break;
				}
				
				
				if(reconstructed_path[1] == 0){
					grid_map[goal_node[0]][goal_node[1]] = '0';
					break;
				}
				flag = 0;
				for(int k = 0; k < 200; k++){
					if(reconstructed_path[k] == 0 || flag == 1){
						if(flag == 0){
							source_node[0] = curr_loc.x;
							source_node[1] = curr_loc.y;
						}
						break;
					}
					next_node = reconstructed_path[k];
					source_node[0] = curr_loc.x;
					source_node[1] = curr_loc.y;
					next_node_c[0] = node_c[next_node-1].row;
					next_node_c[1] = node_c[next_node-1].col;
					row_diff = next_node_c[0] - source_node[0];
					col_diff = next_node_c[1] - source_node[1];
					if((row_diff > 0) && (col_diff == 0)){           //south direction
					switch(dir_flag){
					case 'n':		
						left_90();
						left_90();
						rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'a');
						dir_flag = 's';
						debris_detected = debris_detection();
						if(debris_detected){
							flag = 1;
							debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
							grid_map[curr_loc.x + 1][curr_loc.y] = '0';
							break;
						}
						forward_wls(1);
						forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x + 1][curr_loc.y]);
						curr_loc.x = curr_loc.x + 1;

						break;

					case 'e':
						right_90();
						rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'r');
						dir_flag = 's';
						debris_detected = debris_detection();
						if(debris_detected){
							flag = 1;
							debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
							grid_map[curr_loc.x + 1][curr_loc.y] = '0';
							break;
						}
						forward_wls(1);
						forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x + 1][curr_loc.y]);
						curr_loc.x = curr_loc.x + 1;

						break;

					case 'w':
						left_90();
						rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'l');
						dir_flag = 's';
						debris_detected = debris_detection();
						if(debris_detected){
							flag = 1;
							debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
							grid_map[curr_loc.x + 1][curr_loc.y] = '0';
							break;
						}
						
						forward_wls(1);
						forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x + 1][curr_loc.y]);
						curr_loc.x = curr_loc.x + 1;

						break;

					default:
						debris_detected = debris_detection();
						if(debris_detected){
							flag = 1;
							debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
							grid_map[curr_loc.x + 1][curr_loc.y] = '0';
							break;
						}

						forward_wls(1);
						forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x + 1][curr_loc.y]);
						curr_loc.x = curr_loc.x + 1;
						break;
					  }
					}
					if((row_diff < 0) && (col_diff == 0)){          //north direction
						switch(dir_flag){
						case 's':
							left_90();
							left_90();
							rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'a');
							dir_flag = 'n';
							debris_detected = debris_detection();
							if(debris_detected){
								flag = 1;
								debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
								grid_map[curr_loc.x - 1][curr_loc.y] = '0';
								break;
							}
							forward_wls(1);
							forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x - 1][curr_loc.y]);
							curr_loc.x = curr_loc.x - 1;
							break;
						case 'e':
							left_90();
							rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'l');
							dir_flag = 'n';
							debris_detected = debris_detection();
							if(debris_detected){
								flag = 1;
								debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
								grid_map[curr_loc.x - 1][curr_loc.y] = '0';
								break;
							}
					
							forward_wls(1);
							forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x - 1][curr_loc.y]);
							curr_loc.x = curr_loc.x - 1;
						    break;

						case 'w':
						
							right_90();
							rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'r');
							dir_flag = 'n';
				
							debris_detected = debris_detection();
							if(debris_detected){
								flag = 1;
								debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
								grid_map[curr_loc.x - 1][curr_loc.y] = '0';
								break;
							}

							forward_wls(1);
							forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x - 1][curr_loc.y]);
							curr_loc.x = curr_loc.x - 1;
							break;

						default:
						
							debris_detected = debris_detection();
							if(debris_detected){
								flag = 1;
								debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
								grid_map[curr_loc.x - 1][curr_loc.y] = '0';
								break;
							}
							forward_wls(1);
							forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x - 1][curr_loc.y]);
							curr_loc.x = curr_loc.x - 1;
							break;
						 }
				}
					if((row_diff == 0) && (col_diff > 0)){                 //East direction
						switch(dir_flag){
						case 'n':
							right_90();
							rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'r');
							dir_flag = 'e';

							debris_detected = debris_detection();
							if(debris_detected){
								flag = 1;
								debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
								grid_map[curr_loc.x][curr_loc.y + 1] = '0';
								break;
							}
							
							forward_wls(1);
							forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x][curr_loc.y + 1]);
							curr_loc.y = curr_loc.y + 1;
							break;
						case 's':
							left_90();
							rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'l');
							dir_flag = 'e';
							debris_detected = debris_detection();
							if(debris_detected){
								flag = 1;
								debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
								grid_map[curr_loc.x][curr_loc.y + 1] = '0';
								break;
							}

							forward_wls(1);
							forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x][curr_loc.y + 1]);
							curr_loc.y = curr_loc.y + 1;
						    break;
						case 'w':
							right_90();
							right_90();
							rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'a');
							dir_flag = 'e';
							debris_detected = debris_detection();
							if(debris_detected){
								flag = 1;
								debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
								grid_map[curr_loc.x][curr_loc.y + 1] = '0';
								break;
							}
							forward_wls(1);
							forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x][curr_loc.y + 1]);
							curr_loc.y = curr_loc.y + 1;
                            
							break;
						default:
							debris_detected = debris_detection();
							if(debris_detected){
								flag = 1;
								debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
								grid_map[curr_loc.x][curr_loc.y + 1] = '0';
								break;
							}
					
							forward_wls(1);
							forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x][curr_loc.y + 1]);
							curr_loc.y = curr_loc.y + 1;
					break;
						}
					}
					if((row_diff == 0) && (col_diff < 0)){                 //West direction
						switch(dir_flag){
						case 'n':
							left_90();
							rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'l');
							dir_flag = 'w';
							debris_detected = debris_detection();
							if(debris_detected){
								flag = 1;
								debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
								grid_map[curr_loc.x][curr_loc.y - 1] = '0';
								break;
							}
							forward_wls(1);
							forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x][curr_loc.y - 1]);
							curr_loc.y = curr_loc.y - 1;
							break;
						case 'e':
							left_90();
							left_90();
							rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'a');
							dir_flag = 'w';
							debris_detected = debris_detection();
							if(debris_detected){
								flag = 1;
								debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
								grid_map[curr_loc.x][curr_loc.y - 1] = '0';
								break;
							}
						
							forward_wls(1);
							forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x][curr_loc.y - 1]);
							curr_loc.y = curr_loc.y - 1;
						    break;
						case 's':
							right_90();
							rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'r');
							dir_flag = 'w';
							debris_detected = debris_detection();
							if(debris_detected){
								flag = 1;
								debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
								grid_map[curr_loc.x][curr_loc.y - 1] = '0';
								break;
							}
						
							forward_wls(1);
							forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x][curr_loc.y - 1]);
							curr_loc.y = curr_loc.y - 1;
							break;
						default:
							debris_detected = debris_detection();
							if(debris_detected){
								flag = 1;
								debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
								grid_map[curr_loc.x][curr_loc.y - 1] = '0';
								break;
							}

							forward_wls(1);
							forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x][curr_loc.y - 1]);
							curr_loc.y = curr_loc.y - 1;

							break;
					 }
				}
					

				}	

			
			}
		}
		return true;

	}
	return false;
}

/**
 * @brief      Executes the logic to achieve the aim of Lab 4
 */

void traverse_line_to_goal(void)
{
	int source_node[2] = {8,4};
	int goal_node[2] = {7,8};
	int R = 9;
	int C = 9;
	bool debris_detected;
	int plot_no;
	int plot_1;
	int plot_2;
	int next_node;
	int next_node_c[2];
	int row_diff;
	int col_diff;
	int flag;
	bool connected;
	char inj1;
	char inj2;

	//Serviced_plots
	bool serviced[16] = {false,false,false,false,
			             false,false,false,false,
			             false,false,false,false,
						 false,false,false,false};

	//Define coordinates of each node
	int nodes = 0;
	for(int i = 0; i < R; i++)
	{
		for(int j = 0; j < C; j++)
		{
			node_c[nodes].row = i;
			node_c[nodes].col = j;
			nodes = nodes + 1;
		}
	}
	
	//Define the injury characterisitics
	injury_c[0].inj_type = 'n'; injury_c[0].row = 0; injury_c[0].col = 0; injury_c[0].plot_no = 0;
	injury_c[1].inj_type = 'n'; injury_c[0].row = 0; injury_c[0].col = 0; injury_c[0].plot_no = 1;
	injury_c[2].inj_type = 'n'; injury_c[0].row = 0; injury_c[0].col = 0; injury_c[0].plot_no = 2;
	injury_c[3].inj_type = 'n'; injury_c[0].row = 0; injury_c[0].col = 0; injury_c[0].plot_no = 3;
	injury_c[4].inj_type = 'n'; injury_c[0].row = 0; injury_c[0].col = 0; injury_c[0].plot_no = 4;
	injury_c[5].inj_type = 'n'; injury_c[0].row = 0; injury_c[0].col = 0; injury_c[0].plot_no = 5;
	injury_c[6].inj_type = 'n'; injury_c[0].row = 0; injury_c[0].col = 0; injury_c[0].plot_no = 6;
	injury_c[7].inj_type = 'n'; injury_c[0].row = 0; injury_c[0].col = 0; injury_c[0].plot_no = 7;
	injury_c[8].inj_type = 'n'; injury_c[0].row = 0; injury_c[0].col = 0; injury_c[0].plot_no = 8;
	injury_c[9].inj_type = 'n'; injury_c[0].row = 0; injury_c[0].col = 0; injury_c[0].plot_no = 9;
	injury_c[10].inj_type = 'n'; injury_c[0].row = 0; injury_c[0].col = 0; injury_c[0].plot_no = 10;
	injury_c[11].inj_type = 'n'; injury_c[0].row = 0; injury_c[0].col = 0; injury_c[0].plot_no = 11;
	injury_c[12].inj_type = 'n'; injury_c[0].row = 0; injury_c[0].col = 0; injury_c[0].plot_no = 12;
	injury_c[13].inj_type = 'n'; injury_c[0].row = 0; injury_c[0].col = 0; injury_c[0].plot_no = 13;
	injury_c[14].inj_type = 'n'; injury_c[0].row = 0; injury_c[0].col = 0; injury_c[0].plot_no = 14;
	injury_c[15].inj_type = 'n'; injury_c[0].row = 0; injury_c[0].col = 0; injury_c[0].plot_no = 15;
	
	//Define the servicing points
	service_m[0].plots_covered[0] = 255; service_m[0].plots_covered[1] = 255;
	service_m[1].plots_covered[0] = 0;  service_m[1].plots_covered[1] = 255;
	service_m[2].plots_covered[0] = 255; service_m[2].plots_covered[1] = 255;
	service_m[3].plots_covered[0] = +1; service_m[3].plots_covered[1] = 255;
	service_m[4].plots_covered[0] = 255; service_m[4].plots_covered[1] = 255;
	service_m[5].plots_covered[0] = +2; service_m[5].plots_covered[1] = 255;
	service_m[6].plots_covered[0] = 255; service_m[6].plots_covered[1] = 255;
	service_m[7].plots_covered[0] = +3; service_m[7].plots_covered[1] = 255;
	service_m[8].plots_covered[0] = 255; service_m[8].plots_covered[1] = 255;
	service_m[9].plots_covered[0] = 0;  service_m[9].plots_covered[1] = 255;
	service_m[10].plots_covered[0] = 255; service_m[10].plots_covered[1] = 255;
	service_m[11].plots_covered[0] = 0; service_m[11].plots_covered[1] = +1;
	service_m[12].plots_covered[0] = 255; service_m[12].plots_covered[1] = 255;
	service_m[13].plots_covered[0] = +1; service_m[12].plots_covered[1] = 2;
	service_m[14].plots_covered[0] = 255; service_m[14].plots_covered[1] = 255;
	service_m[15].plots_covered[0] = +2; service_m[15].plots_covered[1] = +3;
	service_m[16].plots_covered[0] = 255; service_m[16].plots_covered[1] = 255;
	service_m[17].plots_covered[0] = +3; service_m[17].plots_covered[1] = 255;
	service_m[18].plots_covered[0] = 255; service_m[18].plots_covered[1] = 255;
	service_m[19].plots_covered[0] = 0; service_m[19].plots_covered[1] = +4;
	service_m[20].plots_covered[0] = 255; service_m[20].plots_covered[1] = 255;
	service_m[21].plots_covered[0] = +1; service_m[21].plots_covered[1] = +5;
	service_m[22].plots_covered[0] = 255; service_m[22].plots_covered[1] = 255;
	service_m[23].plots_covered[0] = +2; service_m[23].plots_covered[1] = +6;
	service_m[24].plots_covered[0] = 255; service_m[24].plots_covered[1] = 255;
	service_m[25].plots_covered[0] = +3; service_m[25].plots_covered[1] = +7;
	service_m[26].plots_covered[0] = 255; service_m[26].plots_covered[1] = 255;
	service_m[27].plots_covered[0] = +4; service_m[27].plots_covered[1] = 255;
	service_m[28].plots_covered[0] = 255; service_m[28].plots_covered[1] = 255;
	service_m[29].plots_covered[0] = +4; service_m[29].plots_covered[1] = +5;
	service_m[30].plots_covered[0] = 255; service_m[30].plots_covered[1] = 255;
	service_m[31].plots_covered[0] = +5; service_m[31].plots_covered[1] = +6;
	service_m[32].plots_covered[0] = 255; service_m[32].plots_covered[1] = 255;
	service_m[33].plots_covered[0] = +6; service_m[33].plots_covered[1] = +7;
	service_m[34].plots_covered[0] = 255; service_m[34].plots_covered[1] = 255;
	service_m[35].plots_covered[0] = +7; service_m[35].plots_covered[1] = 255;
	service_m[36].plots_covered[0] = 255; service_m[36].plots_covered[1] = 255;
	service_m[37].plots_covered[0] = +4; service_m[37].plots_covered[1] = +8;
	service_m[38].plots_covered[0] = 255; service_m[38].plots_covered[1] = 255;
	service_m[39].plots_covered[0] = +5; service_m[39].plots_covered[1] = +9;
	service_m[40].plots_covered[0] = 255; service_m[40].plots_covered[1] = 255;
	service_m[41].plots_covered[0] = +6; service_m[41].plots_covered[1] = +10;
	service_m[42].plots_covered[0] = 255; service_m[42].plots_covered[1] = 255;
	service_m[43].plots_covered[0] = +7; service_m[43].plots_covered[1] = +11;
	service_m[44].plots_covered[0] = 255; service_m[44].plots_covered[1] = 255;
	service_m[45].plots_covered[0] = +8; service_m[45].plots_covered[1] = 255;
	service_m[46].plots_covered[0] = 255; service_m[46].plots_covered[1] = 255;
	service_m[47].plots_covered[0] = +8; service_m[47].plots_covered[1] = +9;
	service_m[48].plots_covered[0] = 255; service_m[48].plots_covered[1] = 255;
	service_m[49].plots_covered[0] = +9; service_m[49].plots_covered[1] = +10;
	service_m[50].plots_covered[0] = 255; service_m[50].plots_covered[1] = 255;
	service_m[51].plots_covered[0] = +10; service_m[52].plots_covered[1] = +11;
	service_m[52].plots_covered[0] = 255; service_m[52].plots_covered[1] = 255;
	service_m[53].plots_covered[0] = +11; service_m[53].plots_covered[1] = 255;
	service_m[54].plots_covered[0] = 255; service_m[54].plots_covered[1] = 255;
	service_m[55].plots_covered[0] = +8; service_m[55].plots_covered[1] = +12;
	service_m[56].plots_covered[0] = 255; service_m[56].plots_covered[1] = 255;
	service_m[57].plots_covered[0] = +9; service_m[57].plots_covered[1] = +13;
	service_m[58].plots_covered[0] = 255; service_m[58].plots_covered[1] = 255;
	service_m[59].plots_covered[0] = +10; service_m[59].plots_covered[1] = +14;
	service_m[60].plots_covered[0] = 255; service_m[60].plots_covered[1] = 255;
	service_m[61].plots_covered[0] = +11; service_m[61].plots_covered[1] = +15;
	service_m[62].plots_covered[0] = 255; service_m[62].plots_covered[1] = 255;
	service_m[63].plots_covered[0] = +12; service_m[63].plots_covered[1] = 255;
	service_m[64].plots_covered[0] = 255; service_m[64].plots_covered[1] = 255;
	service_m[65].plots_covered[0] = +12; service_m[65].plots_covered[1] = +13;
	service_m[66].plots_covered[0] = 255; service_m[66].plots_covered[1] = 255;
	service_m[67].plots_covered[0] = +13; service_m[67].plots_covered[1] = +14;
	service_m[68].plots_covered[0] = 255; service_m[68].plots_covered[1] = 255;
	service_m[69].plots_covered[0] = +14; service_m[69].plots_covered[1] = +15;
	service_m[70].plots_covered[0] = 255; service_m[70].plots_covered[1] = 255;
	service_m[71].plots_covered[0] = +15; service_m[71].plots_covered[1] = 255;
	service_m[72].plots_covered[0] = 255; service_m[72].plots_covered[1] = 255;
	service_m[73].plots_covered[0] = +12; service_m[73].plots_covered[1] = 255;
	service_m[74].plots_covered[0] = 255; service_m[74].plots_covered[1] = 255;
	service_m[75].plots_covered[0] = +13; service_m[75].plots_covered[1] = 255;
	service_m[76].plots_covered[0] = 255; service_m[76].plots_covered[1] = 255;
	service_m[77].plots_covered[0] = +14; service_m[77].plots_covered[1] = 255;
	service_m[78].plots_covered[0] = 255; service_m[78].plots_covered[1] = 255;
	service_m[79].plots_covered[0] = +15; service_m[79].plots_covered[1] = 255;
	service_m[80].plots_covered[0] = 255; service_m[80].plots_covered[1] = 255;

	//define midpoints of each plot
	plot_m[0].midpoints[0] = 11;
	plot_m[0].midpoints[1] = 9;
	plot_m[0].midpoints[2]= 19;
	plot_m[0].midpoints[3] = 1;    //plot 0
	plot_m[1].midpoints[0] = 3;
	plot_m[1].midpoints[1] = 13;
	plot_m[1].midpoints[2] = 21;
	plot_m[1].midpoints[3] = 11;   //plot 1
	plot_m[2].midpoints[0] = 5;
	plot_m[2].midpoints[1] = 15;
	plot_m[2].midpoints[2] = 23;
	plot_m[2].midpoints[3] = 13;   //plot 2
	plot_m[3].midpoints[0] = 7;
	plot_m[3].midpoints[1] = 17;
	plot_m[3].midpoints[2] = 25;
	plot_m[3].midpoints[3] = 15;   //plot 3
	plot_m[4].midpoints[0] = 19;
	plot_m[4].midpoints[1] = 29;
	plot_m[4].midpoints[2] = 37;
	plot_m[4].midpoints[3] = 27;   //plot 4
	plot_m[5].midpoints[0] = 21;
	plot_m[5].midpoints[1] = 31;
	plot_m[5].midpoints[2] = 39;
	plot_m[5].midpoints[3] = 29;   //plot 5
	plot_m[6].midpoints[0] = 23;
	plot_m[6].midpoints[1] = 33;
	plot_m[6].midpoints[2] = 41;
	plot_m[6].midpoints[3] = 31;   //plot 6
	plot_m[7].midpoints[0] = 35;
	plot_m[7].midpoints[1] = 25;
	plot_m[7].midpoints[2] = 43;
	plot_m[7].midpoints[3] = 33;   //plot 7
	plot_m[8].midpoints[0] = 47;
	plot_m[8].midpoints[1] = 37;
	plot_m[8].midpoints[2] = 55;
	plot_m[8].midpoints[3] = 45;   //plot 8
	plot_m[9].midpoints[0] = 39;
	plot_m[9].midpoints[1] = 49;
	plot_m[9].midpoints[2] = 57;
	plot_m[9].midpoints[3] = 47;   //plot 9
	plot_m[10].midpoints[0] = 41;
	plot_m[10].midpoints[1] = 51;
	plot_m[10].midpoints[2] = 59;
	plot_m[10].midpoints[3] = 49;   //plot 10
	plot_m[11].midpoints[0] = 43;
	plot_m[11].midpoints[1] = 53;
	plot_m[11].midpoints[2] = 61;
	plot_m[11].midpoints[3] = 51;   //plot 11
	plot_m[12].midpoints[0] = 63;
	plot_m[12].midpoints[1] = 65;
	plot_m[12].midpoints[2] = 73;
	plot_m[12].midpoints[3] = 55;   //plot 12
	plot_m[13].midpoints[0] = 57;
	plot_m[13].midpoints[1] = 67;
	plot_m[13].midpoints[2] = 75;
	plot_m[13].midpoints[3] = 65;   //plot 13
	plot_m[14].midpoints[0] = 59;
	plot_m[14].midpoints[1] = 69;
	plot_m[14].midpoints[2] = 77;
	plot_m[14].midpoints[3] = 67;   //plot 14
	plot_m[15].midpoints[0] = 69;
	plot_m[15].midpoints[1] = 71;
	plot_m[15].midpoints[2] = 79;
	plot_m[15].midpoints[3] = 61;   //plot 15

	//move forward into the track at coordinate 8,4 from the start point.
    forward_wls(1); // curr_loc set to {8,4}


    // Keep searching until all plots are not covered
	for(int i = 0; i < 16; i++){
		for(int j = 0; j < 4; j++)
		{	
			
			goal_node[0] = node_c[plot_m[i].midpoints[j]].row;
			goal_node[1] = node_c[plot_m[i].midpoints[j]].col;
			if(serviced[i]){
				break;
			}
			while(1){
			if(((curr_loc.x == goal_node[0]) && (curr_loc.y == goal_node[1]))){
				char reached[] = "Reached";
				lcd_string_EE(reached);
				lcd_clear();
				break;
			}
			path_planning(source_node,goal_node);
			connected = check_connected();
			if(!connected){
				grid_map[goal_node[0]][goal_node[1]] = '0';
				break;
			}
			
			
			if(reconstructed_path[1] == 0){
				grid_map[goal_node[0]][goal_node[1]] = '0';
				break;
			}


			//Follow the reconstructed shortest path
			flag = 0;
			for(int k = 0; k < 200; k++){
			
				// Entertain any request from the server
			
				if(Req_coming())
				{
					source_node[0] = curr_loc.x;
					source_node[1] = curr_loc.y;		
					break;
				}
				
			
				if(reconstructed_path[k] == 0 || flag == 1){
					if(flag == 0){
						source_node[0] = curr_loc.x;
						source_node[1] = curr_loc.y;
					}
					break;
				}
				
            	

				next_node = reconstructed_path[k];
				source_node[0] = curr_loc.x;
				source_node[1] = curr_loc.y;
				next_node_c[0] = node_c[next_node-1].row;
				next_node_c[1] = node_c[next_node-1].col;
				row_diff = next_node_c[0] - source_node[0];
				col_diff = next_node_c[1] - source_node[1];
				if((row_diff > 0) && (col_diff == 0)){           //south direction
				switch(dir_flag){
				case 'n':
					left_90();
					left_90();
					rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'a');
					_delay_ms(80);
					dir_flag = 's';
					_delay_ms(420);
					debris_detected = debris_detection();
					if(debris_detected){
						flag = 1;
						debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
						grid_map[curr_loc.x + 1][curr_loc.y] = '0';
						break;
					}
					_delay_ms(50);
					forward_wls(1);
					forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x + 1][curr_loc.y]);
					curr_loc.x = curr_loc.x + 1;
					if(read_color[curr_loc.x][curr_loc.y] == 'y'){
						inj1 = print_color();
						left_turn_wls();
						inj2 = print_color();
						left_turn_wls();
						for(int i = 0; i < 2; i++){
							if(service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i] == 255){
								continue;
							}
							else{
								plot_no = service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i];
								if(i == 0){
									update_injury(inj1,curr_loc.x,curr_loc.y,plot_no);
									scanned_comm(plot_no + 1, inj1);   //Send type of injury to esp32 									
								}
								else{
									update_injury(inj2, curr_loc.x,curr_loc.y, plot_no);
									scanned_comm(plot_no + 1, inj2);
								}
								serviced[plot_no] = true;
								for(int j = 0; j < 4; j++){
									plot_1 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[0];
									plot_2 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[1];
									if(plot_1 == 255){
										if(serviced[plot_2]){
											read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
										}
									}
									if(plot_2 == 255){
										if(serviced[plot_1]){
											read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
										}
									}
									else{
										if(serviced[plot_1]&& serviced[plot_2]){
											read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
										}

									}


								}
							}
						}
					}
					break;

				case 'e':
					right_90();
					rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'r');
					dir_flag = 's';
					_delay_ms(500);

					debris_detected = debris_detection();
					if(debris_detected){
						flag = 1;
						debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
						grid_map[curr_loc.x + 1][curr_loc.y] = '0';
						break;
					}
					_delay_ms(50);
					forward_wls(1);
					forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x + 1][curr_loc.y]);
					curr_loc.x = curr_loc.x + 1;
					if(read_color[curr_loc.x][curr_loc.y] == 'y'){
						inj1 = print_color();
						left_turn_wls();
						inj2 = print_color();
						left_turn_wls();
						for(int i = 0; i < 2; i++){
							if(service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i] == 255){
								continue;
							}
							else{
								plot_no = service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i];
								if(i == 0){
									update_injury(inj1,curr_loc.x,curr_loc.y,plot_no);
									scanned_comm(plot_no + 1, inj1);   //Send type of injury to esp32
								}
								else{
									update_injury(inj2, curr_loc.x,curr_loc.y, plot_no);
									scanned_comm(plot_no + 1, inj2);   //Send type of injury to esp32
								}

								serviced[plot_no] = true;
								for(int j = 0; j < 4; j++){
									plot_1 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[0];
									plot_2 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[1];
									if(plot_1 == 255){
										if(serviced[plot_2]){
											read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
										}
									}
									if(plot_2 == 255){
										if(serviced[plot_1]){
											read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
										}
									}
									else{
										if(serviced[plot_1]&& serviced[plot_2]){
											read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
										}

									}



							}
						}
					}
				}

					break;

				case 'w':
					left_90();
					rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'l');
					dir_flag = 's';
					_delay_ms(500);

					debris_detected = debris_detection();
					if(debris_detected){
						flag = 1;
						debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
						grid_map[curr_loc.x + 1][curr_loc.y] = '0';
						break;
					}
					_delay_ms(50);
					forward_wls(1);
					forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x + 1][curr_loc.y]);
					curr_loc.x = curr_loc.x + 1;
					if(read_color[curr_loc.x][curr_loc.y] == 'y'){
						inj1 = print_color();
						left_turn_wls();
						inj2 = print_color();
						left_turn_wls();
						for(int i = 0; i < 2; i++){
							if(service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i] == 255){
								continue;
							}
							else{
								plot_no = service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i];
								if(i == 0){
									update_injury(inj1,curr_loc.x,curr_loc.y,plot_no);
									scanned_comm(plot_no + 1, inj1);   //Send type of injury to esp32
								}
								else{
									update_injury(inj2, curr_loc.x,curr_loc.y, plot_no);
									scanned_comm(plot_no + 1, inj2);   //Send type of injury to esp32
								}
								
								serviced[plot_no] = true;
								for(int j = 0; j < 4; j++){
									plot_1 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[0];
									plot_2 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[1];
									if(plot_1 == 255){
										if(serviced[plot_2]){
											read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
										}
									}
									if(plot_2 == 255){
										if(serviced[plot_1]){
											read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
										}
									}
									else{
										if(serviced[plot_1]&& serviced[plot_2]){
											read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
										}

									}


							}
						}
					}
				}

					break;

				default:
					_delay_ms(500);
					debris_detected = debris_detection();
					if(debris_detected){
						flag = 1;
						debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
						grid_map[curr_loc.x + 1][curr_loc.y] = '0';
						break;
					}
					_delay_ms(50);
					forward_wls(1);
					forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x + 1][curr_loc.y]);
					curr_loc.x = curr_loc.x + 1;
					if(read_color[curr_loc.x][curr_loc.y] == 'y'){
						inj1 = print_color();
						left_turn_wls();
						inj2 = print_color();
						left_turn_wls();
						for(int i = 0; i < 2; i++){
							if(service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i] == 255){
								continue;
							}
							else{
								plot_no = service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i];
								if(i == 0){
									update_injury(inj1,curr_loc.x,curr_loc.y,plot_no);
									scanned_comm(plot_no + 1, inj1);   //Send type of injury to esp32
								}
								else{
									update_injury(inj2, curr_loc.x,curr_loc.y, plot_no);
									scanned_comm(plot_no + 1, inj2);   //Send type of injury to esp32
								}
								
								serviced[plot_no] = true;
								for(int j = 0; j < 4; j++){
									plot_1 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[0];
									plot_2 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[1];
									if(plot_1 == 255){
										if(serviced[plot_2]){
											read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
										}
									}
									if(plot_2 == 255){
										if(serviced[plot_1]){
											read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
										}
									}
									else{
										if(serviced[plot_1]&& serviced[plot_2]){
											read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
										}

									}


							}
						}
					}
				}
					break;
				  }
				}
				if((row_diff < 0) && (col_diff == 0)){          //north direction
					switch(dir_flag){
					case 's':
						left_90();
						left_90();
						rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'a');
						dir_flag = 'n';
						_delay_ms(500);
						debris_detected = debris_detection();
						if(debris_detected){
							flag = 1;
							debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
							grid_map[curr_loc.x - 1][curr_loc.y] = '0';
							break;
						}
						_delay_ms(50);
						forward_wls(1);
						forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x - 1][curr_loc.y]);
						curr_loc.x = curr_loc.x - 1;
						if(read_color[curr_loc.x][curr_loc.y] == 'y'){
							inj1 = print_color();
							left_turn_wls();
							inj2 = print_color();
							left_turn_wls();
							for(int i = 0; i < 2; i++){
								if(service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i] == 255){
									continue;
								}
								else{
									plot_no = service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i];
								if(i == 0){
									update_injury(inj1,curr_loc.x,curr_loc.y,plot_no);
									scanned_comm(plot_no + 1, inj1);   //Send type of injury to esp32
								}
								else{
									update_injury(inj2, curr_loc.x,curr_loc.y, plot_no);
									scanned_comm(plot_no + 1, inj2);   //Send type of injury to esp32
								}
									
									serviced[plot_no] = true;
									for(int j = 0; j < 4; j++){
										plot_1 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[0];
										plot_2 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[1];
										if(plot_1 == 255){
											if(serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										if(plot_2 == 255){
											if(serviced[plot_1]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										else{
											if(serviced[plot_1]&& serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}

										}



								}
							}
						}
					}
						break;
					case 'e':
						left_90();
						rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'l');
						dir_flag = 'n';
						_delay_ms(500);
						debris_detected = debris_detection();
						if(debris_detected){
							debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
							flag = 1;
							grid_map[curr_loc.x - 1][curr_loc.y] = '0';
							break;
						}
						_delay_ms(50);
						forward_wls(1);
						forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x + 1][curr_loc.y]);
						curr_loc.x = curr_loc.x - 1;
						if(read_color[curr_loc.x][curr_loc.y] == 'y'){
							inj1 = print_color();
							left_turn_wls();
							inj2 = print_color();
							left_turn_wls();
							for(int i = 0; i < 2; i++){
								if(service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i] == 255){
									continue;
								}
								else{
									plot_no = service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i];
								if(i == 0){
									update_injury(inj1,curr_loc.x,curr_loc.y,plot_no);
									scanned_comm(plot_no + 1, inj1);   //Send type of injury to esp32
								}
								else{
									update_injury(inj2, curr_loc.x,curr_loc.y, plot_no);
									scanned_comm(plot_no + 1, inj2);   //Send type of injury to esp32
								}
									
									serviced[plot_no] = true;
									for(int j = 0; j < 4; j++){
										plot_1 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[0];
										plot_2 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[1];
										if(plot_1 == 255){
											if(serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										if(plot_2 == 255){
											if(serviced[plot_1]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										else{
											if(serviced[plot_1]&& serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}

										}



								}
							}
						}
					}
					    break;

					case 'w':
						right_90();
						rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'r');
						dir_flag = 'n';
						_delay_ms(500);
						debris_detected = debris_detection();
						if(debris_detected){
							debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
							flag = 1;
							grid_map[curr_loc.x - 1][curr_loc.y] = '0';
							break;
						}
						_delay_ms(50);
						forward_wls(1);
						forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x - 1][curr_loc.y]);
						curr_loc.x = curr_loc.x - 1;
						if(read_color[curr_loc.x][curr_loc.y] == 'y'){
							inj1 = print_color();
							left_turn_wls();
							inj2 = print_color();
							left_turn_wls();
							for(int i = 0; i < 2; i++){
								if(service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i] == 255){
									continue;
								}
								else{
									plot_no = service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i];
								if(i == 0){
									update_injury(inj1,curr_loc.x,curr_loc.y,plot_no);
									scanned_comm(plot_no + 1, inj1);   //Send type of injury to esp32
								}
								else{
									update_injury(inj2, curr_loc.x,curr_loc.y, plot_no);
									scanned_comm(plot_no + 1, inj2);   //Send type of injury to esp32
								}
									
									serviced[plot_no] = true;
									for(int j = 0; j < 4; j++){
										plot_1 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[0];
										plot_2 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[1];
										if(plot_1 == 255){
											if(serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										if(plot_2 == 255){
											if(serviced[plot_1]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										else{
											if(serviced[plot_1]&& serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}

										}



								}
							}
						}
					}
						break;

					default:
						_delay_ms(500);
						debris_detected = debris_detection();
						if(debris_detected){
							flag = 1;
							debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
							grid_map[curr_loc.x - 1][curr_loc.y] = '0';
							break;
						}
						_delay_ms(50);
						forward_wls(1);
						forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x - 1][curr_loc.y]);
						curr_loc.x = curr_loc.x - 1;
						if(read_color[curr_loc.x][curr_loc.y] == 'y'){
							inj1 = print_color();
							left_turn_wls();
							inj2 = print_color();
							left_turn_wls();
								for(int i = 0; i < 2; i++){
								if(service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i] == 255){
									continue;
								}
								else{
									plot_no = service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i];
								if(i == 0){
									update_injury(inj1,curr_loc.x,curr_loc.y,plot_no);
									scanned_comm(plot_no + 1, inj1);   //Send type of injury to esp32
								}
								else{
									update_injury(inj2, curr_loc.x,curr_loc.y, plot_no);
									scanned_comm(plot_no + 1, inj2);   //Send type of injury to esp32
								}
									
									serviced[plot_no] = true;
									for(int j = 0; j < 4; j++){
										plot_1 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[0];
										plot_2 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[1];
										if(plot_1 == 255){
											if(serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										if(plot_2 == 255){
											if(serviced[plot_1]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										else{
											if(serviced[plot_1]&& serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}

										}



								}
							}
						}
					}
						break;
					 }
			}
				if((row_diff == 0) && (col_diff > 0)){                 //East direction
					switch(dir_flag){
					case 'n':
						right_90();
						rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'r');
						dir_flag = 'e';
						_delay_ms(500);

						debris_detected = debris_detection();
						if(debris_detected){
							flag = 1;
							debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
							grid_map[curr_loc.x][curr_loc.y + 1] = '0';
							break;
						}
						_delay_ms(50);
						forward_wls(1);
						forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x][curr_loc.y + 1]);
						curr_loc.y = curr_loc.y + 1;
						if(read_color[curr_loc.x][curr_loc.y] == 'y'){
							inj1 = print_color();
							left_turn_wls();
							inj2 = print_color();
							left_turn_wls();

							for(int i = 0; i < 2; i++){
								if(service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i] == 255){
									continue;
								}
								else{
									plot_no = service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i];
								if(i == 0){
									update_injury(inj1,curr_loc.x,curr_loc.y,plot_no);
									scanned_comm(plot_no + 1, inj1);   //Send type of injury to esp32
								}
								else{
									update_injury(inj2, curr_loc.x,curr_loc.y, plot_no);
									scanned_comm(plot_no + 1, inj2);   //Send type of injury to esp32
								}
									
									serviced[plot_no] = true;
									for(int j = 0; j < 4; j++){
										plot_1 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[0];
										plot_2 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[1];
										if(plot_1 == 255){
											if(serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										if(plot_2 == 255){
											if(serviced[plot_1]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										else{
											if(serviced[plot_1]&& serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}

										}

								}
							}
						}
					}
						break;
					case 's':
						left_90();
						rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'l');
						dir_flag = 'e';
						_delay_ms(500);
						debris_detected = debris_detection();
						if(debris_detected){
							flag = 1;
							debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
							grid_map[curr_loc.x][curr_loc.y + 1] = '0';
							break;
						}
						_delay_ms(50);
						forward_wls(1);
						forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x][curr_loc.y + 1]);
						curr_loc.y = curr_loc.y + 1;
						if(read_color[curr_loc.x][curr_loc.y] == 'y'){
							inj1 = print_color();
							left_turn_wls();
							inj2 = print_color();
							left_turn_wls();
							for(int i = 0; i < 2; i++){
								if(service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i] == 255){
									continue;
								}
								else{
									plot_no = service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i];
								if(i == 0){
									update_injury(inj1,curr_loc.x,curr_loc.y,plot_no);
									scanned_comm(plot_no + 1, inj1);   //Send type of injury to esp32
								}
								else{
									update_injury(inj2, curr_loc.x,curr_loc.y, plot_no);
									scanned_comm(plot_no + 1, inj2);   //Send type of injury to esp32
								}
									
									serviced[plot_no] = true;
									for(int j = 0; j < 4; j++){
										plot_1 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[0];
										plot_2 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[1];
										if(plot_1 == 255){
											if(serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										if(plot_2 == 255){
											if(serviced[plot_1]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										else{
											if(serviced[plot_1]&& serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}

										}

								}
							}
						}
					}
					    break;
					case 'w':
						right_90();
						right_90();
						rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'a');
						dir_flag = 'e';
						_delay_ms(500);
						debris_detected = debris_detection();
						if(debris_detected){
							flag = 1;
							debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
							grid_map[curr_loc.x][curr_loc.y + 1] = '0';
							break;
						}
						curr_loc.y = curr_loc.y + 1;
						_delay_ms(50);
						forward_wls(1);
						forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x][curr_loc.y + 1]);
						if(read_color[curr_loc.x][curr_loc.y] == 'y'){
							inj1 = print_color();
							left_turn_wls();
							inj2 = print_color();
							left_turn_wls();
							for(int i = 0; i < 2; i++){
								if(service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i] == 255){
									continue;
								}
								else{
									plot_no = service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i];
								if(i == 0){
									update_injury(inj1,curr_loc.x,curr_loc.y,plot_no);
									scanned_comm(plot_no + 1, inj1);   //Send type of injury to esp32
								}
								else{
									update_injury(inj2, curr_loc.x,curr_loc.y, plot_no);
									scanned_comm(plot_no + 1, inj2);   //Send type of injury to esp32
								}
									
									serviced[plot_no] = true;
									for(int j = 0; j < 4; j++){
										plot_1 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[0];
										plot_2 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[1];
										if(plot_1 == 255){
											if(serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										if(plot_2 == 255){
											if(serviced[plot_1]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										else{
											if(serviced[plot_1]&& serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}

										}

								}
							}
						}
					}
						break;
					default:
						_delay_ms(500);
						debris_detected = debris_detection();
						if(debris_detected){
							flag = 1;
							debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
							grid_map[curr_loc.x][curr_loc.y + 1] = '0';
							break;
						}
						_delay_ms(50);
						forward_wls(1);
						forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x][curr_loc.y + 1]);
						curr_loc.y = curr_loc.y + 1;
						if(read_color[curr_loc.x][curr_loc.y] == 'y'){
							inj1 = print_color();
							left_turn_wls();
							inj2 = print_color();
							left_turn_wls();

							for(int i = 0; i < 2; i++){
								if(service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i] == 255){
									continue;
								}
								else{
									plot_no = service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i];
								if(i == 0){
									update_injury(inj1,curr_loc.x,curr_loc.y,plot_no);
									scanned_comm(plot_no + 1, inj1);   //Send type of injury to esp32
								}
								else{
									update_injury(inj2, curr_loc.x,curr_loc.y, plot_no);
									scanned_comm(plot_no + 1, inj2);   //Send type of injury to esp32
								}

									serviced[plot_no] = true;
									for(int j = 0; j < 4; j++){
										plot_1 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[0];
										plot_2 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[1];
										if(plot_1 == 255){
											if(serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										if(plot_2 == 255){
											if(serviced[plot_1]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										else{
											if(serviced[plot_1]&& serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}

										}



								}
							}
						}
					}
		                break;
					}
				}
				if((row_diff == 0) && (col_diff < 0)){                 //West direction
					switch(dir_flag){
					case 'n':
						left_90();
						rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'l');
						dir_flag = 'w';
						_delay_ms(500);

						debris_detected = debris_detection();
						if(debris_detected){
							flag = 1;
							debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
							grid_map[curr_loc.x][curr_loc.y - 1] = '0';
							break;
						}
						_delay_ms(50);
						forward_wls(1);
						forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x][curr_loc.y - 1]);
						curr_loc.y = curr_loc.y - 1;
						if(read_color[curr_loc.x][curr_loc.y] == 'y'){
							inj1 = print_color();
							left_turn_wls();
							inj2 = print_color();
							left_turn_wls();
							for(int i = 0; i < 2; i++){
								if(service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i] == 255){
									continue;
								}
								else{
									plot_no = service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i];
								if(i == 0){
									update_injury(inj1,curr_loc.x,curr_loc.y,plot_no);
									scanned_comm(plot_no + 1, inj1);   //Send type of injury to esp32
								}
								else{
									update_injury(inj2, curr_loc.x,curr_loc.y, plot_no);
									scanned_comm(plot_no + 1, inj2);   //Send type of injury to esp32
								}
									
									serviced[plot_no] = true;
									for(int j = 0; j < 4; j++){
										plot_1 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[0];
										plot_2 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[1];
										if(plot_1 == 255){
											if(serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										if(plot_2 == 255){
											if(serviced[plot_1]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										else{
											if(serviced[plot_1]&& serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}

										}


								}
							}
						}
					}
						break;
					case 'e':
						left_90();
						left_90();
						rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'a');
						dir_flag = 'w';
						_delay_ms(500);
						debris_detected = debris_detection();
						if(debris_detected){
							flag = 1;
							debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
							grid_map[curr_loc.x][curr_loc.y - 1] = '0';
							break;
						}
						_delay_ms(50);
						forward_wls(1);
						forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x][curr_loc.y - 1]);
						curr_loc.y = curr_loc.y - 1;
						if(read_color[curr_loc.x][curr_loc.y] == 'y'){
							inj1 = print_color();
							left_turn_wls();
							inj2 = print_color();
							left_turn_wls();
							for(int i = 0; i < 2; i++){
								if(service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i] == 255){
									continue;
								}
								else{
									plot_no = service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i];
									if(i == 0){
										update_injury(inj1,curr_loc.x,curr_loc.y,plot_no);
										scanned_comm(plot_no + 1, inj1);   //Send type of injury to esp32
									}
									else{
										update_injury(inj2, curr_loc.x,curr_loc.y, plot_no);
										scanned_comm(plot_no + 1, inj2);   //Send type of injury to esp32
									}
									
																	
									serviced[plot_no] = true;
									for(int j = 0; j < 4; j++){
										plot_1 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[0];
										plot_2 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[1];
										if(plot_1 == 255){
											if(serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										if(plot_2 == 255){
											if(serviced[plot_1]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										else{
											if(serviced[plot_1]&& serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}

										}


								}
							}
						}
					}
					    break;
					case 's':
						right_90();
						rotate_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag,'r');
						dir_flag = 'w';
						_delay_ms(500);


						debris_detected = debris_detection();
						if(debris_detected){
							flag = 1;
							debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
							grid_map[curr_loc.x][curr_loc.y - 1] = '0';
							break;
						}
						_delay_ms(50);
						forward_wls(1);
						forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x][curr_loc.y - 1]);
						curr_loc.y = curr_loc.y - 1;
						if(read_color[curr_loc.x][curr_loc.y] == 'y'){
							inj1 = print_color();
							left_turn_wls();
							inj2 = print_color();
							left_turn_wls();
							for(int i = 0; i < 2; i++){
								if(service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i] == 255){
									continue;
								}
								else{
									plot_no = service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i];
								if(i == 0){
									update_injury(inj1,curr_loc.x,curr_loc.y,plot_no);
									scanned_comm(plot_no + 1, inj1);   //Send type of injury to esp32
								}
								else{
									update_injury(inj2, curr_loc.x,curr_loc.y, plot_no);
									scanned_comm(plot_no + 1, inj2);   //Send type of injury to esp32
								}
									
									serviced[plot_no] = true;
									for(int j = 0; j < 4; j++){
										plot_1 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[0];
										plot_2 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[1];
										if(plot_1 == 255){
											if(serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										if(plot_2 == 255){
											if(serviced[plot_1]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										else{
											if(serviced[plot_1]&& serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}

										}


								}
							}
						}
					}
						break;
					default:
						_delay_ms(500);
						debris_detected = debris_detection();
						if(debris_detected){
							flag = 1;
							debris_comm(node_numbering[curr_loc.x][curr_loc.y],dir_flag);
							grid_map[curr_loc.x][curr_loc.y - 1] = '0';
							break;
						}
						_delay_ms(50);
						forward_wls(1);
						forward_comm(node_numbering[curr_loc.x][curr_loc.y], node_numbering[curr_loc.x][curr_loc.y - 1]);
						curr_loc.y = curr_loc.y - 1;
						if(read_color[curr_loc.x][curr_loc.y] == 'y'){
							inj1 = print_color();
							left_turn_wls();
							inj2 = print_color();
							left_turn_wls();
							for(int i = 0; i < 2; i++){
								if(service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i] == 255){
									continue;
								}
								else{
									plot_no = service_m[node_numbering[curr_loc.x][curr_loc.y]-1].plots_covered[i];
								if(i == 0){
									update_injury(inj1,curr_loc.x,curr_loc.y,plot_no);
									scanned_comm(plot_no + 1, inj1);   //Send type of injury to esp32
								}
								else{
									update_injury(inj2, curr_loc.x,curr_loc.y, plot_no);
									scanned_comm(plot_no + 1, inj2);   //Send type of injury to esp32
								}
									
									serviced[plot_no] = true;
									for(int j = 0; j < 4; j++){
										plot_1 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[0];
										plot_2 = service_m[node_numbering[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col]-1].plots_covered[1];
										if(plot_1 == 255){
											if(serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										if(plot_2 == 255){
											if(serviced[plot_1]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';
											}
										}
										else{
											if(serviced[plot_1]&& serviced[plot_2]){
												read_color[node_c[plot_m[plot_no].midpoints[j]].row][node_c[plot_m[plot_no].midpoints[j]].col] = 'n';

											}

										}
								}
							}
						}
					}

						break;
					 }
				}
			}
			}
	}

	}

}


