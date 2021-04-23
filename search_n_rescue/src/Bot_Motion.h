/*
TITLE	: Bot_Motion
DATE	: 31st March 2021
AUTHOR	: Akshay Pampatwar

AIM: APIs for controlling motion of bot
*/

#ifndef BOT_MOTION_H_
#define BOT_MOTION_H_

#define turn_delay 2000
#define wl_sen_th 15	//Line Exists, sen_read > wl_sen_th
#define wl_sen_th_l 11	//Line Does not exists  sen_read < wl_sen_th_l

#define wl_sen_th_cal 28//25//Line Exists, sen_read > wl_sen_th, For calibred value 0-100
#define wl_sen_th_l_cal 28//22//18	//Line Does not exists  sen_read < wl_sen_th_l, For calibred value 0-100
#define ir_sen_th 200

#define Cruise_Vel 225  //PID cruise vel
#define Cruise_Vel0 225 //Used while turning or moving fwd or back precisely

// char str[] = "Hello, I am a Firebird-------- V\n";

void Update_Read(void); //Updates reading of WL sensor
void MinMax(void);
void calibrate(void);   //Caliberate WL of robot
void PID();
void forward_wls(unsigned char node); //Fn coded by akshay Pampatwar
void left_turn_wls(void);   //Take a left turn till line is detected
void right_turn_wls(void);  //Take a right turn till line is detected
bool debris_detection(void);
void right_90(void);
void left_90(void);
char print_color(void);

// Updates the injury struct for fetching requests later
void update_injury(char inj_t, unsigned char row, unsigned char col, unsigned char plot_no);
void enQueue(struct Queue* q, int k);
int deQueue(struct Queue* q);

bool check_connected(void);
void path_planning(unsigned char source_node[2], unsigned char end_node[2]);
int Get_Dist(void);
unsigned char Fetch_request(char inj);
bool Req_coming(void);
void traverse_line_to_goal(void);


#endif /* BOT_MOTION_H_*/
