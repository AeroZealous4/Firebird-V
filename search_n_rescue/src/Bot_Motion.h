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

#define wl_sen_th_cal 28//25	//Line Exists, sen_read > wl_sen_th, For calibred value 0-100
#define wl_sen_th_l_cal 18	//Line Does not exists  sen_read < wl_sen_th_l, For calibred value 0-100
#define ir_sen_th 200

#define Cruise_Vel 200  //PID cruise vel
#define Cruise_Vel0 175 //Used while turning or moving fwd or back precisely

// char str[] = "Hello, I am a Firebird-------- V\n";

void Update_Read(void); //Updates reading of WL sensor
void calibrate(void);   //Caliberate WL of robot
void forward_wls1(unsigned char node); //Fn coded by suraj
void forward_wls(unsigned char node); //Fn coded by akshay Pampatwar
void left_turn_wls(void);   //Take a left turn till line is detected
void right_turn_wls(void);  //Take a right turn till line is detected
#endif /* BOT_MOTION_H_*/