/*
 * eBot_Sandbox.h
 *
 *  Created on: 11-Jan-2021
 *      Author: TAs of CS 684 Spring 2020
 */

#ifndef EBOT_SANDBOX_H_
#define EBOT_SANDBOX_H_


//---------------------------------- INCLUDES ----------------------------------

#ifdef NON_MATLAB_PARSING
	#include "eBot_Sim_Predef.h"
#else
	// extern "C" {
	// 	#include "firebird_avr.h"//"eBot_MCU_Predef.h"
	// 	#include "ADC_Sensor_Display_on_LCD.h"
	// 	#include "lcd.h"
	// }
	#include "firebird_avr.h"//"eBot_MCU_Predef.h"
	#include "lcd.h"
	#include "ADC_Sensor_Display_on_LCD.h"
	#include "Color_Sensor.h"
	#include <util/delay.h>	
	#include "Position_Control_Interrupts.h"
	#include "atmega_to_esp32_uart.h"
#endif
// #include "firebird_avr.h"//"eBot_MCU_Predef.h"
// #include "ADC_Sensor_Display_on_LCD.h"
// #include "lcd.h"

//---------------------------------- FUNCTIONS ----------------------------------

// void send_sensor_data(void);
void init_all_peripherals(void);
void Controller(void);


#endif /* EBOT_SANDBOX_H_ */
