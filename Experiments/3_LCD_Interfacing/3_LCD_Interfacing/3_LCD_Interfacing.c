/*! \mainpage 3_LCD_Interfacing
 *
 * @author     e-Yantra Team
 * @date       2020/01/31
 *
 * \subsection Aim
 * To display string on LCD.
 *
 * \subsection Connections
 * LCD Connections:								<br>
 *		 LCD Pins	  Micro-controller Pins     <br>
 *			  RS  --> PC0						<br>
 *			  RW  --> PC1						<br>
 *			  EN  --> PC2						<br>
 *			  DB7 --> PC7						<br>
 *			  DB6 --> PC6						<br>
 *			  DB5 --> PC5						<br>
 *			  DB4 --> PC4						<br>
 *
 */


//---------------------------------- HEADER FILES -----------------------------------------------------
#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>					// Standard AVR Delay Library
#include "lcd.h"						// LCD Header file included that contains function definitions essential to deal with LCD

//---------------------------------- MAIN ----------------------------------------------------------------
/**
 * @brief      Main Function
 *
 * @details    First Initializes LCD and displays message on LCD
 */
int main(void) {
	
	lcd_port_config();					// Initialize the LCD port
	lcd_init();							// Initialize the LCD
	
	while(1)
	{
		// Display text on LCD at specific location
		lcd_string(1, 5, "e-Yantra");
		lcd_string(2, 4, "IIT  BOMBAY");
	}
}
//---------------------------------- END ------------------------------------------------------------------