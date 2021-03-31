/*! \mainpage Experiment: 9_ADC_Sensor_Display_on_LCD
 *
 * @author     e-Yantra Team
 * @date       2020/01/31
 *
 * \subsection Aim
 * To get the 8-bit ADC result from the sensors in Single Conversion Mode and display the ADC converted digital values on LCD.
 *
 * \subsection Connections:
 *			ACD CH.		PORT	Sensor
 *			0			PF0		Battery Voltage								<br>
 *			1			PF1		White line sensor 3							<br>
 *			2			PF2		White line sensor 2							<br>
 *			3			PF3		White line sensor 1							<br>
 *			4			PF4		IR Proximity analog sensor 1*****			<br>
 *			5			PF5		IR Proximity analog sensor 2*****			<br>
 *			6			PF6		IR Proximity analog sensor 3*****			<br>
 *			7			PF7		IR Proximity analog sensor 4*****			<br>
 *			8			PK0		IR Proximity analog sensor 5				<br>
 *			9			PK1		Sharp IR range sensor 1						<br>
 *			10			PK2		Sharp IR range sensor 2						<br>
 *			11			PK3		Sharp IR range sensor 3						<br>
 *			12			PK4		Sharp IR range sensor 4						<br>
 *			13			PK5		Sharp IR range sensor 5						<br>
 *			14			PK6		Servo Pod 1									<br>
 *			15			PK7		Servo Pod 2									<br>
 *
 *
 * ***** For using Analog IR proximity (1, 2, 3 and 4) sensors short the jumper J2.			<br>
 * To use JTAG via expansion slot of the micro controller socket remove these jumpers.		<br>
 *
 *		LCD Display interpretation:															<br>
 *
 *		***************************************************************************			<br>
 *		BATTERY VOLTAGE	IR PROX.SENSOR 2	IR PROX.SENSOR 3	IR.PROX.SENSOR 4			<br>
 *		LEFT WL SENSOR	CENTER WL SENSOR	RIGHT WL SENSOR		FRONT SHARP DIS				<br>
 *		***************************************************************************			<br>
 *
 */


//---------------------------------- HEADER FILES -----------------------------------------------------
#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>					// Standard AVR Delay Library
#include <stdio.h>						// Standard C Library for standard input output
#include "lcd.h"						// LCD Header file included that contains function definitions essential to deal with LCD

//---------------------------------- GLOBAL VARIABLES -----------------------------------------------------
unsigned char ADC_Value;
unsigned char sharp, distance, adc_reading;
unsigned int value;
float BATT_Voltage, BATT_V;

//---------------------------------- FUNCTIONS -----------------------------------------------------
/**
 * @brief      Function to make all ADC sensor pins as input and deactivate pull up for all pins
 */
void adc_port_config (void)
{
	adc_sensor_low_ddr_reg		= 0x00;				// set PORTF direction as input
	adc_sensor_low_port_reg		= 0x00;				// set PORTF pins floating
	adc_sensor_high_ddr_reg		= 0x00;				// set PORTK direction as input
	adc_sensor_high_port_reg	= 0x00;				// set PORTK pins floating
}

/**
 * @brief      Initializes the Analog-to-Digital converter inside the micro-controller
 */
void adc_init(){
	
	// enable ADC and pre-scalar = 64 (ADEN = 1, ADPS2 = 1, ADPS1 = 1, ADPS0 = 0)
	// and clear ADC start conversion bit, auto trigger enable bit, interrupt flag bit and interrupt enable bit
	ADCSRA_reg	|= ( (1 << ADEN_bit) | (1 << ADPS2_bit) | (1 << ADPS1_bit) );
	ADCSRA_reg	&= ~( (1 << ADSC_bit) | (1 << ADATE_bit) | (1 << ADIF_bit) | (1 << ADIE_bit) | (1 << ADPS0_bit) );
	
	// In ADCSRB, disable Analog Comparator Multiplexer, MUX5 bit and ADC Auto Trigger Source bits
	ADCSRB_reg	&= ~( (1 << ACME_bit) | (1 << MUX5_bit) | (1 << ADTS2_bit) | (1 << ADTS1_bit) | (1 << ADTS0_bit) );
	
	// In ADMUX, set the Reference Selection bits to use the AVCC as reference, and disable the channel selection bits MUX[4:0]
	ADMUX_reg	&= ~( (1 << REFS1_bit) | (1 << MUX4_bit) | (1 << MUX3_bit) | (1 << MUX2_bit) | (1 << MUX1_bit) | (1 << MUX0_bit) );
	ADMUX_reg	|= (1 << REFS0_bit);
	
	// In ADMUX, enable the ADLAR bit for 8-bit ADC result
	ADMUX_reg	|= (1 << ADLAR_bit);
	
	// In ACSR, disable the Analog Comparator by writing 1 to ACD_bit
	ACSR_reg	|= ( 1 << ACD_bit );
}

/**
 * @brief      Convert the analog readings to 8-bit digital format from the sensor's ADC channel number as input
 *
 * @param[in]  sensor_channel_number   ADC channel number of sensor
 *
 * @return     8-bit digital data from the input sensor ADC channel
 */
unsigned char ADC_Conversion(unsigned char channel_num)
{
	unsigned char adc_8bit_data;
	
	// MUX[5:0] bits to select the ADC channel number
	if ( channel_num > 7 )
	{
		ADCSRB_reg |= ( 1 << MUX5_bit );					// set the MUX5 bit for selecting channel if its greater than 7
	}
	channel_num	= channel_num & 0x07;						// retain the last 3 bits from the variable for MUX[2:0] bits
	ADMUX_reg	= ( ADMUX_reg | channel_num );
	
	// set the ADSC bit in ADCSRA register
	ADCSRA_reg		|= ( 1 << ADSC_bit );
	
	//Wait for ADC conversion to complete
	while( ( ADCSRA_reg & ( 1 << ADIF_bit ) ) == 0x00 );
	
	adc_8bit_data = ADCH_reg;
	
	// clear ADIF bit by writing 1 to it
	ADCSRA_reg		|= ( 1 << ADIF_bit );
	
	// clear the MUX5 bit
	ADCSRB_reg		&= ~( 1 << MUX5_bit );
	
	// clear the MUX[4:0] bits
	ADMUX_reg		&= ~( (1 << MUX4_bit) | (1 << MUX3_bit) | (1 << MUX2_bit) | (1 << MUX1_bit) | (1 << MUX0_bit) );
	
	return adc_8bit_data;
}

/**
 * @brief      Converts 8-bit digital data of sharp sensor to distance in mm
 *
 * @param[in]  adc_reading 8 bit ADC reading
 *
 * @return     distance in mm range - 0 to 800
 */
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

/**
 * @brief      Displays Converted 8-bit digital data from the sensor's ADC channel on LCD
 *
 * @param[in]  row   LCD row 1 or 2
 * @param[in]  column LCD column 1 to 16
 * @param[in]  channel ADC channel number of sensor 0 to 15
 *
 */
void print_sensor(char row, char coloumn,unsigned char channel)
{
	ADC_Value = ADC_Conversion(channel);
	lcd_numeric_value(row, coloumn, ADC_Value, 3);
}

//---------------------------------- MAIN ----------------------------------------------------------------
/**
 * @brief      Main Function
 *
 * @details    Initializes ADC, LCD pins and displays the ADC converted data of all the sensors on LCD
 */
int main(void) {
	
	unsigned int value;
	adc_port_config();					// Initialize the ADC port
	adc_init();							// Initialize the ADC
	
	lcd_port_config();					// Initialize the LCD port
	lcd_init();							// Initialize the LCD
	
	while(1)
	{
		// get the ADC converted data of the sensors from the appropriate channel number
		
		BATT_V = ADC_Conversion(batt_sensor_channel);
		BATT_Voltage = ( ( BATT_V * 100 ) * 0.07902 ) + 0.7;
		lcd_numeric_value(1, 1, BATT_Voltage, 4);					// Prints Battery Voltage Status

		print_sensor(1, 6, ir_prox_2_sensor_channel);				// Prints value of Analog IR Proximity Sensor 2
		print_sensor(1, 10, ir_prox_3_sensor_channel);				// Prints value of Analog IR Proximity Sensor 3
		print_sensor(1, 14, ir_prox_4_sensor_channel);				// Prints value of Analog IR Proximity Sensor 4
		print_sensor(2, 2, left_wl_sensor_channel);					// Prints value of Left White Line Sensor
		print_sensor(2, 6, center_wl_sensor_channel);				// Prints Value of Center White Line Sensor
		print_sensor(2, 10, right_wl_sensor_channel);				// Prints Value of Right White Line Sensor

		sharp = ADC_Conversion(sharp_sensor_channel);				// Stores the Analog value of front Sharp connected to ADC channel 11 into variable "sharp"
		value = Sharp_GP2D12_estimation(sharp);						// Stores Distance calculated in a variable "value"
		lcd_numeric_value(2,14,value,3); 							// Prints Value Of Distance in MM measured by Sharp Sensor

	}
}
//---------------------------------- END ------------------------------------------------------------------