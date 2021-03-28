/*
 * eBot_MCU_Predef.cpp
 *
 *  Created on: 11-Jan-2021
 *      Author: TAs of CS 684 Spring 2020
 */


//---------------------------------- INCLUDES ----------------------------------

#include "eBot_MCU_Predef.h"


//---------------------------------- FUNCTIONS ----------------------------------

/**
 * @brief      Makes **ONLY** three white line sensor pins as input and dectivates pull-up for **ONLY** these sensor pins
 */
void wl_sensors_port_config(void)
{
	// << NOTE >> : Use Masking and Shift Operators here

	// Make **ONLY** three white line sensor pins as input
	wl_sensors_ddr_reg &= ~((1 << left_wl_sensor_pin) | (1 << right_wl_sensor_pin) | (1 << center_wl_sensor_pin));

	// Deactivate pull-up for **ONLY** for three white line sensor pins
	wl_sensors_port_reg	&= ~((1 << left_wl_sensor_pin) | (1 << right_wl_sensor_pin) | (1 << center_wl_sensor_pin)) ;
}


/**
 * @brief      Makes **ONLY** 5th IR proximity sensor pin as input and deactivates pull-up **ONLY** for that sensor pin
 */
void ir_prox_sensors_port_config(void)
{
	// << NOTE >> : Use Masking and Shift Operators here

	// Make **ONLY** 5th IR proximity sensor pin as input
	ir_prox_5_sensor_ddr_reg &= ~(1 << ir_prox_5_sensor_pin);

	// Deactivate pull-up for **ONLY** 5th IR proximity sensor pin
	ir_prox_5_sensor_port_reg &= ~(1 << ir_prox_5_sensor_pin);

}


/**
 * @brief      Initializes the Analog-to-Digital converter inside the micro-controller
 */
void adc_init(void)
{
	// << NOTE >> : Use Masking and Shift Operators here

	// In ADCSRA, enable ADC and pre-scalar = 64 (ADEN = 1, ADPS2 = 1, ADPS1 = 1, ADPS0 = 0)
	//				and clear ADC start conversion bit, auto trigger enable bit, interrupt flag bit and interrupt enable bit
	ADCSRA_reg	= (1 << ADEN_bit) | ~(1 << ADSC_bit) | ~(1 << ADATE_bit)|
                  ~(1 << ADIF_bit) | ~(1 << ADIE_bit) | (1 << ADPS2_bit) |
                   (1 << ADPS1_bit) | ~(1 << ADPS0_bit);//0x86;
    ADCSRA_reg	= (1 << ADEN_bit) | ~(1 << ADSC_bit) | ~(1 << ADATE_bit)|
                  ~(1 << ADIF_bit) | ~(1 << ADIE_bit) | (1 << ADPS2_bit) |
                   (1 << ADPS1_bit) | ~(1 << ADPS0_bit);//0x86;

	// In ADCSRB, disable Analog Comparator Multiplexer, MUX5 bit and ADC Auto Trigger Source bits
	ADCSRB_reg	= 0x40;

	// In ADMUX, set the Reference Selection bits to use the AVCC as reference, and disable the channel selection bits MUX[4:0]
	ADMUX_reg	= ~(1 << REFS1_bit) | (1 << REFS0_bit) | (1 << ADLAR_bit) | ir_prox_5_sensor_channel;
	//ADMUX_reg	= ;

	// In ADMUX, enable the ADLAR  bit for 8-bit ADC result
	//ADMUX_reg	= ;

	// In ACSR, disable the Analog Comparator by writing 1 to ACD_bit
	ACSR_reg = 0x80;
}


/*
 * Function Name: uart_init
 * Input: ubbr_value (calculate UBRR value according to the BAUD_RATE and F_CPU)
 * Output: None
 * Logic: This function initializes the UART
 * Example Call: uart_init();
 */
void uart_init (unsigned int ubbr_value)
{
	// In UCSRB_reg, disable all bits setting the Baud Rate
	UCSRB_reg	= 0x00;

	// In UBRRH_reg and UBRRL_reg, set the baud rate
	UBRRH_reg	= (unsigned char) ( ubbr_value >> 8 );
	UBRRL_reg	= (unsigned char) ( ubbr_value );

	// In UCSRC_reg, select the Asynchronous USART mode and the character size to 8 bits
	UCSRC_reg	&= ~( ( 1 << UMSEL1_bit ) | ( 1 << UMSEL0_bit ) );
	UCSRC_reg	|= ( ( 1 << UCSZ1_bit ) | ( 1 << UCSZ0_bit ) );

	// In UCSRB_reg, enable the receiver and transmitter
	UCSRB_reg	|= ( ( 1 << TXEN_bit ) | ( 1 << RXEN_bit ) );
	UCSRB_reg	&= ~( 1 << UCSZ2_bit );
}


/**
 * @brief      Initializes the setup by configuring all the required devices
 */

void adc_init2(void){
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
int init_setup(void)
{
	// Initialize the three white line sensors
	wl_sensors_port_config();

	// Initialize the 5th IR proximity sensor
	ir_prox_sensors_port_config();

	// Initialize the ADC
	adc_init2();

	// Initialize the UART
	uart_init(UBRR_VALUE);

	return 1;
}


/**
 * @brief      Sets the MUX[5:0] bits according to the sensor's channel number as input
 *
 * @param[in]  channel_num   ADC channel number of sensor
 */
void select_adc_channel( unsigned char channel_num )
{
	// << NOTE >> : Use Masking and Shift Operators here

	// set the MUX[5:0] bits to select the ADC channel number
	if (channel_num > 7 )
	{
		ADCSRB_reg |= (1<<MUX5_bit);					// set the MUX5 bit for selecting channel if its greater than 7
	}else{
        ADCSRB_reg &= ~(1<<MUX5_bit);					// set the MUX5 bit for selecting channel if its greater than 7
    }


	channel_num	&= 0x07;						// retain the last 3 bits from the variable for MUX[2:0] bits

	ADMUX_reg	= (ADMUX_reg | channel_num );
}


/**
 * @brief      Starts the ADC by setting the ADSC bit in ADCSRA register
 */
void start_adc(void)
{
	// << NOTE >> : Use Masking and Shift Operators here

	// set the ADSC bit in ADCSRA register
	ADCSRA_reg	|= (1 << ADSC_bit);
}


/**
 * @brief      Checks if the ADC conversion for the selected channel is complete or not
 *
 * @return     boolean true if the ADC has completed its conversion for the selected channel, else false.
 */
bool check_adc_conversion_complete(void)
{
	// << NOTE >> : Use Masking and Shift Operators here

	/*
	<< TODO >> :
		1. Write an if-else statement with a condition which checks whether the ADC conversion for the selected channel is complete or not.
		2. If the ADC has completed its conversion for the selected channel return true, else return false
	*/

    if(ADCSRA_reg & (1 << ADIF_bit)) return true;
    else return false;
}


/**
 * @brief      Get the ADC converted data from ADC data registers
 *
 * @return     adc_8bit_data	ADC converted data of the sensor by reading ADC data registers
 */
unsigned char read_adc_converted_data(void)
{
	// << NOTE >> : Use Masking and Shift Operators here

	unsigned char adc_8bit_data;

	// read the appropriate ADC data register/s

	unsigned char adc_data_high_byte;

	adc_data_high_byte	= ADCH_reg;

	adc_8bit_data		= ADCH_reg; //ADLAR is set
	return adc_8bit_data;
}


/**
 * @brief      Reset ADC config registers, ADCSRA, ADCSRB and ADMUX
 */
void reset_adc_config_registers(void)
{
	// << NOTE >> : Use Masking and Shift Operators here

    ADCSRA_reg		|= ( 1 << ADIF_bit );
    ADCSRB_reg		&= ~( 1 << MUX5_bit );
    // clear the MUX[4:0] bits
    ADMUX_reg		&= ~( (1 << MUX4_bit) | (1 << MUX3_bit) | (1 << MUX2_bit) | (1 << MUX1_bit) | (1 << MUX0_bit) );

}


/**
 * @brief      Convert the analog readings to 8-bit digital format from the sensor's ADC channel number as input
 *
 * @param[in]  sensor_channel_number   ADC channel number of sensor
 *
 * @return     8-bit digital data from the input sensor ADC channel
 */
unsigned char convert_analog_channel_data( unsigned char sensor_channel_number )
{

	// << NOTE >> : You are not allowed to modify or change anything inside this function

	unsigned char adc_8bit_data;

	select_adc_channel( sensor_channel_number );

	start_adc();

	while( !( check_adc_conversion_complete() ) );

	adc_8bit_data = read_adc_converted_data();

	reset_adc_config_registers();

	return adc_8bit_data;
}


/*
 * Function Name: uart_tx
 * Input: data (a character to send)
 * Output: None
 * Logic: This function sends a character over the UART
 * Example Call: uart_tx();
 */
void uart_tx(char data)
{
//	while( ( UCSRA_reg & ( 1 << UDRE_bit ) ) == 0x00 );				// waiting to transmit
	UDR_reg	= data;
}


/*
 * Function Name: uart_tx_string
 * Input: *data (address of the string)
 * Output: None
 * Logic: This function sends a string over the UART
 * Example Call: uart_tx_string();
 */
void uart_tx_string(char *data)
{
	while(*data != '\0')
	{
		uart_tx(*data);
		data++;
	}
}


/**
 * @brief      Function to print the sensor data on UART Serial Terminal
 *
 * @param[in]  ir_prox_5_sensor_data   ADC result of 5th IR Proximity sensor
 *
 * @return     1, constant value to print the data for infinite times
 */
int print_ir_prox_5_data(unsigned char ir_prox_5_sensor_data)
{
	// To create string for transmitting 5th IR Proximity sensor data over the UART
	char tx_buffer[50];

	sprintf(tx_buffer, "5th IR Prox. data: %03d\n", ir_prox_5_sensor_data);
	uart_tx_string(tx_buffer);

	return 1;
}
