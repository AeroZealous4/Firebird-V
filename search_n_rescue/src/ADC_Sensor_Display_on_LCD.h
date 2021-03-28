/* 
 * lcd.h
 * Created: 10/07/2018 10:47:03
 * Author: Akshay Pampatwar
 */ 
#ifndef ADC_Sensor_Display_on_LCD_H_ 
#define ADC_Sensor_Display_on_LCD_H_ 

// //---------------------------------- GLOBAL VARIABLES -----------------------------------------------------
// unsigned char ADC_Value;
// unsigned char sharp, distance, adc_reading;
// unsigned int value;
// float BATT_Voltage, BATT_V;


void adc_port_config(void);
void adc_init(void);
unsigned char ADC_Conversion(unsigned char channel_num);
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading);
void print_sensor(char row, char coloumn,unsigned char channel);

#endif 