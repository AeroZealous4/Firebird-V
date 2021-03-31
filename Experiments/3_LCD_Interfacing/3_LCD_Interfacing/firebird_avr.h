/*
 * firebird_avr.h
 *
 * Created: 2/15/2020 17:14:54
 * Author: e-Yantra Team
 */ 


#ifndef FIREBIRD_AVR_H_
#define FIREBIRD_AVR_H_


#include <avr/io.h>											// Standard AVR IO Library
#include <avr/interrupt.h>									// Standard AVR Interrupt Library


// Definitions for ATmega2560 micro-controller on Firebird V robot
#if defined(__AVR_ATmega2560__)

	#define		F_CPU							14745600
	
	//---------------------------------- INPUT / OUTPUT PERIPHERALS -----------------------------------------------------

	// Buzzer definitions
	#define		buzzer_ddr_reg					DDRC
	#define		buzzer_port_reg					PORTC
	#define		buzzer_pin						PC3			// 3
	
	// Interrupt Switch definitions
	#define		interrupt_sw_ddr_reg			DDRE
	#define		interrupt_sw_port_reg			PORTE
	#define		interrupt_sw_pin_reg			PINE
	#define		interrupt_sw_pin				PE7			// 7
	
	// Bar-graph LED definitions	
	#define		bar_graph_led_ddr_reg			DDRJ
	#define		bar_graph_led_port_reg			PORTJ
	#define		bar_graph_led_8_pin				PJ7			// 7
	#define		bar_graph_led_7_pin				PJ6			// 6
	#define		bar_graph_led_6_pin				PJ5			// 5
	#define		bar_graph_led_5_pin				PJ4			// 4
	#define		bar_graph_led_4_pin				PJ3			// 3
	#define		bar_graph_led_3_pin				PJ2			// 2
	#define		bar_graph_led_2_pin				PJ1			// 1
	#define		bar_graph_led_1_pin				PJ0			// 0
	
	// LCD definitions
	#define		lcd_data_ddr_reg				DDRC
	#define		lcd_control_ddr_reg				DDRC

	#define		lcd_data_port_reg				PORTC
	#define		lcd_control_port_reg			PORTC

	#define		RS_pin							PC0			// 0
	#define		RW_pin							PC1			// 1
	#define		EN_pin							PC2			// 2

	#define		DB7_pin							PC7			// 7
	#define		DB6_pin							PC6			// 6
	#define		DB5_pin							PC5			// 5
	#define		DB4_pin							PC4			// 4

	// Motor direction registers and pins
	#define 	motors_dir_ddr_reg				DDRA
	#define 	motors_dir_port_reg				PORTA
	#define 	motors_RB_pin					PA3			// 3
	#define 	motors_RF_pin					PA2			// 2
	#define 	motors_LF_pin					PA1			// 1
	#define 	motors_LB_pin					PA0			// 0

	// Motor enable registers and pins
	#define 	motors_pwm_ddr_reg				DDRL
	#define 	motors_pwm_port_reg				PORTL
	#define 	motors_pwm_R_pin				PL4			// 4
	#define 	motors_pwm_L_pin				PL3			// 3

	// Position encoder registers and pins
	#define 	position_encoder_ddr_reg		DDRE
	#define 	position_encoder_port_reg		PORTE
	#define 	left_encoder_pin				PE4			// 4
	#define 	right_encoder_pin				PE5			// 5

	// Servo motor registers
	#define 	servo_ddr_reg					DDRB
	#define 	servo_port_reg					PORTB
	#define 	servo_1_pin						PB5			// 5
	#define 	servo_2_pin						PB6			// 6
	#define 	servo_3_pin						PB7			// 7

	// ADC sensors registers
	#define		adc_sensor_low_ddr_reg			DDRF
	#define		adc_sensor_low_port_reg			PORTF
	#define		adc_sensor_high_ddr_reg			DDRK
	#define		adc_sensor_high_port_reg		PORTK

	// Battery monitoring sensor registers and pins
	#define 	batt_sensor_ddr_reg				DDRF
	#define 	batt_sensor_port_reg			PORTF
	#define 	batt_sensor_pin					PF0			// 0
	#define 	batt_sensor_channel				0			// ADC0 - ADC Channel 0

	// 3 White-Line sensors definitions
	#define		wl_sensors_ddr_reg				DDRF
	#define		wl_sensors_port_reg				PORTF
	#define		left_wl_sensor_pin				PF3			// 3
	#define		left_wl_sensor_channel			3			// ADC3 - ADC Channel 3
	#define		center_wl_sensor_pin			PF2			// 2
	#define		center_wl_sensor_channel		2			// ADC2 - ADC Channel 2
	#define		right_wl_sensor_pin				PF1			// 1
	#define		right_wl_sensor_channel			1			// ADC1 - ADC Channel 1

	// 1 to 4 IR proximity sensors definitions
	#define		ir_prox_1_to_4_sensors_ddr_reg	DDRF
	#define		ir_prox_1_to_4_sensors_port_reg	PORTF
	#define		ir_prox_1_sensor_pin			PF4			// 4
	#define		ir_prox_1_sensor_channel		4			// ADC4 - ADC Channel 4
	#define		ir_prox_2_sensor_pin			PF5			// 5
	#define		ir_prox_2_sensor_channel		5			// ADC5 - ADC Channel 5
	#define		ir_prox_3_sensor_pin			PF6			// 6
	#define		ir_prox_3_sensor_channel		6			// ADC6 - ADC Channel 6
	#define		ir_prox_4_sensor_pin			PF7			// 7
	#define		ir_prox_4_sensor_channel		7			// ADC7 - ADC Channel 7

	// 5th IR proximity sensor definitions
	#define		ir_prox_5_sensor_ddr_reg		DDRK
	#define		ir_prox_5_sensor_port_reg		PORTK
	#define		ir_prox_5_sensor_pin			PK0			// 0
	#define		ir_prox_5_sensor_channel		8			// ADC8 - ADC Channel 8
	
	// Sharp sensors definitions
	#define		sharp_sensors_ddr_reg			DDRK
	#define		sharp_sensors_port_reg			PORTK
	#define		sharp_1_sensor_pin				PK1			// 1
	#define		sharp_1_sensor_channel			9			// ADC9 - ADC Channel 9
	#define		sharp_2_sensor_pin				PK2			// 2
	#define		sharp_2_sensor_channel			10			// ADC10 - ADC Channel 10
	#define		sharp_3_sensor_pin				PK3			// 3
	#define		sharp_3_sensor_channel			11			// ADC11 - ADC Channel 11
	#define		sharp_4_sensor_pin				PK4			// 4
	#define		sharp_4_sensor_channel			12			// ADC12 - ADC Channel 12
	#define		sharp_5_sensor_pin				PK5			// 5
	#define		sharp_5_sensor_channel			13			// ADC13 - ADC Channel 13

	// Sensor switching definitions
	#define		wl_pwr_ctrl_ddr_reg				DDRG
	#define		wl_pwr_ctrl_port_reg			PORTG
	#define		wl_pwr_ctrl_pin					PG2			// 2
	#define		ir_prox_pwr_ctrl_ddr_reg		DDRH
	#define		ir_prox_pwr_ctrl_port_reg		PORTH
	#define		ir_prox_pwr_ctrl_pin			PH2			// 2
	#define		sharp_pwr_ctrl_ddr_reg			DDRH
	#define		sharp_pwr_ctrl_port_reg			PORTH
	#define		sharp_pwr_ctrl_pin				PH3			// 3
	
	// For firebird_simulation.h compatibility (Optional)
	#define		ir_prox_3_4_sensors_ddr_reg		DDRF
	#define		ir_prox_3_4_sensors_port_reg	PORTF

	#define		sharp_sensor_pin				2			// PK2
	#define		sharp_sensor_channel			10			// ADC10 - ADC Channel 10
	
	//---------------------------------- INTERNAL REGISTERS -----------------------------------------------------

	// Bits of EIMSK register  ( External Interrupt Mask Register )
	#define 	EIMSK_reg						EIMSK
	#define 	interrupt_left_encoder_pin		INT4		// 4
	#define 	interrupt_right_encoder_pin		INT5		// 5
	#define 	interrupt_switch_pin			INT7		// 7

	// Bits of EICRB register  ( External Interrupt Control Register B )
	#define 	EICRB_reg						EICRB
	#define 	interrupt_ISC_switch_bit1		ISC71		// 7	( Interrupt Sense Control Bit 1 for INT7 )
	#define 	interrupt_ISC_switch_bit0		ISC70		// 6	( Interrupt Sense Control Bit 0 for INT7 )
	#define 	interrupt_ISC_right_bit1		ISC51		// 3	( Interrupt Sense Control Bit 1 for INT5 )
	#define 	interrupt_ISC_right_bit0		ISC50		// 2	( Interrupt Sense Control Bit 0 for INT5 )
	#define 	interrupt_ISC_left_bit1			ISC41		// 1	( Interrupt Sense Control Bit 1 for INT4 )
	#define 	interrupt_ISC_left_bit0			ISC40		// 0	( Interrupt Sense Control Bit 0 for INT4 )
	
	// Timer / Counter 5 registers
	#define		TCCR5A_reg						TCCR5A		// Timer / Counter Control Register 5A
	#define		TCCR5B_reg						TCCR5B		// Timer / Counter Control Register 5B
	#define		TCCR5C_reg						TCCR5C		// Timer / Counter Control Register 5C
	#define		TCNT5H_reg						TCNT5H		// Timer / Counter 5 High Byte register
	#define		TCNT5L_reg						TCNT5L		// Timer / Counter 5 Low Byte register
	#define		OCR5AH_reg						OCR5AH		// Output Compare Register 5 A High Byte
	#define		OCR5AL_reg						OCR5AL		// Output Compare Register 5 A Low Byte
	#define		OCR5BH_reg						OCR5BH		// Output Compare Register 5 B High Byte
	#define		OCR5BL_reg						OCR5BL		// Output Compare Register 5 B Low Byte
	#define		OCR5CH_reg						OCR5CH		// Output Compare Register 5 C High Byte
	#define		OCR5CL_reg						OCR5CL		// Output Compare Register 5 C Low Byte
	#define		TIMSK5_reg						TIMSK5		// Timer / Counter Interrupt Mask register 5
	#define		TIFR5_reg						TIFR5		// Timer / Counter Interrupt Flag register 5

	// Timer / Counter 4 registers
	#define		TCCR4A_reg						TCCR4A		// Timer / Counter Control Register 4A
	#define		TCCR4B_reg						TCCR4B		// Timer / Counter Control Register 4B
	#define		TCCR4C_reg						TCCR4C		// Timer / Counter Control Register 4C
	#define		TCNT4H_reg						TCNT4H		// Timer / Counter 4 High Byte register
	#define		TCNT4L_reg						TCNT4L		// Timer / Counter 4 Low Byte register
	#define		OCR4AH_reg						OCR4AH		// Output Compare Register 4 A High Byte
	#define		OCR4AL_reg						OCR4AL		// Output Compare Register 4 A Low Byte
	#define		OCR4BH_reg						OCR4BH		// Output Compare Register 4 B High Byte
	#define		OCR4BL_reg						OCR4BL		// Output Compare Register 4 B Low Byte
	#define		OCR4CH_reg						OCR4CH		// Output Compare Register 4 C High Byte
	#define		OCR4CL_reg						OCR4CL		// Output Compare Register 4 C Low Byte
	#define		TIMSK4_reg						TIMSK4		// Timer / Counter Interrupt Mask register 4
	#define		TIFR4_reg						TIFR4		// Timer / Counter Interrupt Flag register 4

	// Timer / Counter 3 registers
	#define		TCCR3A_reg						TCCR3A		// Timer / Counter Control Register 3A
	#define		TCCR3B_reg						TCCR3B		// Timer / Counter Control Register 3B
	#define		TCCR3C_reg						TCCR3C		// Timer / Counter Control Register 3C
	#define		TCNT3H_reg						TCNT3H		// Timer / Counter 3 High Byte register
	#define		TCNT3L_reg						TCNT3L		// Timer / Counter 3 Low Byte register
	#define		OCR3AH_reg						OCR3AH		// Output Compare Register 3 A High Byte
	#define		OCR3AL_reg						OCR3AL		// Output Compare Register 3 A Low Byte
	#define		OCR3BH_reg						OCR3BH		// Output Compare Register 3 B High Byte
	#define		OCR3BL_reg						OCR3BL		// Output Compare Register 3 B Low Byte
	#define		OCR3CH_reg						OCR3CH		// Output Compare Register 3 C High Byte
	#define		OCR3CL_reg						OCR3CL		// Output Compare Register 3 C Low Byte
	#define		TIMSK3_reg						TIMSK3		// Timer / Counter Interrupt Mask register 3
	#define		TIFR3_reg						TIFR3		// Timer / Counter Interrupt Flag register 3

	// Timer / Counter 2 registers
	#define		TCCR2A_reg						TCCR2A		// Timer / Counter Control Register 2A
	#define		TCCR2B_reg						TCCR2B		// Timer / Counter Control Register 2B
	#define		TCNT2_reg						TCNT2		// Timer / Counter 2 register
	#define		OCR2A_reg						OCR2A		// Output Compare Register 2 A
	#define		OCR2B_reg						OCR2B		// Output Compare Register 2 B
	#define		TIMSK2_reg						TIMSK2		// Timer / Counter Interrupt Mask register 2
	#define		TIFR2_reg						TIFR2		// Timer / Counter Interrupt Flag register 2
	
	// Timer / Counter 1 registers
	#define		TCCR1A_reg						TCCR1A		// Timer / Counter Control Register 1A
	#define		TCCR1B_reg						TCCR1B		// Timer / Counter Control Register 1B
	#define		TCCR1C_reg						TCCR1C		// Timer / Counter Control Register 1C
	#define		TCNT1H_reg						TCNT1H		// Timer / Counter 1 High Byte register
	#define		TCNT1L_reg						TCNT1L		// Timer / Counter 1 Low Byte register
	#define		OCR1AH_reg						OCR1AH		// Output Compare Register 1 A High Byte
	#define		OCR1AL_reg						OCR1AL		// Output Compare Register 1 A Low Byte
	#define		OCR1BH_reg						OCR1BH		// Output Compare Register 1 B High Byte
	#define		OCR1BL_reg						OCR1BL		// Output Compare Register 1 B Low Byte
	#define		OCR1CH_reg						OCR1CH		// Output Compare Register 1 C High Byte
	#define		OCR1CL_reg						OCR1CL		// Output Compare Register 1 C Low Byte
	#define		TIMSK1_reg						TIMSK1		// Timer / Counter Interrupt Mask register 1
	#define		TIFR1_reg						TIFR1		// Timer / Counter Interrupt Flag register 1

	// Timer / Counter 0 registers
	#define		TCCR0A_reg						TCCR0A		// Timer / Counter Control Register 0A
	#define		TCCR0B_reg						TCCR0B		// Timer / Counter Control Register 0B
	#define		TCNT0_reg						TCNT0		// Timer / Counter 0 register
	#define		OCR0A_reg						OCR0A		// Output Compare Register 0 A
	#define		OCR0B_reg						OCR0B		// Output Compare Register 0 B
	#define		TIMSK0_reg						TIMSK0		// Timer / Counter Interrupt Mask register 0
	#define		TIFR0_reg						TIFR0		// Timer / Counter Interrupt Flag register 0

	// Bits of TCCRnA register ( Timer / Counter 'n' Control Register A, where n = 0, 1, 2, 3, 4, 5 )
	#define		COMA1_bit						COM5A1		// 7	( Compare Output Mode bit 1 for Channel A )
	#define		COMA0_bit						COM5A0		// 6	( Compare Output Mode bit 0 for Channel A )
	#define		COMB1_bit						COM5B1		// 5	( Compare Output Mode bit 1 for Channel B )
	#define		COMB0_bit						COM5B0		// 4	( Compare Output Mode bit 0 for Channel B )
	#define		COMC1_bit						COM5C1		// 3	( Compare Output Mode bit 1 for Channel C )
	#define		COMC0_bit						COM5C0		// 2	( Compare Output Mode bit 0 for Channel C )
	#define		WGM1_bit						WGM51		// 1	( Waveform Generation Mode bit 1 )
	#define		WGM0_bit						WGM50		// 0	( Waveform Generation Mode bit 0 )
	
	// Bits of TCCRnB register ( Timer / Counter 'n' Control Register B, where n = 0, 1, 2, 3, 4, 5 )
	#define		WGM3_bit						WGM53		// 4	( Waveform Generation Mode bit 3 )
	#define		WGM2_bit						WGM52		// 3	( Waveform Generation Mode bit 2 )
	#define		CS2_bit							CS52		// 2	( Clock Select bit 2 )
	#define		CS1_bit							CS51		// 1	( Clock Select bit 1 )
	#define		CS0_bit							CS50		// 0	( Clock Select bit 0 )
	
	// Bits of TIMSKn register ( Timer / Counter 'n' Interrupt Mask Register, where n = 0, 1, 2, 3, 4, 5 )
	#define		OCIEC_bit						OCIE5C		// 3	( Timer / Counter 'n' Output Compare C Match Interrupt Enable bit )
	#define		OCIEB_bit						OCIE5B		// 2	( Timer / Counter 'n' Output Compare B Match Interrupt Enable bit )
	#define		OCIEA_bit						OCIE5A		// 1	( Timer / Counter 'n' Output Compare A Match Interrupt Enable bit )
	#define		TOIE_bit						TOIE5		// 0	( Timer / Counter 'n' Overflow Interrupt Enable bit )
	
	// Bits of TIFRn register ( Timer / Counter 'n' Interrupt Flag Register, where n = 0, 1, 2, 3, 4, 5 )
	#define		TOV_bit							TOV5		// 0	( Timer / Counter 'n' Overflow Flag bit )
	
	// Bits of ADCSRA register ( ADC Control and Status Register A )
	#define		ADCSRA_reg						ADCSRA
	#define		ADEN_bit						ADEN		// 7	( ADC Enable bit )
	#define		ADSC_bit						ADSC		// 6	( ADC Start Conversion bit )
	#define		ADATE_bit						ADATE		// 5	( ADC Auto Trigger Enable bit )
	#define		ADIF_bit						ADIF		// 4	( ADC Interrupt Flag bit )
	#define		ADIE_bit						ADIE		// 3	( ADC Interrupt Enable bit )
	#define		ADPS2_bit						ADPS2		// 2	( ADC Pre-scalar bit 2 )
	#define		ADPS1_bit						ADPS1		// 1	( ADC Pre-scalar bit 1 )
	#define		ADPS0_bit						ADPS0		// 0	( ADC Pre-scalar bit 0 )
	
	// Bits of ADCSRB register ( ADC Control and Status Register B )
	#define		ADCSRB_reg						ADCSRB
	#define		ACME_bit						ACME		// 6	( Analog Comparator Multiplexer Enable bit )
	#define		MUX5_bit						MUX5		// 3	( ADC Channel Selection bit 5 )
	#define		ADTS2_bit						ADTS2		// 2	( ADC Auto Trigger Source bit 2 )
	#define		ADTS1_bit						ADTS1		// 1	( ADC Auto Trigger Source bit 1 )
	#define		ADTS0_bit						ADTS0		// 0	( ADC Auto Trigger Source bit 0 )
	
	// Bits of ADMUX register ( ADC Multiplexer Selection Register )
	#define		ADMUX_reg						ADMUX
	#define		REFS1_bit						REFS1		// 7	( Reference Selection bit 1 )
	#define		REFS0_bit						REFS0		// 6	( Reference Selection bit 0 )
	#define		ADLAR_bit						ADLAR		// 5	( ADC Left Result bit )
	#define		MUX4_bit						MUX4		// 4	( ADC Channel Selection bit 4 )
	#define		MUX3_bit						MUX3		// 3	( ADC Channel Selection bit 3 )
	#define		MUX2_bit						MUX2		// 2	( ADC Channel Selection bit 2 )
	#define		MUX1_bit						MUX1		// 1	( ADC Channel Selection bit 1 )
	#define		MUX0_bit						MUX0		// 0	( ADC Channel Selection bit 0 )
	
	// Bits of ACSR register ( Analog Comparator Control and Status Register )
	#define		ACSR_reg						ACSR
	#define		ACD_bit							ACD			// 7	( Analog Comparator Disable bit )
	
	// ADC Data registers
	#define		ADCH_reg						ADCH		// ADC High Byte Data register
	#define		ADCL_reg						ADCL		// ADC Low Byte Data register
	
	#define		BAUD_RATE						9600		// Baud rate setting
	
	// Calculate UBRR value for Baud Rate setting according to the F_CPU value
	#define		UBRR_VALUE						( ( ( F_CPU / 16 ) / BAUD_RATE ) - 1 )
	
	// USART2 Data register
	#define		UDR_reg							UDR2		// USART2 Data register
	
	// Bits of UCSR1A register
	#define		UCSRA_reg						UCSR2A		// UCSR2A register
	#define		TXC_bit							TXC2		// 6	( Transmit Complete Flag bit for UART2 )
	#define		UDRE_bit						UDRE2		// 5	( Data Register Empty Flag bit for UART2 )
	#define		U2X_bit							U2X2		// 1	( Double USART Transmission Speed bit for UART2 )
	
	// Bits of UCSR2B register
	#define		UCSRB_reg						UCSR2B		// UCSR2B register
	#define		RXCIE_bit						RXCIE2		// 7	( RX Complete Interrupt Enable bit for UART2 )
	#define		TXCIE_bit						TXCIE2		// 6	( TX Complete Interrupt Enable bit for UART2 )
	#define		UDRIE_bit						UDRIE2		// 5	( USART Data Register Empty Interrupt Enable bit for UART2 )
	#define		RXEN_bit						RXEN2		// 4	( Receiver Enable bit for UART2 )
	#define		TXEN_bit						TXEN2		// 3	( Transmitter Enable bit for UART2 )
	#define		UCSZ2_bit						UCSZ22		// 2	( Character Size selection bit 2 for UART2 )
	
	// Bits of UCSR2C register
	#define		UCSRC_reg						UCSR2C		// UCSR2C register
	#define		UMSEL1_bit						UMSEL21		// 7	( USART Mode Select bit 1 for UART2 )
	#define		UMSEL0_bit						UMSEL20		// 6	( USART Mode Select bit 0 for UART2 )
	#define		UPM1_bit						UPM21		// 5	( Parity Mode Select bit 1 for UART2 )
	#define		UPM0_bit						UPM20		// 4	( Parity Mode Select bit 0 for UART2 )
	#define		USBS_bit						USBS2		// 3	( Stop Bits Select bit for UART2 )
	#define		UCSZ1_bit						UCSZ21		// 2	( Character Size selection bit 1 for UART2 )
	#define		UCSZ0_bit						UCSZ20		// 1	( Character Size selection bit 0 for UART2 )
	#define		UCPOL_bit						UCPOL2		// 0	( Clock Polarity Select bit for UART2 )
	
	// USART2 Baud Rate registers
	#define		UBRRH_reg						UBRR2H		// USART2 High Byte Baud Rate register
	#define		UBRRL_reg						UBRR2L		// USART2 Low Byte Baud Rate register

#endif


#endif /* FIREBIRD_AVR_H_ */