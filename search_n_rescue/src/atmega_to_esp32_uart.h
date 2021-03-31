/*
TITLE	: UART Communication: ATmega 2560 & ESP32
DATE	: 2019/11/12
AUTHOR	: e-Yantra Team

AIM: To send data on UART#0 of ATMega 2560 to ESP32
*/

// #define PIN_LED_GREEN	PH5		// Macro for Pin Number of Green Led
// #define MESSAGE			"Tx from ATmega 2560> Hello ESP32!\n" // Message to be sent on UART #0

// volatile unsigned long int count = 0;	// Used in ISR of Timer2 to store ms elasped
// unsigned long seconds = 0;			// Stores seconds elasped
// char rx_byte;
// char str[] = "Hello, I am a Firebird-------- V\n";
void uart_init1(void);
// void init_timer2(void);s


void init_led(void);

void led_greenOn(void);

void led_greenOff(void);
		
char uart3_readByte(void);

void uart3_SendByte(char rx_byte);



void uart_send_string(char * Str);
