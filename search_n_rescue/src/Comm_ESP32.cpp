#define F_CPU 14745600
#include <stdio.h>
#include <string.h>
#include "Comm_ESP32.h"
#include "uart.h"
#include "atmega_to_esp32_uart.h"
#include <avr/io.h>				// Standard AVR IO Library
#include <util/delay.h>			// Standard AVR Delay Library
#include <avr/interrupt.h>		// Standard AVR Interrupt Library
#include "firebird_avr.h"		// Header file included that contains macro definitions essential for Firebird V robot

#define DEBUG_COMM 1 

#define delay_uart 100 //Uart not working if delay is not provided , Tune for optimal performance

bool Fetch_Minor = false;
bool Fetch_Major = false;
bool Scan = false;
bool Cmd_Rx = false;

bool Res_Minor = false;
bool Res_Major = false;
bool Res_NoInjury = false;

int plot_number = 1;    //Plot no which needs tobe scanned
int Complete_In_Time = 1;

int Plot_number_fetch = 1; //Plot no. which is fetched for req fetch

bool Buzzer_state = false;
int Buzzer_Time = 0;


char str_temp[] = "Hello, I am a Firebird-------- V\n";

volatile unsigned int count_ms = 0;	// Used in ISR of Timer2 to store ms elasped
unsigned int count_seconds = 0;			// Stores seconds elasped

void buzzer_pin_config (void) ;
void buzzer_off (void);
void buzzer_on (void);
void Stop_Timer(void);

void init_timer2(void){
	cli();	// Turn off global interrupts

	//Setup Timer2 to fire every 1ms
	TCCR2B = 0x00;        						// Cut off Clock Source to disbale Timer2 while we set it up
	TCNT2  = 130;         						// Reset Timer Count to 130 out of 255
	TIFR2  &= ~(1 << TOV2);        				// Timer2 INT Flag Reg: Clear Timer Overflow Flag
	TIMSK2 |= (1 << TOIE2);        				// Timer2 INT Reg: Timer2 Overflow Interrupt Enable
	TCCR2A = 0x00;        						// Timer2 Control Reg A: Wave Gen Mode normal
	TCCR2B |= (1 << CS22);// | (1 << CS20);        // Timer2 Control Reg B: Timer Prescaler set to 128 and Start Timer2

	sei();	// Turn on global interrupts
}


//Timer2 Overflow Interrupt Vector
ISR(TIMER2_OVF_vect) {
  count_ms++;	// increment after 1 ms               
  
  // increment seconds variable after 1000 ms
  if(count_ms > 999){
	count_seconds++;	
	// uart3_puts(MESSAGE);    // Send data on UART #0 after 1 second
    count_ms = 0;  

    if(Buzzer_state)
    {
        if(Buzzer_Time++>1)
        {
            buzzer_off();
            Stop_Timer();
            Buzzer_state = false;
            Buzzer_Time = 0;
        }
    }        
  }

  
  TCNT2 = 130;           	// Reset Timer to 130 out of 255
  TIFR2  &= ~(1 << TOV2);	// Timer2 INT Flag Reg: Clear Timer Overflow Flag
};

void Start_Timer(void) //Resets timer
{
    count_seconds = 0;
    count_ms = 0; 
    TCCR2B |= (1 << CS20);  //Starts timer
}
void Stop_Timer(void)   //Stops timer
{
    TCCR2B &= ~(1 << CS20);  //Starts timer
}

void Comm_ESP32_init(void)
{
    init_timer2();
    buzzer_pin_config();	
}
void Update_Command(void) //Updates any command received from ESP32
{
    // char testStr[] ="Request accepted------------------------------";
    // int index = 0;
    // String d = "";
    // int flag = 0;

    char ch_msg = uart3_readByte();

    if(ch_msg!= -1)
    {
        // #ifdef DEBUG_COMM
        //     sprintf(str_temp,"Comm_ESP32:");
        //     uart_send_string(str_temp);
        // #endif

        if( (ch_msg!= -1) && ch_msg == 'S')
        {
            #ifdef DEBUG_COMM
                sprintf(str_temp,"S");
                uart_send_string(str_temp);
            #endif

            _delay_ms(delay_uart);
            ch_msg = uart3_readByte();

            if(ch_msg!= -1)
            {
                plot_number = (int) ch_msg;

                #ifdef DEBUG_COMM
                    sprintf(str_temp,"-%d",(unsigned int) ch_msg);
                    uart_send_string(str_temp);
                #endif

                _delay_ms(delay_uart);
                ch_msg = uart3_readByte();

                if(ch_msg!= -1)
                {
                    Complete_In_Time = (int) ch_msg;

                #ifdef DEBUG_COMM
                    sprintf(str_temp,"-%d\n",(unsigned int) ch_msg);
                    uart_send_string(str_temp);
                #endif

                    Start_Timer();
                    Fetch_Minor = false;
                    Fetch_Major = false;
                    Scan = true;
                    Cmd_Rx = true;
                }
            }
        }
        else if(ch_msg == 'F')
        {
            _delay_ms(delay_uart);
            ch_msg = uart3_readByte();

            #ifdef DEBUG_COMM
                sprintf(str_temp,"F");
                uart_send_string(str_temp);
            #endif

            if(( ch_msg!= -1) && (ch_msg == 'M') )
            {
                #ifdef DEBUG_COMM
                    sprintf(str_temp,"M-");
                    uart_send_string(str_temp);
                #endif

                _delay_ms(delay_uart);
                ch_msg = uart3_readByte();

                if(ch_msg!= -1)
                {
                    Complete_In_Time = (int) ch_msg;

                    #ifdef DEBUG_COMM
                        sprintf(str_temp,"%d\n",(unsigned int) ch_msg);
                        uart_send_string(str_temp);
                    #endif
                    Start_Timer();
                    Fetch_Major= true;
                    Fetch_Minor = false;
                    Scan = false;
                    Cmd_Rx = true;
                }
                     
            }
            else if(( ch_msg!= -1) && (ch_msg == 'm') )
            {

                #ifdef DEBUG_COMM
                    sprintf(str_temp,"m-");
                    uart_send_string(str_temp);
                #endif

                _delay_ms(delay_uart);
                ch_msg = uart3_readByte();

                if(ch_msg!= -1)
                {
                    Complete_In_Time = (int) ch_msg;

                #ifdef DEBUG_COMM
                    sprintf(str_temp,"%d\n",(unsigned int) ch_msg);
                    uart_send_string(str_temp);
                    sprintf(str_temp,"Cmd Rx\n");
	                uart_send_string(str_temp);
                #endif
                    Start_Timer();
                    Fetch_Minor = true;
                    Fetch_Major= false;
                    Scan = false;
                    Cmd_Rx = true;

                    uart3_flush();
                }                     
            }
        }
    }
}

bool Is_Command(void)
{
    if(Cmd_Rx)
    {

        #ifdef DEBUG_COMM
            sprintf(str_temp,"Is_Cmd:true\n");
            uart_send_string(str_temp);
        #endif 
        Cmd_Rx = false;
        return true;
    }
    return false;
}
int Scan_Plot_No(void) //Returns plot number which needs to be scanned
{
    return plot_number;
}
bool Is_Scan(void) //Returns true if Major injury needs to be fetched
{
    return Scan;
}
bool Is_Major(void) //Returns true if Major injury needs to be fetched
{
    return Fetch_Major;
}
bool Is_Minor(void) //Returns true if Minor injury needs to be fetched
{
    return Fetch_Minor;
}
int Complete_In(void)  //Returns complete in time for command received
{
    return Complete_In_Time;
}

//Call one of the following after reception of command
void Cmd_Accepted(void) //Sends ack to ESP32 indicating acceptance of cmd
{
    sprintf(str_temp,"accepted");
    uart_send_string(str_temp);
}
void Cmd_Ignore(void) //Sends msg to ESP32 i.e. Cmd is rejected/flushed and will not be executed
{
    sprintf(str_temp,"ignore");
    uart_send_string(str_temp);   
    Stop_Timer();
}

unsigned int Time_Completed(void) //Returns time in secs after reception of valid command
{
    return count_seconds;
}

//Call corresponding fn for response to scan request 
//Only one function needs to be called 
void Set_Major(void)
{
    Res_Minor = false;
    Res_Major = true;
    Res_NoInjury = false;
}
void Set_Minor(void)
{
    Res_Minor = true;
    Res_Major = false;
    Res_NoInjury = false;
}
void Set_NoInjury(void)
{
    Res_Minor = false;
    Res_Major = false;
    Res_NoInjury = true;  
}
//If request was of type fetch
void Set_Plot_Number(int plot_no_fetched)
{
     Plot_number_fetch = plot_no_fetched;
}
//Finally calll this fn to push data to ESP32
void Task_Complete(void)   //Sends msg to ESP32 about conveying completion of last task
{
    // Stop_Timer();        //Will stop after 2 sec completes
    Buzzer_state = true;    //Will be false after 2 secs
    buzzer_on();
    // unsigned int case_no;
    // case_no = Res_NoInjury * 4
    if(Is_Scan())
    {
        switch ( (Res_NoInjury<<2)|(Res_Major<<1)|Res_Minor ) 
        {
        case 0:
            sprintf(str_temp,"Error in task completion");
            break;
        case 1:
            sprintf(str_temp,"minor-%d", Time_Completed() );
            break;   
        case 2:
            sprintf(str_temp,"major-%d", Time_Completed() );
            break;  
        case 4:
            sprintf(str_temp,"no-%d", Time_Completed() );
            break; 
        default:
            sprintf(str_temp,"Scan-Error");
            break;
        }
    }
    else if(Is_Major() || Is_Minor())
        sprintf(str_temp,"fetch-%d-%d",Plot_number_fetch,Time_Completed() );
    else
        sprintf(str_temp,"Error decoding task");

    uart_send_string(str_temp);
} 

//---------------------------------- FUNCTIONS ----------------------------------------------------------

//-----------------------------CONFIGURATION FUNCTIONS --------------------------------------------------

/**
 * @brief      Function to make **ONLY** 'buzzer_pin' as output and initially set it to low
 */
void buzzer_pin_config (void) {

	// Make 'buzzer_pin' as output
	buzzer_ddr_reg	|= ( 1 << buzzer_pin );
	
	// Set 'buzzer_pin' to low initially to turn it OFF
	buzzer_port_reg &= ~( 1 << buzzer_pin );
}

//----------------------------- BUZZER RELATED FUNCTIONS -----------------------------------------------

/**
 * @brief      Function to set **ONLY** 'buzzer_pin' to high, hence turn ON the Buzzer
 */
void buzzer_on (void) {

	// Set 'buzzer_pin' to high to turn it ON
	buzzer_port_reg |= ( 1 << buzzer_pin );
}


/**
 * @brief      Function to set **ONLY** 'buzzer_pin' to low, hence turn OFF the Buzzer
 */
void buzzer_off (void) {

	// Set 'buzzer_pin' to low to turn it OFF
	buzzer_port_reg &= ~( 1 << buzzer_pin );
}
