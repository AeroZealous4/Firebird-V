/*
 * eBot_Sandbox.cpp
 *
 *  Created on: 21-Mar-2021
 *      Author: Akshay Pampatwar (183079007)
 */


//---------------------------------- INCLUDES -----------------------------------
// #define DEBUG 1	//Uncomment for debug msg from esp32 to laptop
#include "eBot_Sandbox.h"
#include <stdio.h>

// #define DEBUG_SAND 1	//Comment if debug msgs are not required
// //------------------------------ GLOBAL VARIABLES -------------------------------

char str[] = "Hello, I am a Firebird--------------------------------------------- V\n";
//Min Vel from going to one node to other node i.e. No_nodes/Sec
float Vel_N_N = 0.27; //Tune it
/**
 * @brief Initializes all peripherals reuired for project
 */
void init_all_peripherals(void)
{
	adc_port_config();					// Initialize the ADC port
	adc_init();							// Initialize the ADC
	lcd_port_config();					// Initialize the LCD port
	lcd_init();							// Initialize the LCD
	init_color_sensor();				//Initialize colour sensor
	filter_clear();	//
	//Related to motors
	motors_pin_config();
	pwm_pin_config();
	position_encoder_pin_config();
	position_encoder_interrupt_config();

	timer5_init(); //For PWM

	Comm_ESP32_init(); //Init timer3 for time keeping and buzzer
	//Uart Initialization
	uart_init1();	//UART3 of atmega
}

void Task_1B(void)
{
	forward_wls(1); //Goes to next node
	
	sprintf(str,"1st Node\n^");
	uart_send_string(str);
	forward_wls(1); //Goes to next node i.e. Mid point of required node
	sprintf(str,"Mid Node\n^");
	uart_send_string(str);
	velocity(Cruise_Vel0,Cruise_Vel0);
	left_degrees(90); //Left turn by 90Degrees
	sprintf(str,"90 Degree Left turn\n^");
	uart_send_string(str);
	velocity(Cruise_Vel,Cruise_Vel);
	forward_mm(180);
	sprintf(str,"Center of block reached\n^");
	uart_send_string(str);

	char Type_Inj = Sense_Color();
	uart3_SendByte(Type_Inj); //Scan and send a required colour
	lcd_clear();
	lcd_wr_char_EE(Type_Inj);

	if(Type_Inj == 'R')
		Set_Major();
	else if(Type_Inj == 'G')
		Set_Minor();
	else
		Set_NoInjury();

	back_mm(180);
	sprintf(str,"Back on line\n^");
	uart_send_string(str);

	left_turn_wls();
	sprintf(str,"Left Turn\n^");
	uart_send_string(str);

	forward_wls(1); //Goes to next node
	sprintf(str,"1st Node\n^");
	uart_send_string(str);

	forward_wls(1); //Goes to next node
	sprintf(str,"Start Location\n^");
	uart_send_string(str);
	sprintf(str,"Finished\n^");
	lcd_string_EE(str);
}

/**
 * @brief  Checks for new cmd and if any then executes it depending on conditions
 * Returns: true if cmd is entertained and false if not
 */
bool Ent_Cmd(void)
{
	int Req_Plot_No=11,Nxt_Node;
	int Dest_Node=66;

	Update_Command();

	if(Is_Command())
	{
		#ifdef DEBUG_SAND
		sprintf(str,"Command Received^");
		uart_send_string(str);
		#endif

		if( Is_Scan() )
		{
			Dest_Node = Min_Dist_Node_Fr_Plot( Scan_Plot_No(),Get_Curr_Node());
			#ifdef DEBUG_SAND
			sprintf(str,"Req to scan Plot Nu:%d\n^",Scan_Plot_No());
			uart_send_string(str);
			#endif
			Req_Plot_No = Scan_Plot_No();

			if( (int) (Get_Dist()/Vel_N_N) < Complete_In() )
			{
				#ifdef DEBUG_SAND
                    sprintf(str,"Cmd Accptd:Time: %d > %d$",(int) ( Get_Dist()/Vel_N_N ),Complete_In() );
                    uart_send_string(str);
                #endif
				Cmd_Accepted(); //Send ack that cmd is accepted
			}
			else{
				
				#ifdef DEBUG_SAND
                    sprintf(str,"Cmd Rejected:Time to comp: %d, In: %d$",(int) ( Get_Dist()/Vel_N_N ),Complete_In() );
                    uart_send_string(str);
                #endif

				Cmd_Ignore();
				return true;//false;
			}
			
		}					
		else if( Is_Major() )
		{
			Req_Plot_No = Fetch_plot_no_req('M',Get_Curr_Node());

			if( Req_Plot_No == -1)
			{
				Cmd_Ignore();
				return true;//false;
			}				
			else
			{	
				if( (int) (Get_Dist()/Vel_N_N) < Complete_In() )
				{
					Dest_Node = Fetch_plot_node_req();
					Set_Plot_Number(Req_Plot_No); 
					Cmd_Accepted(); //Send ack that cmd is accepted
				}
				else{
				
					#ifdef DEBUG_SAND
						sprintf(str,"Cmd Rejected:Time to comp: %d, In: %d^",(int) ( Get_Dist()/Vel_N_N ),Complete_In() );
                   		uart_send_string(str);
					#endif
				Cmd_Ignore();
				return true;//false;
				}				
			}
		}
		else if( Is_Minor() )
		{
			Req_Plot_No = Fetch_plot_no_req('m',Get_Curr_Node());

			if( Req_Plot_No == -1)
			{
				Cmd_Ignore();
				return true;//false;
			}	
			else
			{	

				if( (int) (Get_Dist()/Vel_N_N) < Complete_In() )
				{
					Dest_Node = Fetch_plot_node_req();
					Set_Plot_Number(Req_Plot_No); 
					Cmd_Accepted(); //Send ack that cmd is accepted
				}
				else{
				
					#ifdef DEBUG_SAND
						sprintf(str,"Cmd Rejected:Time to comp: %d, In: %d^",(int) ( Get_Dist()/Vel_N_N ),Complete_In() );
                   		uart_send_string(str);
					#endif
				Cmd_Ignore();
				return true;//false;
				}

			}
		}

		#ifdef DEBUG_SAND
			sprintf(str,"Scan dikstra, Dest node:%d\n^",Dest_Node);
			uart_send_string(str);
		#endif
		dijkstra(Dest_Node);	//Updates path to travel to Dest node



	
		while( Get_Curr_Node() != Dest_Node )	//Goes to next node
		{
			Nxt_Node = Next_Node(Get_Curr_Node());

			#ifdef DEBUG_SAND
				sprintf(str,"Moving to node:%d\n^",Nxt_Node);
				uart_send_string(str);
			#endif

			
			// rotate_comm(Get_Curr_Node(),Get_Curr_Head(),rot_dir);
			turn_head( Next_Dir(Get_Curr_Node(), Nxt_Node) );	//Rotate to desired direction
			forward_comm(Get_Curr_Node(),Nxt_Node);//Sends current node status to esp32
			forward_wls(1);	//Move to next node

			if(Is_Debris())
			{
				// sprintf(str,"Debris Detected: %d to %d$",Get_Curr_Node(),Nxt_Node);
				// uart_send_string(str);

				if( Get_Curr_Head()== 'E')
					turn_head( 'W' );
				else if( Get_Curr_Head()== 'S')
					turn_head( 'N' );
				else if( Get_Curr_Head()== 'W')
					turn_head( 'E' );
				else if( Get_Curr_Head()== 'N')
					turn_head( 'S' );

				forward_wls(1);	//Move to next node

				Adj_Update(Get_Curr_Node(), Nxt_Node, Dest_Node);	

				if(Is_Scan())
					Dest_Node = Min_Dist_Node_Fr_Plot( Scan_Plot_No(),Get_Curr_Node());
				else if( Is_Major() || Is_Minor())
					Dest_Node = Min_Dist_Node_Fr_Plot( Req_Plot_No,Get_Curr_Node());
				// 	Req_Plot_No = Fetch_plot_no_req('M',Get_Curr_Node());
				// else if( Is_Minor() )
				// 	Req_Plot_No = Fetch_plot_no_req('m',Get_Curr_Node());

				dijkstra(Dest_Node);	//Updates path to travel to Dest node
				
			}
			else
				Set_Curr_Node(Nxt_Node);	//Reached to nxt node thus update curr node
		
			#ifdef DEBUG_SAND
			sprintf(str,"Reached node:%d\n^",Nxt_Node);
			uart_send_string(str);
			#endif
		}
		
		if( Dest_Node != 66 )// Next_Plot_to_Scan()!= 17)//Plot reached
		{
			#ifdef DEBUG_SAND
			sprintf(str,"Plot nu %d reached\n^",Req_Plot_No);
			uart_send_string(str);
			#endif
			turn_head_to_plot(Get_Dir_Plot());

			#ifdef DEBUG_SAND
			sprintf(str,"Rotation finished\n^");
			uart_send_string(str);
			#endif

			// forward_mm(180);
				// sprintf(str,"Center of block no: %d reached\n^",Next_Plot_to_Scan());
				// uart_send_string(str);

			char Type_Inj = Sense_Color();	//Scan Injury

			if(Type_Inj == 'R')
			{
				#ifdef DEBUG_SAND
				sprintf(str,"Plot no: %d with Major Injury\n^",Req_Plot_No);
				#endif
				Scan_Res(Req_Plot_No,'M');
				Set_Major();

				// scanned_comm(Req_Plot_No,'M');
			}				
			else if(Type_Inj == 'G')
			{
				#ifdef DEBUG_SAND
				sprintf(str,"Plot no: %d with Minor Injury\n^",Req_Plot_No);
				#endif
				Scan_Res(Req_Plot_No,'m');
				Set_Minor();
			}				
			else
			{
				#ifdef DEBUG_SAND
				sprintf(str,"Plot no: %d with No Injury\n^",Req_Plot_No);
				#endif
				Scan_Res(Req_Plot_No,'N');
				Set_NoInjury(); 
				
			}	
			#ifdef DEBUG_SAND		
				uart_send_string(str);
			#endif

			Task_Complete(); //Req Full filled
			// back_mm(180);
			#ifdef DEBUG_SAND
			sprintf(str,"Plot scan compl^");
			uart_send_string(str);
			#endif
		}

		return true;
	}
	return false;
}
/**
 * @brief  Scan arena and executes any task if encountered before that
 */
bool Task_2B(void)
{
	// bool Is_Finished = false;
	static bool In_Path = false; // If bot is traveling to desired loc then true 
	// static int Current_Node = 0;
	static int Nxt_Node = 0;
	static int Dest_Node = 66;	//Final goal


	// sprintf(str,"In Task 2B\n^");
	// uart_send_string(str);
	//Check if req is there, if it is there then take appropriate action

	if(Ent_Cmd()) //Executes cmd if any first 
	{
		In_Path = false; // Thus scanning of node again starts
	}

	if(In_Path == false)
	{
		if(Inj_at_Plot(Next_Plot_to_Scan()) == '0')
		{

			
			Dest_Node = Min_Dist_Node_Fr_Plot ( Next_Plot_to_Scan(),Get_Curr_Node());

			#ifdef DEBUG_SAND
				sprintf(str,"Before dikstra, Dest node:%d\n$",Dest_Node);
				uart_send_string(str);
			#endif

			dijkstra(Dest_Node);	//Updates path to travel to Dest node
			In_Path = true;

			#ifdef DEBUG_SAND
			sprintf(str,"Des Plot Nu:%d\n$",Next_Plot_to_Scan());
			uart_send_string(str);
			#endif
		}
		else
		{
			Plot_Scan_Compl(); //Current plot is already scanned
			In_Path = false;

			#ifdef DEBUG_SAND
			sprintf(str,"Plot Scan Complete\n$");
			uart_send_string(str);
			#endif
		}
		
	}
	else if( In_Path == true)
	{
		Nxt_Node = Next_Node(Get_Curr_Node());


		if( Get_Curr_Node() != Dest_Node )	//Goes to next node
		{
			#ifdef DEBUG_SAND
			sprintf(str,"Moving to node:%d\n$",Nxt_Node);
			uart_send_string(str);
			#endif
			forward_comm(Get_Curr_Node(),Nxt_Node);//Sends current node status to esp32
			turn_head( Next_Dir(Get_Curr_Node(), Nxt_Node) );	//Rotate to desired direction
			forward_wls(1);	//Move to next node

			if(Is_Debris())
			{
				// sprintf(str,"Debris Detected: %d to %d$",Get_Curr_Node(),Nxt_Node);
				// uart_send_string(str);

				if( Get_Curr_Head()== 'E')
					turn_head( 'W' );
				else if( Get_Curr_Head()== 'S')
					turn_head( 'N' );
				else if( Get_Curr_Head()== 'W')
					turn_head( 'E' );
				else if( Get_Curr_Head()== 'N')
					turn_head( 'S' );

				forward_wls(1);	//Move to next node

				Adj_Update(Get_Curr_Node(), Nxt_Node, Dest_Node);	
				Dest_Node = Min_Dist_Node_Fr_Plot ( Next_Plot_to_Scan(),Get_Curr_Node());
				dijkstra(Dest_Node);	//Updates path to travel to Dest node
			}
			else
				Set_Curr_Node(Nxt_Node);

			In_Path = true;
			
			// sprintf(str,"Reached node:%d\n$",Nxt_Node);
			// uart_send_string(str);
			#ifdef DEBUG_SAND
				sprintf(str,"Reached node:%d\n$",Nxt_Node);
				uart_send_string(str);
			#endif
		}
		else if( Dest_Node != 66 )// Next_Plot_to_Scan()!= 17)//Plot reached
		{
			#ifdef DEBUG_SAND
			sprintf(str,"Plot nu %d reached\n$",Next_Plot_to_Scan());
			uart_send_string(str);
			#endif

			turn_head_to_plot(Get_Dir_Plot());

			#ifdef DEBUG_SAND
			sprintf(str,"Rotation finished\n$");
			uart_send_string(str);
			#endif

			// forward_mm(180);
				// sprintf(str,"Center of block no: %d reached\n^",Next_Plot_to_Scan());
				// uart_send_string(str);

			char Type_Inj = Sense_Color();	//Scan Injury

			if(Type_Inj == 'R')
			{
				#ifdef DEBUG_SAND
				sprintf(str,"Plot no: %d with Major Injury\n$",Next_Plot_to_Scan());
				#endif
				Scan_Res(Next_Plot_to_Scan(),'M');
				scanned_comm(Next_Plot_to_Scan(),'M');
			}				
			else if(Type_Inj == 'G')
			{
				#ifdef DEBUG_SAND
                sprintf(str,"Plot no: %d with Minor Injury\n^",Next_Plot_to_Scan());
				#endif
			    Scan_Res(Next_Plot_to_Scan(),'m');
				scanned_comm(Next_Plot_to_Scan(),'m');	//Send scaned data to esp
			}
			else
			{
				#ifdef DEBUG_SAND
					sprintf(str,"Plot no: %d with No Injury\n$",Next_Plot_to_Scan());
					Scan_Res(Next_Plot_to_Scan(),'n');
				#endif
			}
			#ifdef DEBUG_SAND
				uart_send_string(str);
			#endif
			Plot_Scan_Compl();
			// back_mm(180);

			In_Path = false;

			#ifdef DEBUG_SAND
				sprintf(str,"Plot scan compl$");
				uart_send_string(str);
			#endif
		}
		else if(Dest_Node == 66)	//Next_Plot_to_Scan()==17)//Goal reached
		{
			Plot_Scan_Compl();

			#ifdef DEBUG_SAND
				sprintf(str,"Final Goal 66 reached$");
				uart_send_string(str);
			#endif
			In_Path = false;
			return true;
		}
	}
	// Is_Finished = true;
	return false;
}
/**
 * @brief      Executes the logic to achieve the aim of Project
 */
void Controller(void)
{
	// int return_code;

	init_all_peripherals();

	// while(1)
	// {
	// 	//Debug function for cal debris
	// 	Deb_WL_Debris();
	// 	/*
	// 	Cal:
	// 		White:
	// 		Black:
	// 	Sens:
	// 		White:
	// 		Black:3
	// 	*/
	// }
	
	calibrate();

	// forward_comm(82,77);
	// _delay_ms(3000);
	// rotate_comm(77,'n','r');
	// _delay_ms(3000);
	// forward_comm(77,78);
	// _delay_ms(3000);
	// forward_comm(78,79);
	// _delay_ms(3000);

	// rotate_comm(79,'e','l');
	// _delay_ms(3000);
	// debris_comm(79,'n');
	// _delay_ms(3000);



	
	#ifdef DEBUG_SAND
		sprintf(str,"Cal Done\n$");
		uart_send_string(str);
	#endif

	Update_Command();

	#ifdef DEBUG_SAND
		sprintf(str,"Command Update\n$");
		uart_send_string(str);
	#endif
	while (Task_2B()==false)
	{
		continue;
	}
	#ifdef DEBUG_SAND
		sprintf(str,"Task2B complete\n$");
		uart_send_string(str);
	#endif
	// Task_1B();		//Complete task related to 1B
	// int i=1; 
	while (1)
	{
			/* code */
		Update_Command();
		// _delay_ms(100);	//Delay
		if(Is_Command())
		{
			Cmd_Accepted(); //Send ack that cmd is accepted
			Task_1B();		//Complete task related to 1B
			Task_Complete(); 	
			// i = 0;
		}

	}
	while(1);
	
/*

	#ifdef DEBUG
		sprintf(str,"Cal Done, Min Max\n^");
		uart_send_string(str);
		sprintf(str,"%d %d, %d %d,%d %d\n^",leftMin,leftMax,centerMin,centerMax,rightMin,rightMax);
		uart_send_string(str);
	#endif
*/
}
