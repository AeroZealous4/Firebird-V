/*
Files deal with communication between Robot and ESP32 over UART3 of atmega 2560.
ESP32 sends data with format defined in ble/esp32-ble-two-way/ReadMe.txt file.
Functions in this file incapsulates and creates abstraction between incoming data and outgoing
data. 
This also takes care of time keeping
Following APIS can be used for processing of communication
*/
#ifndef COMM_ESP32_H_
#define COMM_ESP32_H_

#define DEBUG_COMM 1 //Comment if debug MSG is not required
                     //Debug data will be sent to ESP32

void Comm_ESP32_init(void); //Inititalizes timer
void Update_Command(void); //Updates any command received from ESP32
bool Is_Command(void); //Returns true if valid command is recieved, Retuns false after reading it once
// bool Is_Fetch(void); //Returns true if command is of fetch type   
bool Is_Scan(void); //Returns true if command is of scan type
int Scan_Plot_No(void); //Returns plot number which needs to be scanned
bool Is_Major(void); //Returns true if Major injury needs to be fetched
bool Is_Minor(void); //Returns true if Minor injury needs to be fetched
int Complete_In(void);  //Returns complete in time for command received

/*Important: Call one of the following after reception of command,*/
//Sends ACK to ESP32
void Cmd_Accepted(void); //Sends ack to ESP32 indicating acceptance of cmd
void Cmd_Ignore(void); //Sends msg to ESP32 i.e. Cmd is rejected/flushed and will not be executed

unsigned int Time_Completed(void); //Returns time in secs after reception of valid command

/*After completion of task*/
//Only one function needs to be called 
    //Call corresponding fn for response to scan request 
    void Set_Major(void);
    void Set_Minor(void);
    void Set_NoInjury(void); 
    //Call if it was fetch req
    void Set_Plot_Number(int); //Send plot number of fetched request

//Finally calll this fn to push data to ESP32
void Task_Complete(void);   //Sends msg to ESP32 about conveying completion of last task 

#endif