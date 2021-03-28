/*! \mainpage Experiment: 5_Position_Control_Interrupt
 *
 * @author     e-Yantra Team
 * @date       2020/02/12
 *
 * \subsection Aim
 * To interface position encoders and demonstrate use of it to traverse the robot by specified distance and rotate by specific angle.
 *
 * \subsection Connections
 * Motors Connections: 
 * Motors are connected to the Microcontroller through L293D Motor Driver IC. <br>
 *		 Motors Pin	  	Microcontroller Pin      	<br>
 *			  RB  	--> 	PA3						<br>
 *			  RF  	--> 	PA2						<br>
 *			  LF  	--> 	PA1						<br>
 *			  LB 	--> 	PA0						<br>
 * 
 * PWM Pins of the Microcontroller are connected to the L293D Motor Driver IC.
 *		   PWM Pin	  		Microcontroller Pin      		<br>
 *		  Left Motor  	--> 	PL4	(OC5A)					<br>
 *		  Right Motor  	--> 	PL3	(OC5B)					<br> 
 * 
 * Position Encoders Connections:
 *		   Encode Pin	  			Microcontroller Pin			<br>
 *		  Left Motor Encoder  	--> 	PE4 (INT4)				<br>
 *		  Right Motor Encoder 	--> 	PE5 (INT5)				<br>
 *
 * \subsection Macro Definitions
 * Macros for Motors:	<br>			
 *		motors_dir_ddr_reg			:  DDRA				<br>
 *		motors_dir_port_reg			:  PORTA			<br>
 *		motors_pwm_ddr_reg			:  DDRL				<br>
 *		motors_pwm_port_reg			:  PORTL			<br>
 *		position_encoder_ddr_reg	:  DDRE				<br>
 *		position_encoder_port_reg	:  PORTE			<br>
 *
 *		motors_RB_pin				:  PA3				<br>
 *		motors_RF_pin				:  PA2				<br>
 *		motors_LF_pin				:  PA1				<br>
 *		motors_LB_pin				:  PA0				<br>
 *		motors_pwm_R_pin			:  PL4				<br>
 *		motors_pwm_L_pin			:  PL3				<br>
 *		left_encoder_pin			:  PE4				<br>
 *		right_encoder_pin			:  PE5				<br>
 *		
 * Macros for Position Encoder Interrupt: <br>
 *		EIMSK_reg					:  EIMSK			<br>
 *		EICRB_reg					:  EICRB			<br>
 *
 *		interrupt_left_encoder_pin	:  INT4				<br>
 *		interrupt_right_encoder_pin	:  INT5				<br>
 *		interrupt_ISC_right_bit1	:  ISC51			<br>
 *		interrupt_ISC_right_bit0	:  ISC50			<br>
 *		interrupt_ISC_left_bit1		:  ISC41			<br>
 *		interrupt_ISC_left_bit0		:  ISC40			<br>
 *
 * Note: Make sure that optimization: -O0
 *
 */

// unsigned long int ShaftCountLeft = 0; 	//to keep track of left position encoder 
// unsigned long int ShaftCountRight = 0; 	//to keep track of right position encoder
//---------------------------------- FUNCTIONS ----------------------------------------------------------

//-----------------------------CONFIGURATION FUNCTIONS --------------------------------------------------
int Angle_Rotated(unsigned char status);//status == 1 for Reset angle to zero
/**
 * @brief      Function to configure motor pins
 */
void motors_pin_config(void);

/**
 * @brief      Function to configure left and right channel pins of the L293D Motor Driver IC for PWM
 */
void pwm_pin_config(void);

/**
 * @brief      Function to configure left and right encoder pins
 */
void position_encoder_pin_config (void);

/**
 * @brief      Function to configure external interrupt for encoder pins
 */
void position_encoder_interrupt_config (void);

//----------------------------- INTERRUPT SERVICE ROUTINES ----------------------------------------------

//----------------------------- MOTION RELATED FUNCTIONS ----------------------------------------------

/**
 * @brief      Function to make Firebird-V move forward.
 */
void forward(void); //both wheels forward

/**
 * @brief      Function to make Firebird-V move backward.
 */
void back(void); //both wheels backward


/**
 * @brief      Function to make Firebird-V rotate left.
 */
void left(void); //Left wheel backward, Right wheel forward

/**
 * @brief      Function to make Firebird-V rotate right.
 */
void right(void); //Left wheel forward, Right wheel backward


/**
 * @brief      Function to make Firebird-V rotate soft left.
 */
void soft_left(void); //Left wheel stationary, Right wheel forward

/**
 * @brief      Function to make Firebird-V rotate soft right.
 */
void soft_right(void); //Left wheel forward, Right wheel is stationary

/**
 * @brief      Function to make Firebird-V rotate backward left.
 */
void soft_left_2 (void); //Left wheel backward, right wheel stationary

/**
 * @brief      Function to make Firebird-V rotate backward right.
 */
void soft_right_2 (void); //Left wheel stationary, Right wheel backward

/**
 * @brief      Function to make Firebird-V stop.
 */
void stop (void);

//----------------------------- ENCODER RELATED FUNCTIONS ----------------------------------------------

/**
 * @brief      Function to rotate Firebird-V by specified degrees
 * @param[in]  Degrees   Rotation angle 0 to 360
 */
void angle_rotate(unsigned int Degrees);

/**
 * @brief      Function to move Firebird-V by specified distance
 * @param[in]  DistanceInMM   Distance in mm 0 to 65535
 */
void linear_distance_mm(unsigned int DistanceInMM);

/**
 * @brief      Function to move forward Firebird-V by specified distance
 * @param[in]  DistanceInMM   Distance in mm 0 to 65535
 */
void forward_mm(unsigned int DistanceInMM);

/**
 * @brief      Function to move backward Firebird-V by specified distance
 * @param[in]  DistanceInMM   Distance in mm 0 to 65535
 */
void back_mm(unsigned int DistanceInMM);

/**
 * @brief      Function to rotate Firebird-V left by specified degrees
 * @param[in]  Degrees   Rotation angle 0 to 360
 */
void left_degrees(unsigned int Degrees);

/**
 * @brief      Function to rotate Firebird-V right by specified degrees
 * @param[in]  Degrees   Rotation angle 0 to 360
 */
void right_degrees(unsigned int Degrees);

/**
 * @brief      Function to rotate Firebird-V left by specified degrees
 * @param[in]  Degrees   Rotation angle 0 to 360
 */
void soft_left_degrees(unsigned int Degrees);
/**
 * @brief      Function to rotate Firebird-V right by specified degrees
 * @param[in]  Degrees   Rotation angle 0 to 360
 */
void soft_right_degrees(unsigned int Degrees);

/**
 * @brief      Function to rotate Firebird-V left by specified degrees
 * @param[in]  Degrees   Rotation angle 0 to 360
 */
void soft_left_2_degrees(unsigned int Degrees);

/**
 * @brief      Function to rotate Firebird-V right by specified degrees
 * @param[in]  Degrees   Rotation angle 0 to 360
 */
void soft_right_2_degrees(unsigned int Degrees);

/**
 * @brief      Function to control the speed of both the motors of Firebird-V
 *
 * @param[in]  left_motor   Left motor speed 0 to 255
 * @param[in]  right_motor  Right motor speed 0 to 255
 */
void velocity (unsigned char left_motor, unsigned char right_motor);

/**
 * @brief      Function to initialize Timer 5 in FAST PWM mode for speed control of motors of Firebird-V
 *
 */
void timer5_init();

//---------------------------------- MAIN ----------------------------------------------------------------

//---------------------------------- END ------------------------------------------------------------------
