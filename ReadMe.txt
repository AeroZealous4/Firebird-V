(1)
	folder named search_n_rescue have a latest working code

Build Command:
avr_hex_file_generator -cpp src/search_n_rescue -dcpp src/eBot_Sandbox src/lcd src/ADC_Sensor_Display_on_LCD src/Color_Sensor src/Position_Control_Interrupts src/atmega_to_esp32_uart src/uart -dc src/firebird_avr -m atmega2560

(2) cd build
(2)sudo avrdude -c stk500v2 -p m2560 -P /dev/ttyACM0 -U flash:w:file_name.hex:i
(3)fOR esP32, Check com port
	dmesg | grep tty
	putty
	select ttyUSB0 and baud rate of 115200

Rotary Encoder: https://www.youtube.com/watch?v=SQIE1Cr7mzk
(1)Number of slots in disc = 30
(2)Number of pulse/rot =30
(3)Wheel Radius = 52 mm
(5)Resolution of position encoder= pi*d/30 = 5.44 mm traveled/ pulse
(6)Pulse count = distance/5.44

(7)SREG-AVR satus register: 
	(i) Enable global intr: 7th bit: sei(),cli() set and cleargloabll intr
	(ii)EIMSK: External inturrupt mask register:
		set INT5 and INT4 (7,6,..0)
	EIMSK = 0x30
(8)Interrupt sense control bits
	ISCn1 and ISCn0: (1,0) int generated at falling edge
	External interrupt control register A
	EICRB = 0x0A h
(9)ISR: 
	#incluse<avr/interrupt.h>
	PE4 and PE5 as input USING DDRE with pull ups
	EIMSK = 0x30 // To Enable INT4 and INT5
	EICRB = 0x0A	//To falling edge
	sei();	//Enable global interrupt

	ISR(INTn_vect) //n = External interrupt number i.e.4 and 5
	{
	
	}
//------------------------------------------------------------------
		 
Colour Sensor interfacing: https://drive.google.com/drive/folders/1TPrljaSlRFnT5yZN1Gd8J9wUb6i-_Oh5
(1)20%: Output freq: S0: H S1: L
(2)Diode selection:
	Rred: 	S2:L,s3:L
	Blue:	S2:l,S3:H
	Green: 	S2:H,S3:H
(3)Check Pin Diagram on slide no 39
	2,4,6,.............56
	1,3,5,.............55
	All the odd numbered pins are in lower row and all the even numbered pins are in the upper row. These pins are arranged in a SNAKE
pattern as shown by the red mark.

