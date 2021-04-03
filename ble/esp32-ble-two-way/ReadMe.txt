BLE communication from ESP32 to Laptop (ESP32 BLE Address: "84:CC:A8:5F:90:D6")
(1) Two characteristics:
	(1-1) ledCharacteristic: Read & write,
		(a) WRITE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
		(b) Used for comunication from laptop to ESP32
		(c) Send data in a ASCII string, Following are valid commands (Do not send any other commands, else ESP32 will crash)
			*Do not send complete_in time > 255*
			(c-1) "scan-<plot_num>-<complete_in>": plot_number is number from 1 to 16, eg "scan-5-20"
			(c-2) "fetch-major-<complete_in>": eg: fetch-major-40
			(c-3) "fetch-minor-<complete_in>": eg: fetch-minor-40
			(c-4) "no-request"
	(1-2) buttonCharacteristic: Notify
		(a)	NOTIFY_CHARACTERISTIC_UUID "8801f158-f55e-4550-95f6-d260381b99e7"
		(b) Used for communication from ESP32 to laptop
		(c) Following data are possible valid data sent & are in ASCII string format, Ignore other strings which are for debugging
			(c-1) "Invalid_Plot_number" : If plot number sent from (1-1)(c-1) is invalid
			(c-2) "Invalid_Injury" : If injury type is invalid
			(c-3) "Invalid_Command_received" : Commands sent (1-1)(c) are invalid 
			(c-4) "accepted" : Request sent from laptop is accepted
			(c-5) Completion_Time : It is time taken in seconds for completing requested task, it also indicates that task completed successfully, 
					e.g. for scan request:
						"minor-25" : minor injury
						"major-25" : major injury
						"no-25" : no injury
					e.g. for fetch nearest request:
						"fetch-<plot_num>-25"

			(c-6) "ignore" : If robot decides to ignore requested task then this string will be sent
(2) LED on ESP32: It will toggle each time new request is received from laptop to ESP32
