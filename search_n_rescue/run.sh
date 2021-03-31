avr_hex_file_generator -cpp src/search_n_rescue -dcpp src/eBot_Sandbox src/lcd src/ADC_Sensor_Display_on_LCD src/Color_Sensor src/Position_Control_Interrupts src/atmega_to_esp32_uart src/uart src/Comm_ESP32 src/Bot_Motion -dc src/firebird_avr -m atmega2560
dmesg | grep tty
sudo avrdude -c stk500v2 -p m2560 -P /dev/ttyACM0 -U flash:w:build/search_n_rescue.hex:i


