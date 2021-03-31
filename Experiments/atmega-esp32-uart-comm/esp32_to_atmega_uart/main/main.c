/* 
TITLE	: UART Communication: ESP32 & ATmega 2560
DATE	: 2019/11/12
AUTHOR	: e-Yantra Team
*/

#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "driver/uart.h"
#include "driver/gpio.h"

#define ECHO_TEST_TXD  (GPIO_NUM_32)            // Connected to AVR Rx-0
#define ECHO_TEST_RXD  (GPIO_NUM_33)            // Connected to AVR Tx-0
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (1024)

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    uint8_t *data_uart = (uint8_t *) malloc(BUF_SIZE);

    	int len_uart = 0;
        unsigned int count = 0;
        char ack_buffer[100];

        while (1) {
            
            // Read data from the UART
            len_uart = uart_read_bytes(UART_NUM_1, data_uart, BUF_SIZE, 20 / portTICK_RATE_MS);
            data_uart[len_uart] = NULL;

            if(len_uart > 0 && data_uart[0] == 'T'){
                
                printf("Rx from ATmega 2560: %s", (char*) data_uart);

                // Give ACK to ATmega 2560
	        	sprintf(ack_buffer, "Rx from ESP32: [%d] Hello ATmega2560!\n", count++);
	        	ack_buffer[strlen(ack_buffer)] = NULL;

	       		uart_write_bytes(UART_NUM_1, (const char *) ack_buffer, strlen(ack_buffer));
	       		len_uart = 0;
            }

        }
}


