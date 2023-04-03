/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "string.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "CRC.h"

#define BUF_SIZE 1024


// tmc library global variables
uart_port_t tmcUART = UART_NUM_2;

void tmc_begin(uart_port_t uart) {
    const uart_config_t uart_config = {
        .baud_rate = 250000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    tmcUART = uart;
    uart_driver_install(tmcUART, BUF_SIZE, 0, 0, NULL, 0); // we not use TX buffer, so uart_write_bytes is blocking
    uart_param_config(tmcUART, &uart_config);
    uart_set_pin(tmcUART, GPIO_NUM_17, GPIO_NUM_18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    // CRC table initialization 
    tmc_fillCRC8Table(0x07, true, 1);
}

void tmc_end() {
    uart_driver_delete(tmcUART);
}

// returns false if operation fails
bool tmc_writeRead(uint8_t *data, size_t writeLen, size_t readLen){
    uart_flush_input(tmcUART);
    int txBytes = uart_write_bytes(tmcUART, data, writeLen);
    if (txBytes !=writeLen){
        ESP_LOGI("UART","TX write_bytes FAILED");
        return false;
    }
    int rxBytes = uart_read_bytes(tmcUART, data, writeLen, 4/portTICK_PERIOD_MS);
    if (rxBytes != writeLen) {
        ESP_LOGI("UART","RX read_bytes echo FAILED");
        return false;
    }
    rxBytes = uart_read_bytes(tmcUART, data, readLen, 4/portTICK_PERIOD_MS);
    if (rxBytes != readLen) {
        ESP_LOGI("UART","RX read_bytes received FAILED");
        return false;
    }
    return true;
}

void tmc_writeData(uint8_t addr, uint8_t reg, uint32_t data)
{
	uint8_t buf[8];

	buf[0] = 0x05;
	buf[1] = addr & 0x03;
	buf[2] = (reg & 0x7F) | 0x80; // write flag
	buf[3] = (data >> 24) & 0xFF;
	buf[4] = (data >> 16) & 0xFF;
	buf[5] = (data >> 8) & 0xFF;
	buf[6] = data & 0xFF;
	buf[7] = tmc_CRC8(buf, 7,1);

	tmc_writeRead(&buf[0], 8, 0);
}

uint32_t tmc_readData(uint8_t addr, uint8_t reg)
{
	uint8_t buf[8];

	buf[0] = 0x05;
	buf[1] = addr & 0x03;
	buf[2] = reg & 0x7F;
	buf[3] = tmc_CRC8(buf, 3,1);

	tmc_writeRead(&buf[0], 4, 8);

	// Byte 0: Sync nibble correct?
	if (buf[0] != 0x05)
		return 0;

	// Byte 1: Master address correct?
	if (buf[1] != 0xFF)
		return 0;

	// Byte 2: Address correct?
	if (buf[2] != (reg & 0x7F))
		return 0;

	// Byte 7: CRC correct?
	if (buf[7] != tmc_CRC8(buf, 7, 1))
		return 0;

	return ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) | (buf[5] << 8) | buf[6];
}

void app_main(void)
{
    tmc_begin(UART_NUM_2);
    if (uart_is_driver_installed(UART_NUM_2)) {
        ESP_LOGI("MAIN","UART driver installed");
    } else {
        ESP_LOGI("MAIN","UART driver instalation FAILED");
    }
    tmc_end();
    ESP_LOGI("MAIN","End of test");
    while (1)
    {
        vTaskDelay(1000);
        ESP_LOGI("MAIN","Tick ...");
    }
   
}
