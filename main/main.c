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
    int rxBytes = uart_read_bytes(tmcUART, data, writeLen, 1000/portTICK_PERIOD_MS);
    if (rxBytes != writeLen) {
        ESP_LOGI("UART","RX read_bytes echo FAILED (received %d out of %d)", rxBytes,writeLen);
        return false;
    }    
    rxBytes = uart_read_bytes(tmcUART, data, readLen, 1000/portTICK_PERIOD_MS);
    if (rxBytes != readLen) {
        ESP_LOGI("UART","RX read_bytes received FAILED");
        return false;
    }
    return true;
}

// returns true if ok
bool tmc_writeData(uint8_t addr, uint8_t reg, uint32_t data)
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

	return tmc_writeRead(&buf[0], 8, 0);
}

// returns true if ok
bool tmc_readData(uint8_t addr, uint8_t reg, uint32_t *data) 
{
	uint8_t buf[8];

	buf[0] = 0x05;
	buf[1] = addr & 0x03;
	buf[2] = reg & 0x7F;
	buf[3] = tmc_CRC8(buf, 3,1);

    *data = 0;

	if (!tmc_writeRead(&buf[0], 4, 8)){
        return false;
    };

	// Byte 0: Sync nibble correct?
	if (buf[0] != 0x05)
		return false;

	// Byte 1: Master address correct?
	if (buf[1] != 0xFF)
		return false;

	// Byte 2: Address correct?
	if (buf[2] != (reg & 0x7F))
		return false;

	// Byte 7: CRC correct?
	if (buf[7] != tmc_CRC8(buf, 7, 1))
		return false;

	*data = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) | (buf[5] << 8) | buf[6];
    return true;
}

void app_main(void)
{
    tmc_begin(UART_NUM_2);
    if (uart_is_driver_installed(UART_NUM_2)) {
        ESP_LOGI("MAIN","UART driver installed");
    } else {
        ESP_LOGI("MAIN","UART driver instalation FAILED");
    }
    uint32_t status;
    if (tmc_readData(0,0x06,&status)){    
        ESP_LOGI("MAIN","0x06 - Driver IOIN: %lX",status);
        ESP_LOGI("MAIN","============================");        
        ESP_LOGI("MAIN","       ENN - %d",(uint8_t)(status & 0x001));
        ESP_LOGI("MAIN","       MS1 - %d",(uint8_t)((status >> 2) & 0x001));
        ESP_LOGI("MAIN","       MS2 - %d",(uint8_t)((status >> 3) & 0x001));
        ESP_LOGI("MAIN","      DIAG - %d",(uint8_t)((status >> 4) & 0x001));
        ESP_LOGI("MAIN","  PDN_UART - %d",(uint8_t)((status >> 6) & 0x001));
        ESP_LOGI("MAIN","      STEP - %d",(uint8_t)((status >> 7) & 0x001));
        ESP_LOGI("MAIN","SPREAD_ENN - %d",(uint8_t)((status >> 8) & 0x001));
        ESP_LOGI("MAIN","       DIR - %d",(uint8_t)((status >> 9) & 0x001));
        ESP_LOGI("MAIN","   VERSION - %x",(uint8_t)(status >> 24));
    } else {
        ESP_LOGI("MAIN","0x06 - Driver IOIN read failed");
    }
    tmc_end();
    ESP_LOGI("MAIN","UART driver removed");
    while (1)
    {
        vTaskDelay(1000);
        ESP_LOGI("MAIN","Tick ...");
    }
   
}
