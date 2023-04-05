#include "tmc2209.h"
#include "CRC.h"

#define BUF_SIZE 1024

// tmc library global variables
uart_port_t tmcUART = UART_NUM_2;

// returns ture if ok, uses ESP_LOGI to log errors
bool tmc2209_begin(uart_port_t uart,int baud_rate,int tx_io_num, int rx_io_num) {
    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
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
    uart_set_pin(tmcUART, tx_io_num, rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (uart_is_driver_installed(tmcUART)) {
        ESP_LOGI("MAIN","UART driver installed");
        // CRC table initialization 
        tmc_fillCRC8Table(0x07, true, 1);
        return true;
    } else {
        ESP_LOGI("MAIN","UART driver instalation FAILED");
        return false;
    }
}

void tmc2209_end() {
    uart_driver_delete(tmcUART);
}

// returns ture if ok, uses ESP_LOGI to log errors
bool tmc2209_writeRead(uint8_t *data, size_t writeLen, size_t readLen){
    uart_flush_input(tmcUART);
    int txBytes = uart_write_bytes(tmcUART, data, writeLen);
    if (txBytes !=writeLen){
        ESP_LOGI("TMC","TX write_bytes FAILED");
        return false;
    }
    int rxBytes = uart_read_bytes(tmcUART, data, writeLen, 1000/portTICK_PERIOD_MS);
    if (rxBytes != writeLen) {
        ESP_LOGI("TMC","RX read_bytes echo FAILED (received %d out of %d)", rxBytes,writeLen);
        return false;
    }    
    rxBytes = uart_read_bytes(tmcUART, data, readLen, 1000/portTICK_PERIOD_MS);
    if (rxBytes != readLen) {
        ESP_LOGI("TMC","RX read_bytes received FAILED");
        return false;
    }
    return true;
}

// returns ture if ok, uses ESP_LOGI to log errors
bool tmc2209_writeData(uint8_t addr, uint8_t reg, uint32_t data)
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

	return tmc2209_writeRead(&buf[0], 8, 0);
}

// returns ture if ok, uses ESP_LOGI to log errors
bool tmc2209_readData(uint8_t addr, uint8_t reg, uint32_t *data) 
{
	uint8_t buf[8];

	buf[0] = 0x05;
	buf[1] = addr & 0x03;
	buf[2] = reg & 0x7F;
	buf[3] = tmc_CRC8(buf, 3,1);

    *data = 0;

	if (!tmc2209_writeRead(&buf[0], 4, 8)){
        return false;
    };

	// Byte 0: Sync nibble correct?
	if (buf[0] != 0x05) {
        ESP_LOGI("TMC","Read initial nibble is not 0x05 ERROR");
        return false;
    }

	// Byte 1: Master address correct?
	if (buf[1] != 0xFF) {
        ESP_LOGI("TMC","Read master address is not 0xFF ERROR");
        return false;
    }

	// Byte 2: Address correct?
	if (buf[2] != (reg & 0x7F)) {
        ESP_LOGI("TMC","Read register address changed ERROR");
        return false;
    }

	// Byte 7: CRC correct?
	if (buf[7] != tmc_CRC8(buf, 7, 1)) {
        ESP_LOGI("TMC","Read - bad CRC");
        return false;
    }

	*data = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) | (buf[5] << 8) | buf[6];
    return true;
}
