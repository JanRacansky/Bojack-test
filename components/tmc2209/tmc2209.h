#include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
// #include "string.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "tmc2209_reg.h"

bool tmc2209_uart_begin(uart_port_t uart,int baud_rate,int tx_io_num, int rx_io_num);
bool tmc2209_begin();
void tmc2209_end();

bool tmc2209_writeData(uint8_t addr, uint8_t reg, uint32_t data);
bool tmc2209_readData(uint8_t addr, uint8_t reg, uint32_t *data);