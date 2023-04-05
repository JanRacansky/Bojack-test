/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
// #include "string.h"
// #include "driver/uart.h"
// #include "driver/gpio.h"
#include "tmc2209.h"

void app_main(void)
{
    tmc2209_begin(UART_NUM_2);
    if (uart_is_driver_installed(UART_NUM_2)) {
        ESP_LOGI("MAIN","UART driver installed");
    } else {
        ESP_LOGI("MAIN","UART driver instalation FAILED");
    }
    uint32_t status;
    if (tmc2209_readData(0,0x06,&status)){    
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
    tmc2209_end();
    ESP_LOGI("MAIN","UART driver removed");
    while (1)
    {
        vTaskDelay(1000);
    }
   
}
