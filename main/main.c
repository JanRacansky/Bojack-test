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
#include "tmc2209.h"

#define TMC_EN_PIN GPIO_NUM_6

bool TMConfError = true;    //  helper

void app_main(void)
{
    if (tmc2209_uart_begin(UART_NUM_2, 200000, GPIO_NUM_16, GPIO_NUM_15)){   //  GPIO_NUM_17 GPIO_NUM_18 for Bojack, GPIO_NUM_16 GPIO_NUM_15 for Dev
        ESP_LOGI("MAIN","UART driver installed");    
        if (tmc2209_begin()){
            ESP_LOGI("MAIN","TMC Setup done");  

            // TMC ENable
            gpio_set_direction(TMC_EN_PIN, GPIO_MODE_OUTPUT);
            gpio_set_drive_capability(TMC_EN_PIN, GPIO_DRIVE_CAP_3);
              
            union tmc2209_vactual velocity = {
                .d = 0,
                .vactual = 0
            };

            while (1)
            {
                vTaskDelay(1000);

                if (!TMConfError) {
                    gpio_set_level(TMC_EN_PIN, 0);
                    
                    velocity.vactual = 20000;
                    tmc2209_writeData(0,TMC2209_R_VACTUAL_W,velocity.d);
                    ESP_LOGI("MAIN","VACTUAL:= %lX",velocity.d);

                    vTaskDelay(1000);

                    velocity.vactual = 0;
                    tmc2209_writeData(0,TMC2209_R_VACTUAL_W,velocity.d);
                    ESP_LOGI("MAIN","VACTUAL:= %lX",velocity.d);

                    gpio_set_level(TMC_EN_PIN, 1);
                }
            }
        };
        tmc2209_end();
        ESP_LOGI("MAIN","UART driver removed");    
    };
    
}