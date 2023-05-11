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
    if (tmc2209_begin(UART_NUM_2, 200000, GPIO_NUM_17, GPIO_NUM_18)){   //  GPIO_NUM_17 GPIO_NUM_18 for Bojack, GPIO_NUM_16 GPIO_NUM_15 for Dev
        // test status registru - version musí být 0x21
        union tmc2209_ioin status;
        if (tmc2209_readData(0,TMC2209_R_IOIN_R,&status.d)){    
            ESP_LOGI("MAIN","0x06 - Driver IOIN: %lX",status.d);
            ESP_LOGI("MAIN","============================");        
            ESP_LOGI("MAIN","       ENN - %d",status.enn);
            ESP_LOGI("MAIN","       MS1 - %d",status.ms1);
            ESP_LOGI("MAIN","       MS2 - %d",status.ms2);
            ESP_LOGI("MAIN","      DIAG - %d",status.diag);
            ESP_LOGI("MAIN","  PDN_UART - %d",status.pdn_uart);
            ESP_LOGI("MAIN","      STEP - %d",status.step);
            ESP_LOGI("MAIN","SPREAD_ENN - %d",status.spread_enn);
            ESP_LOGI("MAIN","       DIR - %d",status.dir);
            ESP_LOGI("MAIN","   VERSION - %X",status.version);
            if (status.version!=0x21){
                ESP_LOGI("MAIN","0x06 - IOIN.vertsion != 0x21");
            } else {
                union tmc2209_chopconf chopconf = { 
                    .d = 0,
                    .toff = 0, .hstrt = 5, .vsense = 1, .intpol = 1, .tbl = 2   
                };
                ESP_LOGI("MAIN","CHOPCONF:= %lX",chopconf.d);
                tmc2209_writeData(0,TMC2209_R_CHOPCONG_RW,chopconf.d);

                union tmc2209_gconf gconf = {
                    .d = 0,
                    .pdn_disable = 1, .mstep_reg_select = 1, .multistep_filt = 1
                };
                ESP_LOGI("MAIN","GCONF:= %lX",gconf.d);
                tmc2209_writeData(0,TMC2209_R_GCONF_RW,gconf.d);

                union tmc2209_ihold_irun ihr = {
                    .d = 0,
                    .iholddelay = 1, .irun = 30, .ihold = 11    // set irun=31, ihold=16 for full power
                };
                tmc2209_writeData(0,TMC2209_R_IHOLD_IRUN_W,ihr.d);
                ESP_LOGI("MAIN","IHOLD_IRUN:= %lX",ihr.d);

                union tmc2209_tpowerdown tpw = {
                    .d = 0,
                    .tpowerdown = 20
                };
                tmc2209_writeData(0,TMC2209_R_TPOWERDOWN_W,tpw.d);
                ESP_LOGI("MAIN","TPOWERDOWN:= %lX",tpw.d);

                union tmc2209_pwmconf pwm = {
                    .d=0,
                    .pwm_ofs = 36, .pwm_freq = 1, .pwm_autoscale = 0, .pwm_autograd = 0, .pwm_reg = 1, .pwm_lim = 12
                };
                tmc2209_writeData(0,TMC2209_R_PWMCONF_RW,pwm.d);
                ESP_LOGI("MAIN","PWMCONF:= %lX",pwm.d);
                
                chopconf.toff = 3; chopconf.tbl = 2; chopconf.hstrt = 4; chopconf.hend = 0;
                tmc2209_writeData(0,TMC2209_R_CHOPCONG_RW,chopconf.d);
                ESP_LOGI("MAIN","CHOPCONF:= %lX",chopconf.d);
                
                TMConfError = false;

                gpio_set_direction(TMC_EN_PIN, GPIO_MODE_OUTPUT);
                gpio_set_drive_capability(TMC_EN_PIN, GPIO_DRIVE_CAP_3);
            }
        } else {
            ESP_LOGI("MAIN","0x06 - Driver IOIN read failed");
            TMConfError = true;
        }
    }

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
    tmc2209_end();
    ESP_LOGI("MAIN","UART driver removed");
}