/* SPI Master Half Duplex EEPROM example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "esp_log.h"

#include "spi_adxl355.h"


void app_main(void)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    /*Configurates SPI clock and SPI PINS*/
    adxl355_config_spi(); 
    vTaskDelay(100 / portTICK_PERIOD_MS);

    /*Configurates the accelerometer's sample rate*/
    adxl355_100hz_rate();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    /*Configurates and turns on the accelerometer (+-2G range)*/
    adxl355_range_conf();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    //se declara el buffer de recepcion de datos
    int medicion_procesada=0;
        
    //se declara el buffer de recepcion de datos
    uint8_t data_received[3];

    while (1)
    {   
        adxl355_read_accl(data_received,sizeof(data_received));

        //medicion_procesada=data_received[0]; //para 8 bits
        //medicion_procesada=(data_received[0]<<8)|(data_received[1]); //para 16 bits
        medicion_procesada=(data_received[0]<<12)|(data_received[1]<<4)|(data_received[2]>>4); //para 20 bits    

        //if (medicion_procesada > 127) medicion_procesada -= 127*2; //para 8 bits
        //if (medicion_procesada > 32768) medicion_procesada -= 32768*2; //para 16 bits
        if (medicion_procesada > 524287) medicion_procesada -= 524287*2; //para 20 bits

        float aceleracion=medicion_procesada/256; //para 20 bits
        printf("Medicion procesada: %d   Aceleracion: %f mg\n\n",medicion_procesada,aceleracion);
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
