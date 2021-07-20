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
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "esp_log.h"

#include "spi_mcp356x.h"

/*
PINES SPI

PIN      SPI2     SPI3
CS0       15        5
SCLK      14       18
MISO      12       19
MOSI      13       23
QUADWP    2        22
QUADHD    4        21

Only the first Device attached to the bus can use the CS0 pin.

Si el pin CS esta en 0 se activa el dispositivo esclavo
*/


void app_main(void)
{
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);      
    mcp356x_config_spi();
    
    vTaskDelay(100 / portTICK_PERIOD_MS);      
    mcp356x_reg_config();
    int medicion_procesada = 0;
    
    //se declara el buffer de recepcion de datos
    uint8_t data_received[3];

    while (1)
    {   
        mcp356x_read_adc(data_received,sizeof(data_received));    
        
        /*
        sensibilidad del SM-24 = 28.8 V/m/s = 0.288 V/cm/s

        para calcular velocidad = (medicion_procesada*1.65)/(8388607*0.288*amplificacion);
        */

       //medicion_procesada=data_received[0]; //para 8 bits
        //medicion_procesada=(data_received[0]<<8)|(data_received[1]); //para 16 bits
        //medicion_procesada=(data_received[0]<<12)|(data_received[1]<<4)|(data_received[2]>>4); //para 20 bits
        //medicion_procesada=(data_received[0]<<14)|(data_received[1]<<6)|(data_received[2]>>2); //para 22 bits
        medicion_procesada=(data_received[0]<<16)|(data_received[1]<<8)|(data_received[2]); //para 24 bits
        

        //if (medicion_procesada > 127) medicion_procesada -= 127*2; //para 8 bits
        //if (medicion_procesada > 32768) medicion_procesada -= 32768*2; //para 16 bits
        //if (medicion_procesada > 524287) medicion_procesada -= 524287*2; //para 20 bits
        //if (medicion_procesada > 2097151) medicion_procesada -= 2097151*2; //para 22 bits
        if (medicion_procesada > 8388607) medicion_procesada -= 8388607*2; //para 24 bits

        //printf("ADC[0]:%d ADC[1]:%d ADC[2]:%d \n",data_received[0],data_received[1],data_received[2]);
        printf("%d\n",medicion_procesada);        
        vTaskDelay(10 / portTICK_PERIOD_MS);       

    }
}