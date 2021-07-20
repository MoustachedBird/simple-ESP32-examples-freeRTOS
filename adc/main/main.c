/* Lee un valor del adc y muestra por el puerto serie
*/
//Incluimos los archivos de cabecera necesarios para el ejemplo
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "driver/gpio.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

#define TAM_COLA_LECTURA 1 /*1 mensajes*/
#define TAM_MSG_LECTURA 2 /*Cada Mensaje 1 Entero (2 bytes)*/

//creamos como variables globales los manejadores de las dos colas que utilizaremos para enviar datos entre tareas
xQueueHandle cola_Lectura;

//tarea para leer el ACD1
void adctask(void* arg)
{
    //Para el adc 1
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0);
    
    //Para el adc 2
    //adc2_config_channel_atten(ADC2_CHANNEL_0, ADC_ATTEN_DB_0);
    
    int valor;
    int16_t valor2;
    while(1){
        valor=adc1_get_raw(ADC1_CHANNEL_6);
        
        //adc2_get_raw(ADC2_CHANNEL_0, ADC_WIDTH_BIT_12, &valor);
        

        valor2 = valor;
        xQueueSendToBack(cola_Lectura, &valor2,portMAX_DELAY);
        printf("valor ADC antes: %d\n",valor);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

//Tarea que recibe los datos de tipo entero del ACD1 los convierte a un tipo float y luego a una cadena de caracteres.
void convierteDatos(void *pvParameter)
{
   uint16_t dato;
   float Voltaje=0.00;
    while(1){
        xQueueReceive(cola_Lectura,&dato,portMAX_DELAY);
        //1s --> Tiempo max. que la tarea está bloqueada si la cola está vacía
        Voltaje= (3.3*dato)/4095;
        printf("valor ADC despues: %d\n",dato);
        printf("%1.2f volts\n",Voltaje);
        printf("===============\n"); 
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


void app_main()
{
    //Creamos las dos colas que utilizaremos en el programa para comunicar datos entre las tareas
    cola_Lectura= xQueueCreate(TAM_COLA_LECTURA, TAM_MSG_LECTURA);

   //Creamos las tareas del programa
    xTaskCreate(adctask, "adctask", 1024*10, NULL, 5, NULL);
    xTaskCreate(convierteDatos, "convierteDatos", 1024*10, NULL, 10, NULL);
}
