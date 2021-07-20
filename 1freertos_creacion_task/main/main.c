/* Ejemplo básico con tareas

   En este ejemplo se puede apreciar como se accede a la tareas de acuerdo a su prioridad


*/


/*Librerias basicas*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "nvs_flash.h"

void tarea1(void *pvParameter)
{
    while(1) {
        printf("Ejecutando tarea 1\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
    vTaskDelete(NULL);
}

void tarea2(void *pvParameter)
{
    while(1) {
        printf("Ejecutando tarea 2\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
    vTaskDelete(NULL);
}

void tarea3(void *pvParameter)
{
    while(1) {
        printf("Ejecutando tarea 3\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
    vTaskDelete(NULL);
}

void tarea4(void *pvParameter)
{
    while(1) {
        printf("Ejecutando tarea 4\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
    vTaskDelete(NULL);
}



void app_main()
{
    /*
    La biblioteca de almacenamiento en memoria no volátil (NVS) está diseñada 
    para almacenar pares clave-valor en la memeoria flash.

    La funcion nvs_flash_init() inicializa la partición NVS predeterminada.
    La partición NVS predeterminada es la que está etiquetada como "nvs" en la tabla de particiones.
 
    Para más información de la memoria NVS consultar:
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/nvs_flash.html?highlight=nvs_flash_init#_CPPv414nvs_flash_initv
    
    Para consultar tablas de particiones consultar:
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html
    */
    
    nvs_flash_init();
   
    /*  Estructura:
    xTaskCreate (Direccion de memoria de la tarea, Nombre de la tarea, 
                 Cantidad de memoria asignada a la tarea,
                 Parametros que ingresan a la tarea en este caso ninguno por eso NULL
                 Prioridad que se le da a la tarea entre mayor el numero se ejecuta primero
                 Parametros que devuelve la tarea en este caso ninguno por eso NULL)
    
    
    Para mas informacion consultar: 
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html?highlight=xtaskcreate#_CPPv411xTaskCreate14TaskFunction_tPCKcK8uint32_tPCv11UBaseType_tPC12TaskHandle_t
    
    */
    xTaskCreate(&tarea1, "tarea1", 1024, NULL, 1, NULL);
    xTaskCreate(&tarea2, "tarea2", 1024, NULL, 2, NULL);
    xTaskCreate(&tarea3, "tarea3", 1024, NULL, 3, NULL);
    xTaskCreate(&tarea4, "tarea4", 1024, NULL, 4, NULL);


    //vTaskStartScheduler();
}
