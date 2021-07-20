#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define NUCLEO_0 0
#define NUCLEO_1 1


void task1(void* arg) {

   while(1) {
        printf("Hola desde el nucleo %d",xPortGetCoreID());
        vTaskDelay(1500 / portTICK_PERIOD_MS);
   }
}

void task2(void* arg) {

   while(1) {
        printf("Hola desde el nucleo %d",xPortGetCoreID());
        vTaskDelay(3000 / portTICK_PERIOD_MS);
   }
}

void app_main(void)
{
    xTaskCreatePinnedToCore(task1, "task1", 2048, NULL, 10, NULL, NUCLEO_0);
    xTaskCreatePinnedToCore(task2, "task2", 2048, NULL, 10, NULL, NUCLEO_1);
}
