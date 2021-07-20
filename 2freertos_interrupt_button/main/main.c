#include <stdio.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "freertos/semphr.h"


#define ESP_INTR_FLAG_DEFAULT 0
#define PIN_PULSADOR 13

//creo el manejador para el semáforo como variable global
SemaphoreHandle_t xSemaphore = NULL;

// Rutina de interrupción, llamada cuando se presiona el pulsador
void IRAM_ATTR pulsador_isr_handler(void* arg) {

    // da el semáforo para que quede libre para la tarea pulsador
   xSemaphoreGiveFromISR(xSemaphore, NULL);
}

void init_GPIO(){
   //configuro el PIN_PULSADOR como un pin GPIO
   gpio_pad_select_gpio(PIN_PULSADOR);
   // seleciono el PIN_PULSADOR como pin de entrada
   gpio_set_direction(PIN_PULSADOR, GPIO_MODE_INPUT);
   // instala el servicio ISR con la configuración por defecto.
   gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
   // añado el manejador para el servicio ISR
   gpio_isr_handler_add(PIN_PULSADOR, pulsador_isr_handler, NULL);
   // habilito interrupción por flanco descendente (1->0)
   gpio_set_intr_type(PIN_PULSADOR, GPIO_INTR_NEGEDGE);

}

void task_pulsador(void* arg) {

   while(1) {

      // Espero por la notificación de la ISR
      if(xSemaphoreTake(xSemaphore,portMAX_DELAY) == pdTRUE) {
         printf("Pulsador presionado!\n");

      }
   }
}

void app_main()
{
   //llamo a la función init_GPIO()
   init_GPIO();

   // se crea el semáforo binario
   xSemaphore = xSemaphoreCreateBinary();

   // creo la tarea task_pulsador
   xTaskCreate(task_pulsador, "task_pulsador", 2048, NULL, 5, NULL);

}
