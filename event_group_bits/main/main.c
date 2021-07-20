#include <stdio.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "freertos/semphr.h"

#include "freertos/event_groups.h" //global flags
// Se definen los pines a usar por la interrupcion
#define ESP_INTR_FLAG_DEFAULT 0
#define PIN_PULSADOR 13


// Se definen los bits de los event group
#define CONMUTADOR      (1UL << 0UL) // conmuta cuando el boton es presionado
#define ESTADO_MEMORIA      (1UL << 1UL) // conmuta cuando el boton es presionado
#define ERROR_CONEXION      (1UL << 7UL) // conmuta cuando el boton es presionado


//se declara el event group (por default los event group permiten 8 bits)
EventGroupHandle_t flag_boton;
// |  B7 | B6 |  B5  |  B4  |  B3  |  B2   |  B1 |  B0 | 
                                                    //CONMUTADOR 

//variable tipo semaforo
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
        xSemaphoreTake(xSemaphore,portMAX_DELAY);
            printf("Pulsador presionado!\n");

            if ((xEventGroupGetBits(flag_boton)&CONMUTADOR)){
                
                // FLAG_BOTON AND 1UL << 0UL 
                //  FLAG_BOTON AND (1UL << 0UL) 
                //   FLAG_BOTON   AND   CONMUTADOR
                // X X X X X X X B0 AND 00000001 =? 1
                
                xEventGroupClearBits(flag_boton, CONMUTADOR); //pone en 0
                printf("Conmutador = 0\n");
            }else{
                xEventGroupSetBits(flag_boton, CONMUTADOR); //pone en 1
                printf("Conmutador = 1\n");
            }
   }
}

void app_main(void)
{
    //de configuran los pines de entrada y salida y la interrupcion
    init_GPIO();
    
    //Se crean los event group bits
    flag_boton= xEventGroupCreate();

    // se crea el semáforo binario
    xSemaphore = xSemaphoreCreateBinary();

    // creo la tarea task_pulsador
    xTaskCreate(task_pulsador, "task_pulsador", 2048, NULL, 5, NULL);

}
