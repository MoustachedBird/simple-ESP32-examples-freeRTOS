#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_types.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

/*
The ESP32 has two timer groups, each one with two general purpose hardware timers. 
All the timers are based on 64 bits counters and 16 bit prescalers.

The prescaler is used to divide the frequency of the base signal (usually 80 MHz = TIMER_BASE_CLK), 
which is then used to increment / decrement the timer counter. Since the prescaler has 16 bits, 
it can divide the clock signal frequency by a factor from 2 to 65536, giving a lot 
of configuration freedom.

The timer counters can be configured to count up or down and support automatic reload 
and software reload. They can also generate alarms when they reach a specific value, 
defined by the software. The value of the counter can be read by the software program.

For more information about hardware timers check: 
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/timer.html?highlight=hw_timer

For sotfware timers (not this example) check: 
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_timer.html
*/


//TIMER_BASE_CLK = 80 MHZ
#define TIMER_INTERVAL0_SEC   (1) // sample period interval for the timer 0 (1/400HZ)
#define TIMER_DIVIDER         8  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload


//creo el manejador para el semáforo como variable global
SemaphoreHandle_t interruption_semaphore = NULL;

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
void IRAM_ATTR timer_group0_isr(void *para)
{
    timer_spinlock_take(TIMER_GROUP_0);
    int timer_idx = (int) para;

    /* Clear the interrupt (with reload)*/
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    
   /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);
    
    /*Se termina el proceso de conteo de la interrupcion*/
    timer_spinlock_give(TIMER_GROUP_0);
    
    //Se activa una bandera para que la tarea que depende del semaforo se active
    xSemaphoreGiveFromISR(interruption_semaphore, NULL);
}

/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 */
static void conf_timer(int timer_idx,bool auto_reload, double timer_interval_sec){
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr,(void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_0, timer_idx);
}

/*
 * The main task of this example program
 */
static void example_task(void *arg)
{
    while (1) {
        if(xSemaphoreTake(interruption_semaphore,portMAX_DELAY) == pdTRUE) {
            printf("Timer event\n");
        }
    }
}

/*
 * In this example, we will test hardware timer0 and timer1 of timer group0.
 */
void app_main(void)
{
     // se crea el semáforo binario
    interruption_semaphore = xSemaphoreCreateBinary();
    conf_timer(TIMER_0, TEST_WITH_RELOAD, TIMER_INTERVAL0_SEC);
    xTaskCreate(example_task, "timer_evt_task", 2048, NULL, 5, NULL);
}
