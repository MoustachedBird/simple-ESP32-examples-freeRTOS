#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_types.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

/*
EDITED by MoustachedBird


***** IMPORTANT: Task notify is better than a semaphore?*****
Each RTOS task has a 32-bit notification value. An RTOS task notification is an event sent 
directly to a task that can unblock the receiving task, and optionally update the receiving 
task’s notification value.

Task notifications can update the receiving task’s notification value in the following ways:

    Set the receiving task’s notification value without overwriting a previous value
    Overwrite the receiving task’s notification value
    Set one or more bits in the receiving task’s notification value
    Increment the receiving task’s notification value

That flexibility allows task notifications to be used where previously it 
would have been necessary to create a separate queue, binary semaphore, counting semaphore or 
event group. Unblocking an RTOS task with a direct notification is 45% faster * and uses less 
RAM than unblocking a task with a binary semaphore.

More information about task notifications: https://www.freertos.org/RTOS-task-notifications.html

The ESP32 has two timer groups, each one with two general purpose hardware timers. 
All the timers are based on 64 bits counters and 16 bit prescalers.


**HARDWARE TIMER INFORMATION**

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


/* TIMER_BASE_CLK = 80 MHZ */ 
#define TIMER_PERIOD   (0.5) // sample period interval for the timer 0 (1/TIMER_PERIOD = Timer frequency in HZ)

#define TIMER_SENSOR TIMER_0 //Timer for counting 
#define TIMER_DIVIDER         8  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload


//Task ID for calling the task from the interrupt vector (timer_isr)
TaskHandle_t example_taskID; 


/*
 * Timer ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
void IRAM_ATTR timer_isr()
{
    /*Start the interrupt and lock timer_group 0 (i guess)*/
    timer_spinlock_take(TIMER_GROUP_0);

    /* Clear the interrupt (with reload)*/
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    
   /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_SENSOR);
    
    /* Notify example_task that the buffer is full which means that example_task gets activated every
    time the isr vector is called
    */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(example_taskID, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }

    /*Finish the interrupt and unlock timer_group 0 (i guess)*/
    timer_spinlock_give(TIMER_GROUP_0);
}

/*
 * Initialize selected timer of the timer group 0
 *
 * TIMER_SENSOR - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * TIMER_PERIOD - the interval of alarm to set
 */
static void conf_timer(){
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TEST_WITH_RELOAD,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, TIMER_SENSOR, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_SENSOR, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_SENSOR, TIMER_PERIOD * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_SENSOR);
    timer_isr_register(TIMER_GROUP_0, TIMER_SENSOR, timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_0, TIMER_SENSOR);
}

/*
 * The main task of this example program
 */
static void example_task(void *arg)
{
    while (1) {
        // Sleep until the ISR gives us something to do, if nothing is recieved then waits forever
        uint32_t tcount = ulTaskNotifyTake(pdFALSE, portMAX_DELAY); 
        // if ISR give us something to do then print the following message 
        printf("Timer task!!! %.2f seconds has passed\n",TIMER_PERIOD);
    }
}

/*
 * In this example, we will test hardware timer0 of timer group0.
 */
void app_main(void)
{
    conf_timer(); //configurate the timer 0 for this example
    xTaskCreate(example_task, "example_task", 2048, NULL, 5, &example_taskID); //create the example task
}









