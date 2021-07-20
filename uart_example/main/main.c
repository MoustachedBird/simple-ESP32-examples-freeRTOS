
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

static const int RX_BUF_SIZE = 1024;

//Se definen los pines para utilizar con la comunicacion de la UART
#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

void init(void) {

    //Guardamos la configuracion que queremos usar en con la UART
    //en una variable para cargarla posteriormente
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // Se instala el driver para utilizar la UART
    // en este caso la UART_NUM_1 (existen UART 0, 1 y 2)
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);

    //Se carga la configuracion
    uart_param_config(UART_NUM_1, &uart_config);

    //Se colocan los pines que se van a utilizar
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// funcion para enviar datos a través de UART
int sendData(const char* logName, const char* data)
{
    //se obtiene el largo del mensaje que se quiere enviar con la funcion strlen
    const int len = strlen(data);

    //Se envia el mensaje a través de la UART_NUM_1, esta funcion regresa el numero 
    //de bytes enviados si el envio fue correcto
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    
    //Se imprimen los bytes enviados en color verde
    ESP_LOGI(logName, "Bytes enviados: %d bytes", txBytes);
    
    //Se imprime el mensaje que se envió en color rojo
    ESP_LOGE(logName, "%s", data);
    return txBytes;
}


// TAREA: TX, para envio de mensajes a través de la UART
static void tx_task(void *arg)
{
    //se establece el nombre de la tarea para imprimirlo
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        //se envia el mensaje "HELLO WORLD" cada 2 segundos
        sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

//TAREA: RX, para recepcion de mensajes a través de la UART
static void rx_task(void *arg)
{
    //Se establece el nombre de la tarea
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    //Se declara una variable para guardar el mensaje de llegada (el buffer)
    //con tamaño de 1024 bytes
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        //se lee el puerto serie 
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        //en caso de recibir un mensaje hace...
        if (rxBytes > 0) {
            //se coloca 0 en el ultimo byte para eliminar el caracter de parada
            data[rxBytes] = 0;
            
            //Se imprime el mensaje en color verde
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            //Se imprime el mensaje en hexadecimal
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}

void app_main(void)
{
    //se ejecutan las configuraciones de inicio
    init();
    //se crean las tareas
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    //xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}
