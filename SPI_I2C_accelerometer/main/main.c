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
#include "driver/i2c.h" //Headers for I2C
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "esp_log.h"


/*
I2C PINS
*/
#define PIN_SCL GPIO_NUM_22 // SCL = PIN 22
#define PIN_SDA GPIO_NUM_21 // SDA = PIN 21

#define MMA8452Q_ADDRESS 0x1C // MMA8452Q I2C address is 0x1C(28)
#define MMA8452Q_ACCEL_XOUT_H 0x01  //Address of the first X acceleration data register
#define MMA8452Q_ACCEL_YOUT_H 0x03  //Address of the first Y acceleration data register
#define MMA8452Q_ACCEL_ZOUT_H 0x05  //Address of the first Z acceleration data register

#define XYZ_DATA_CFG 0x0E //Register for configurating sensivity range
#define CTRL_REG1 0x2A //Control register for selecting the operation mode


/*
 SPI PINS
 */

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   32


static const char TAG[] = "main";

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

#define READ_FLAG_ADXL355 0x01
#define WRITE_FLAG_ADXL355 0x00

//Device values
#define RANGE_2G  0x01
#define RANGE_4G  0x02
#define RANGE_8G  0x03


void task_i2c(void* arg) {

    //I2C configuration
	printf("Drive configuration \n");
	i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = PIN_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };


    i2c_param_config(I2C_NUM_0, &conf);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    printf("SENSOR: i2c driver installed  \n");	
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	//vTaskDelay(200/portTICK_PERIOD_MS);
	
	//-----------------------[ Select Active Mode (Accelerometer configuration) ]-----------------------------------------------
	printf("SENSOR: i2c cmd link created\n");	
    /*Start I2C communication as MASTER*/
    i2c_master_start(cmd);
    /*Select the I2C device, in this case we write mma8452Q adress to select it
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, ACK_EN);
    ACK= acknowledge bit.*/
    i2c_master_write_byte(cmd, (MMA8452Q_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
    /*Select control register*/
    i2c_master_write_byte(cmd, CTRL_REG1, true);
    /* Select active mode (sensor sends data continuously)*/
    i2c_master_write_byte(cmd, 0B00000001, true);
    
	//-----------------------[  Set range to +/- 2g (Accelerometer configuration)]--------------------------------------------
	printf("SENSOR: i2c sensor initialized\n");	
	/* Start I2C communication as MASTER*/
    i2c_master_start(cmd);
    /* Select the I2C device, in this case we write mma8452Q adress to select it*/
    i2c_master_write_byte(cmd, (MMA8452Q_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    /* XYZ_DATA_CFG register*/
    i2c_master_write_byte(cmd, XYZ_DATA_CFG, true);
    /* Set range to +/- 2g*/
    i2c_master_write_byte(cmd, 0B00000000, true);
    //--------------------------------------------------------------------------------------------
 
	/* Stop I2C communication*/
    i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

    uint8_t datos_recibidos[2]; // 8 bit data to save 12 bits per channel of the accelerometer
    int16_t medicion_procesada=0; //variable para almacenar 

    printf("SENSOR: i2c configuration finish\n");
    float aceleracion=0;
    while (1){

        //se crea el link de la comunicacion 
        cmd = i2c_cmd_link_create();

        i2c_master_start(cmd);
        //Se envia la direccion del dispositivo 
        i2c_master_write_byte(cmd, (MMA8452Q_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        //el registro a partir del cual se empezar?? a leer
        i2c_master_write_byte(cmd, MMA8452Q_ACCEL_ZOUT_H, true);
        
        i2c_master_start(cmd);
        //se lee a partir de la direccion anterior en el mismo dispositivo
        i2c_master_write_byte(cmd, (MMA8452Q_ADDRESS << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, datos_recibidos,sizeof(datos_recibidos), false); //read 3 axis of the accelerometer

        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        //para 12 bits     
        //medicion_procesada = ((datos_recibidos[0] << 4) | (datos_recibidos[1]) >> 4);
        //if (medicion_procesada > 2047) medicion_procesada -= 2*2047;  
        
        //para 14 bits
        medicion_procesada = ((datos_recibidos[0] << 6) | (datos_recibidos[1]) >> 2);
        if (medicion_procesada > 8191) medicion_procesada -= 2*8191;  
        

        aceleracion=(medicion_procesada*1000)/4096; //para sacar el valor de aceleracion en mg

        printf("I2C: bits: %d Accl: %.2f mg",medicion_procesada,aceleracion);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}


void task_spi(void* arg) {
    
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing bus SPI%d...", SPI2_HOST+1);
    
    //manejador de la comunicacion SPI
    spi_device_handle_t spi;

    //configuraciones del bus SPI
    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,  //-1 means not used
        .quadhd_io_num = -1,  // not used
        .max_transfer_sz = 32, //maximum 4094 minimum 0 in bytes
    };

    /*
    mode0 means CPOL=0, CPHA=0.
    When SPI is idle, the clock output is logic low; 
    data changes on the falling edge of the SPI clock 
    and is sampled on the rising edge;

    mode1 means CPOL=0, CPHA=1.
    When SPI is idle, the clock output is logic low; 
    data changes on the rising edge of the SPI clock 
    and is sampled on the falling edge;

    mode2 means when CPOL=1, CPHA=0.
    When SPI is idle, the clock output is logic high; 
    data changes on the rising edge of the SPI clock and 
    is sampled on the falling edge;

    mode3 means when CPOL=1, CPHA=1.
    When SPI is idle, the clock output is logic high; 
    data changes on the falling edge of the SPI clock and is 
    sampled on the rising edge.
    
    */

    //configuraciones del dispositivo 
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=1*1000*1000,           //Clock out at 10 MHz, overclock = 26 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .address_bits=8, //El largo de la direccion de memoria del dispositivo
        //.pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    
    /*
      Initialize the SPI bus
      se selecciona el bus SPI a utilizar, 
      se caga la configuracion del bus SPI, 
      dma_chan es un tipo de comunicacion derivada del SPI, en caso de necesitarse
      este valor es igual a 0 
    */ 
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, 0);
    ESP_ERROR_CHECK(ret);

    //Se a??ade el dispositivo
    ret=spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);


    // =====================================================================
    // =========== [Para ENV??O de informacion por el bus SPI] ==============
    // =====================================================================
    
    //-=-=-=-=-=-=-=-=-=[ Cambia el rango de medicion ]-=-=-=-=-=-=-=-=-=-=-=-=
    
    //se declara la variable de transaccion
    spi_transaction_t transaccion;
    //se resetea el manejador de transaccion
    memset(&transaccion, 0, sizeof(transaccion));       //Se limpia con 0 la variable para evitar errores

    uint8_t datos_enviar=RANGE_2G; //segun la hoja de datos poner un 01 en 
    //los bits b0 y b1 establece el ranglo de +-2g en el registro 0x2C

    transaccion.length=8;      //Largo de los datos en bits.
    transaccion.tx_buffer=&datos_enviar;   //Puntero de datos a enviar
    transaccion.addr=(0x2C<<1)|WRITE_FLAG_ADXL355; /*Direccion (address) a la  
    cual enviar el dato, el largo de la address se establece en 
    spi_device_interface_config_t con el parametro address_bits
    */

    //se transmiten los datos
    ret=spi_device_transmit(spi, &transaccion); 
    assert(ret==ESP_OK);            //Se verifica si no hubo errores
    //printf("Direccion: 010 1100   RW= 0   Datos: 0000 0001 \n");

    vTaskDelay(50 / portTICK_PERIOD_MS); //se espera un tiempo para estabilizar el sensor

    //-=-=-=-=-=-=-=-=-=[ Cambia la frecuencia de muestreo ]-=-=-=-=-=-=-=-=-=-=-=-=
    
    /*La frecuencia de muestreo incluye un filtro pasa bajas
    La relacion de la frecuencia de corte del filtro es 

    frecuencia_corte = frecuencia_muestreo/4
    
    Registro: 0x28

    Dato      Frecuencia muestreo  Filtro pasa bajas (Hz)  
    0x00            4000                 1000
    0x01            2000                 500
    0x02            1000                 250
    0x03            500                  125
    0x04            250                  62.5
    0x05            125                  31.25
    0x06            62.5                 15.625 
    0x07            31.25                7.813
    0x08            15.625               3.906
    0x09            7.813                1.953
    0x0A            3.906                0.977
    */

    memset(&transaccion, 0, sizeof(transaccion));       //Se limpia con 0 la variable para evitar errores
    datos_enviar=0x05; //Frecuencia de muestreo de 1000 Hz

    //se escribe el dato 1 en la direccion 0x28, el registro de filtros
    transaccion.length=8;      //total de datos 8 bits.
    transaccion.tx_buffer=&datos_enviar;   //Puntero de datos a enviar
    transaccion.addr=(0x28<<1)|WRITE_FLAG_ADXL355; //Registro para activar el modo medicion

    ret=spi_device_transmit(spi, &transaccion);  //Transmite los datos
    assert(ret==ESP_OK);            //Should have had no issues.
    //printf("Preparando frecuencia de muestreo \n");

    vTaskDelay(50 / portTICK_PERIOD_MS); //se espera un tiempo para estabilizar el sensor



    //-=-=-=-=-=-=-=-=-=[ Cambia a modo medicion ]-=-=-=-=-=-=-=-=-=-=-=-=
        
    memset(&transaccion, 0, sizeof(transaccion));       //Se limpia con 0 la variable para evitar errores
    datos_enviar=0x06; //solo acelerometros, se desabilita el sensor de temperatura
    //se cambia a modo medicion, se quita modo standby

    //se escribe el dato 1 en la direccion 0x2C para colocar el rango de +-2g
    transaccion.length=8;      //Len is in bytes, transaction length is in bits.
    transaccion.tx_buffer=&datos_enviar;   //Puntero de datos a enviar
    transaccion.addr=(0x2D<<1)|WRITE_FLAG_ADXL355; //Registro para activar el modo medicion

    ret=spi_device_transmit(spi, &transaccion);  //Transmite los datos
    assert(ret==ESP_OK);            //Should have had no issues.
    //printf("Direccion: 010 1101   RW= 0   Datos: 0000 0110 \n");

    vTaskDelay(50 / portTICK_PERIOD_MS); //se espera un tiempo para estabilizar el sensor

    //int menor=100000000;
    //int mayor=-100000000;
    
    printf("SENSOR: SPI configuration finish\n");

    //se declara el buffer de recepcion de datos
    int32_t medicion_procesada=0;
    uint8_t datos_recibidos[3];
    float aceleracion=0;

    while (1){

        // =====================================================================
        // =========== [Para RECIBIR de informacion por el bus SPI] ==============
        // =====================================================================
        
        //se resetea el manejador de transaccion
        memset(&transaccion, 0, sizeof(transaccion));
        //Se limpia con 0 la variable para evitar errores

        transaccion.length=8*sizeof(datos_recibidos);      //tama??o de transaccion en bits
        transaccion.rx_buffer= &datos_recibidos;  //Puntero del buffer para recibir los datos
        transaccion.addr=(0x0E<<1)|READ_FLAG_ADXL355 ; //Leer el registro del eje Z byte m??s significativo

        ret=spi_device_transmit(spi, &transaccion);  //Transmitir
        assert(ret==ESP_OK);            //Should have had no issues.

        //printf("Direccion: 000 1110   RW= 1   Datos_recibidos: %d  %d   %d \n",datos_recibidos[0],datos_recibidos[1],datos_recibidos[2]);
        
        //medicion_procesada=datos_recibidos[0]; //para 8 bits
        //medicion_procesada=(datos_recibidos[0]<<8)|(datos_recibidos[1]); //para 16 bits
        medicion_procesada=(datos_recibidos[0]<<12)|(datos_recibidos[1]<<4)|(datos_recibidos[2]>>4); //para 20 bits
        

        //if (medicion_procesada > 127) medicion_procesada -= 127*2; //para 8 bits
        //if (medicion_procesada > 32768) medicion_procesada -= 32768*2; //para 16 bits
        if (medicion_procesada > 524287) medicion_procesada -= 524287*2; //para 20 bits
    
        //float aceleracion=(medicion_procesada*4096)/256; //para 8 bits
        //float aceleracion=(medicion_procesada*16)/256; //para 16 bits
        aceleracion=medicion_procesada/256; //para 20 bits
        
        //Para medir el ruido pico a pico        
        //if(medicion_procesada<menor) menor=medicion_procesada;
        //if(medicion_procesada>mayor) mayor=medicion_procesada;
        //float ruido= (mayor-menor);

        //printf("     SPI: %d\n",medicion_procesada);
        //printf("Ruido: %f mg\n",ruido/256);

        printf("     SPI:  bits: %d   Accl: %.2f mg\n",medicion_procesada,aceleracion);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    xTaskCreate(task_i2c, "task_i2c", 2048, NULL, 10, NULL);
    xTaskCreate(task_spi, "task_spi", 2048, NULL, 10, NULL);
}
