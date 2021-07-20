/*Configurates SPI clock and SPI PINS*/
void adxl355_config_spi(void);

/*Configurates the accelerometer's sample rate*/
void adxl355_100hz_rate(void);

/*Configurates and turns on the accelerometer (+-2G range)*/
void adxl355_range_conf(void);

/*Read acceleration values*/
void adxl355_read_accl(uint8_t * data_received, uint8_t size_buffer);


#define READ_FLAG_ADXL355 0x01
#define WRITE_FLAG_ADXL355 0x00

//Range values
#define RANGE_2G  0x01
#define RANGE_4G  0x02
#define RANGE_8G  0x03

//Registers addresses
#define ZDATA3  (0x0E << 1)
#define YDATA3  (0x0B << 1)
#define XDATA3  (0x08 << 1)

/*
 This code tries to read ADXL355 accelerometer 
 */

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   32

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
