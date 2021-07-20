

#define MOUNT_POINT "/sdcard"

//      SD pin  | ESP32 gpio number

#define SD_PIN_9       12
#define SD_PIN_1       13
#define SD_PIN_2       15
// SD_PIN3 = GND
// SD_PIN4 = 3.3 V
#define SD_PIN_5       14
// SD_PIN6 = GND
#define SD_PIN_7       2
#define SD_PIN_8       4

    /* GPIOs LIST:

        =============[SLOT 0]=============
        ESP32  PULL-UP     SD-CARD or Adapter

        9      +10K        9 (DAT2)
        10     +10K        1 (CD/DAT3)  
        11     +10K        2 (CMD)
        GND                3 (VSS)
        3.3v               4 (VDD)
        6      +10K*       5 (CLK)
        GND                6 (VSS)
        7      +10K        7 (DAT0)
        8      +10K        8 (DAT1)


        =============[SLOT 1]=============
        ESP32  PULL-UP     SD-CARD or Adapter

        12     +10K        9 (DAT2)
        13     +10K        1 (CD/DAT3)  
        15     +10K        2 (CMD)
        GND                3 (VSS)
        3.3v               4 (VDD)
        14     +10K*       5 (CLK)
        GND                6 (VSS)
        2      +10K        7 (DAT0)
        4      +10K        8 (DAT1)

    */

void sd_unmount_card(void);

void sd_set_config(void);