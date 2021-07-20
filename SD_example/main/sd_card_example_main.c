/* SD card and FAT filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

    In some ESP32 bords is neccesary to burn one fuse if pin 12 is used.
    Execute this line in the terminal for doing this:
    
    espefuse.py set_flash_voltage 3.3V

*/

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

#include "sd_config.h"

static const char *TAG = "main";


void app_main(void)
{
    
    /*Config and mount sd card*/
    sd_set_config();

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening filex1");
    FILE* f = fopen(MOUNT_POINT"/hello.txt", "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    fprintf(f, "Hello world!\n");
    fclose(f);
    ESP_LOGI(TAG, "File writtenx1");

    
    ESP_LOGI(TAG, "Opening filex2");
    f = fopen(MOUNT_POINT"/hello.txt", "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    fprintf(f, "Hello world 2!\n");
    fclose(f);
    ESP_LOGI(TAG, "File writtenx2");

    /*Unmount SD card*/
    sd_unmount_card();       
}
