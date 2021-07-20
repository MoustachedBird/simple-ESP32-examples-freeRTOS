/* SPIFFS filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"


void app_main(void)
{
    printf("Initializing SPIFFS \n");
    
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs", /*File path prefix associated with the filesystem.*/
      .partition_label = NULL, /*Optional, label of SPIFFS partition to use. If set to NULL, 
      first partition with subtype=spiffs will be used.*/
      .max_files = 5,  //Maximum files that could be open at the same time.
      .format_if_mount_failed = false //If true, it will format the file system if it fails to mount.
    };
    
    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf); //Register and mount SPIFFS to VFS with given path prefix.

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            printf("Failed to mount or format filesystem\n");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            printf("Failed to find SPIFFS partition\n");
        } else {
            printf("Failed to initialize SPIFFS (%s)\n", esp_err_to_name(ret));
        }
        return;
    }
    
    size_t total = 0, used = 0;
    /*
    conf.partition_label: Optional, label of the partition to get info for. 
    If not specified, first partition with subtype=spiffs is used. 
    
    total: Size of the file system

    used: Current used bytes in the file system
    */
    ret = esp_spiffs_info(conf.partition_label, &total, &used); 
    if (ret != ESP_OK) {
        printf("Failed to get SPIFFS partition information (%s)\n", esp_err_to_name(ret));
    } else {
        printf("Partition size: total: %d bytes, used: %d bytes\n", total, used);
    }

    

    // Use POSIX and C standard library functions to work with files.
    // Open the file for reading
    printf("Reading file\n");
    FILE* f = fopen("/spiffs/hello_world.txt", "r");
    if (f == NULL) {
        printf("Failed to open file for reading\n");
        return;
    }
    
    // Store reading information
    char buf[64];
    memset(buf, 0, sizeof(buf)); //clean the buffer, fill with zeros
    fread(buf, 1, sizeof(buf), f); //read the file an store info in buf
    fclose(f); //close the file

    // Print the information of the file
    printf("Information of the file:\n '%s'\n", buf);

    // All done, unmount partition and disable SPIFFS
    esp_vfs_spiffs_unregister(conf.partition_label);
    printf("SPIFFS unmounted\n");
}