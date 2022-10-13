#ifndef __SD_CARD_H__
#define __SD_CARD_H__

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"

#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

/* enable or disable the SD card*/
#if CONFIG_SD_CARD == 1 
    #define SD_CARD                1
#else   
    #define SD_CARD                0
#endif

//todo: add CD?
#define PIN_NUM_MISO    2
#define PIN_NUM_MOSI    15
#define PIN_NUM_CLK     14
#define PIN_NUM_CS      13

//FUNCTIONS
/**
 * @brief Install the sd card module 
 * 
 */
void install_sd_card(void);

/**
 * @brief Write into the sd card
 * 
 */
void write_sd_card(char *topic, float value, struct tm* time);

/**
 * @brief Read a line into the sd card
 * 
 */
void read_sd_card(char *topic);

/**
 * @brief Delete completely the SD card content
 * 
 */
void delete_sd_card(void);


#endif