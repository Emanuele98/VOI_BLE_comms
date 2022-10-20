/* 
	DHT22 temperature sensor driver
*/

#ifndef DHT22_H_  
#define DHT22_H_

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include <sys/time.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "peer.h"
#include "wifi.h"
#include "sd_card.h"

#define DHT_GPIO 			4	
#define DHT_OK 				0
#define DHT_CHECKSUM_ERROR 	-1
#define DHT_TIMEOUT_ERROR 	-2

time_t reconn_time;

/* Non-Volatile-Storage */
nvs_handle_t my_handle;
/* Waiting times after disconnection */
int64_t timePad[4];
int64_t timeScooter[4];


// == function prototypes =======================================

void 	setDHTgpio(int gpio);
void 	errorHandler(int response);
int 	readDHT();
float 	getHumidity();
float 	getTemperature();
int 	getSignalLevel( int usTimeOut, bool state );
void    CTU_ambient_temperature(void *arg);

#endif