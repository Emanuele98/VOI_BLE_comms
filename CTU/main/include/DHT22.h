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
#include "wifi.h"
#include "sd_card.h"

#define DHT_GPIO 			4	
#define DHT_OK 				0
#define DHT_CHECKSUM_ERROR 	-1
#define DHT_TIMEOUT_ERROR 	-2

// == function prototypes =======================================

void 	setDHTgpio(int gpio);
void 	errorHandler(int response);
int 	readDHT();
float 	getHumidity();
float 	getTemperature();
int 	getSignalLevel( int usTimeOut, bool state );
void    CTU_ambient_temperature(void *arg);

#endif