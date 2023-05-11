#ifndef BLE_WPT_H__
#define BLE_WPT_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

//watchdog
#include "esp_task_wdt.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"


#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nimble/ble.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/util/util.h"



#define N_BYTES_IN_UUID 16
#define COMPANY_ID                      0x00FF                                 /**< Airfuel company id. Is passed in advertising data. */

/******************************************************************/

/**@brief   Macro for defining all characteristic payload instances */
#define WPT_DYNAMIC_PAYLOAD_DEF(_value1_name)                                                       \
wpt_dynamic_payload_t _value1_name;

#define WPT_STATIC_PAYLOAD_DEF(_value2_name)                                                        \
wpt_static_payload_t _value2_name;

#define WPT_ALERT_PAYLOAD_DEF(_value3_name)                                                         \
wpt_alert_payload_t _value3_name;                  

/* WPT SERVICE UUID */
#define WPT_SERVICE_UUID                                       0xFFFE

#define WPT_CHARGING_PTU_STATIC_UUID                           0xE671
#define WPT_CHARGING_alert_UUID                                0xE672
#define WPT_CHARGING_PRU_STATIC_UUID                           0xE673
#define WPT_CHARGING_PRU_DYNAMIC_UUID                          0xE674

// XXX_CHAR_SIZE characteristic sizes.
#define PTU_STATIC_CHAR_SIZE                                   17
#define PRU_STATIC_CHAR_SIZE                                   6
#define PRU_DYNAMIC_CHAR_SIZE                                  18
#define ALERT_CHAR_SIZE                                         1


/*******************************************************************/
/**                   LOCAL PRU CONFIGURATIONS                    **/
/*******************************************************************/
//LIMITS BEFORE SENDING ALERTS
#define OVER_CURRENT                    2								 	   /**< Maximum current tolerated */
#define OVER_VOLTAGE                    80 //check under 50 	               /**< Maximum voltage tolerated */
#define OVER_TEMPERATURE                50									   /**< Maximum temperature tolerated */
/*Voltage treshold during FULL-POWER mode */
#define VOLTAGE_FULL_THRESH 110
/* Voltage treshold for checking charging complete */
#define CURRENT_THRESH 0.15


union i2c{
	float f;
	uint8_t b[4];
};

/** @brief Dynamic characteristic structure. This contains elements necessary for dynamic payload. */
typedef struct
{
    union i2c         vrect;              /**< [mandatory] Vrect value from I2C (4 bytes). */
    union i2c         irect;              /**< [mandatory] Irect value from I2C (4 bytes). */
    union i2c         temp1;               /**< [optional] Temperature value from I2C (4 bytes). */
    union i2c         temp2;               /**< [optional] Temperature value from I2C (4 bytes). NOT CURRENTLY USED */
	uint8_t	          alert;  		      /**< [mandatory] CRU alert field for warnings to send to CTU (1 byte). */
	uint8_t	          RFU; 		          /**< [optional] Reserved for future use (1 byte). */
} wpt_dynamic_payload_t;

/** @brief Dynamic characteristic structure. This contains elements necessary for static payload. */
typedef struct
{
    uint8_t           ble_addr0;    /**< [mandatory] address field 0 (1 byte). */
    uint8_t           ble_addr1;    /**< [mandatory] address field 1 (1 byte). */
    uint8_t           ble_addr2;    /**< [mandatory] address field 2 (1 byte). */
    uint8_t           ble_addr3;    /**< [mandatory] address field 3 (1 byte). */
    uint8_t           ble_addr4;    /**< [mandatory] address field 4 (1 byte). */
    uint8_t           ble_addr5;    /**< [mandatory] address field 5 (1 byte). */
} wpt_static_payload_t;

/**@brief Alert characteristic structure. This contains elements necessary for alert payload. */
typedef union
{
	struct {
		uint8_t           charge_complete:1;    /**< [mandatory] Defines which optional fields are populated (1 byte). */
		uint8_t           overtemperature:1;    /**< [mandatory] Defines which optional fields are populated (1 byte). */
		uint8_t           overcurrent:1;        /**< [mandatory] Defines which optional fields are populated (1 byte). */
		uint8_t           overvoltage:1;        /**< [mandatory] Defines which optional fields are populated (1 byte). */
		uint8_t           FOD:1;        /**< [mandatory] Defines which optional fields are populated (1 byte). */
	};
	uint8_t internal;
} alert_field_t;

typedef struct
{
	alert_field_t     alert_field;
} wpt_alert_payload_t;

/* Instanciation of characteristic values available for Airfuel. */
WPT_DYNAMIC_PAYLOAD_DEF(dyn_payload);
WPT_STATIC_PAYLOAD_DEF(static_payload);
WPT_ALERT_PAYLOAD_DEF(alert_payload);


//* BLE FUNCTIONS
int gatt_svr_init(void);

#endif
