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

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nimble/ble.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/util/util.h"
#include "esp_bt.h"
#include <sys/time.h>

#include "aux_ctu_hw.h"

#define OR_TIME_GAP 30

//timers
TimerHandle_t dynamic_t_handle, alert_t_handle, connected_leds_handle, misaligned_leds_handle, charging_leds_handle;

#define N_BYTES_IN_UUID 16

/******************************************************************/

/**@brief   Macro for defining all characteristic payload instances */
#define WPT_DYNAMIC_PAYLOAD_DEF(_value1_name)                                                       \
wpt_dynamic_payload_t _value1_name;

#define WPT_STATIC_PAYLOAD_DEF(_value2_name)                                                        \
wpt_static_payload_t _value2_name;

#define WPT_ALERT_PAYLOAD_DEF(_value3_name)                                                         \
wpt_alert_payload_t _value3_name;                  

#define WPT_CONTROL_PAYLOAD_DEF(_value4_name)                                                       \
wpt_control_payload_t _value4_name; 

// XXX_CHAR_SIZE characteristic sizes.
#define PRU_CONTROL_CHAR_SIZE                                  5
#define PTU_STATIC_CHAR_SIZE                                   17
#define PRU_STATIC_CHAR_SIZE                                   6
#define PRU_DYNAMIC_CHAR_SIZE                                  18
#define ALERT_CHAR_SIZE                		                    1


/*******************************************************************/
/**                   LOCAL PTU CONFIGURATIONS                    **/
/*******************************************************************/

#define OVER_CURRENT                    2.75								   /**< Maximum current tolerated for I2C measurements. Flow will change in the future to handle such alert. */
#define OVER_VOLTAGE                    70 	    	                           /**< Maximum voltage tolerated for I2C measurements. Flow will change in the future to handle such alert. */
#define OVER_TEMPERATURE                55									   /**< Maximum temperature tolerated for I2C measurements. */
/*Voltage treshold during FULL-POWER mode */
#define VOLTAGE_FULL_THRESH 110
/* Voltage treshold for checking charging complete */
#define CURRENT_THRESH 0.1

bool alert, charge_comp;
time_t ch_comp, ale, now;
// connected variable
bool connected;

//i2c counter
uint8_t Temp_counter, Volt_counter, Curr_counter;


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
    union i2c         temp2;               /**< [optional] Temperature value from I2C (4 bytes). */
	uint8_t	          alert;		      /**< [mandatory] CRU alert field for warnings to send to CTU (1 byte). */
	uint8_t	          RFU;		          /**< [optional] Reserved for future use (1 byte). */
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


/**@brief Control characteristic structure. This contains elements necessary for control payload. */
typedef struct
{
	uint8_t           enable;              /**< [mandatory] Enable command for PTU (1 byte). */
	uint8_t           full_power;          /**< [mandatory] Full power mode or not (1 byte). */
	uint8_t           led;           	   /**< [optional] Set LED state */
	uint16_t          RFU;                 /**< [N/A] Undefined (1 byte). */
} wpt_control_payload_t;

/* Instanciation of characteristic values available for Airfuel. */
WPT_DYNAMIC_PAYLOAD_DEF(dyn_payload);
WPT_STATIC_PAYLOAD_DEF(static_payload);
WPT_ALERT_PAYLOAD_DEF(alert_payload);
WPT_CONTROL_PAYLOAD_DEF(control_payload);

//* BLE FUNCTION
int gatt_svr_init(void);

#endif
