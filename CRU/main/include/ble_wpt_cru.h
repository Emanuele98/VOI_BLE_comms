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


//timers
TimerHandle_t dynamic_t_handle, alert_t_handle;

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

#define WPT_PTU_STATIC_PAYLOAD_DEF(_value4_name)                                                    \
wpt_ptu_static_payload_t _value4_name;


/* WPT SERVICE UUID */
#define WPT_SERVICE_UUID                                       0xFFFE

#define WPT_CHARGING_PTU_STATIC_UUID                           0xE671
#define WPT_CHARGING_alert_UUID                            0xE672
#define WPT_CHARGING_PRU_STATIC_UUID                           0xE673
#define WPT_CHARGING_PRU_DYNAMIC_UUID                          0xE674

// XXX_CHAR_SIZE characteristic sizes.
#define PTU_STATIC_CHAR_SIZE                                   17
#define PRU_STATIC_CHAR_SIZE                                   20
#define PRU_DYNAMIC_CHAR_SIZE                                  20
#define alert_CHAR_SIZE                                    6


/*******************************************************************/
/**                   LOCAL PRU CONFIGURATIONS                    **/
/*******************************************************************/

#define OPTIONAL_FIELDS_STAT            0x00                                   /**< Optional fields requiered. */
#define PROTOCOL_REVISION               0x00                                   /**< Protocol revision of PRU. */
#define PRU_CATEGORY                    0x00                                   /**< Category of PRU. */
#define PRU_INFORMATION                 0x00                                   /**< Information relative to PRU setup. */
#define PRU_HARD_REVISION               0x00                                   /**< Hardware revision of PRU. */
#define PRU_FIRM_REVISION               0x00                                   /**< Firmware revision of PRU. */
#define PRECT_MAXIMUM                   0x00                                   /**< Maximum Prect allowed for PRU. */

#define OVER_CURRENT                    3500								   /**< Maximum current tolerated */
#define OVER_VOLTAGE                    40000                                  /**< Maximum voltage tolerated */
#define OVER_TEMPERATURE                600									   /**< Maximum temperature tolerated */


union i2c{
	float f;
	uint8_t b[4];
};

/** @brief Dynamic characteristic structure. This contains elements necessary for dynamic payload. */
typedef struct
{
    union i2c         vrect;              /**< [mandatory] Vrect value from I2C (4 bytes). */
    union i2c         irect;              /**< [mandatory] Irect value from I2C (4 bytes). */
    union i2c         temp;               /**< [optional] Temperature value from I2C (4 bytes). */
	uint8_t	          alert;		      /**< [mandatory] CRU alert field for warnings to send to CTU (1 byte). */
	uint8_t	          RFU;		          /**< [optional] Reserved for future use (1 byte). */
} wpt_dynamic_payload_t;

/** @brief Dynamic characteristic structure. This contains elements necessary for static payload. */
typedef struct
{
    uint8_t           optional_fields;    /**< [mandatory] Defines which optional fields are populated (1 byte). */
    uint8_t           protocol_rev;       /**< [mandatory] Airfuel resonantsupported revision (1 bytes). */
    uint8_t           RFU1;				  /**< [N/A] Reserved for future use (1 bytes). */
	uint8_t		      pru_cat;            /**< [mandatory] Category of PRU (according to Airfuel) (1 bytes). */
	uint8_t           pru_info;           /**< [mandatory] Information regarding capabilities of PRU (1 bytes). */
    uint8_t           hard_rev;           /**< [mandatory] Hardware revision for PRU (1 bytes). */
    uint8_t           firm_rev;           /**< [mandatory] Firmware revision for PRU (1 bytes). */
    uint8_t           prect_max;          /**< [mandatory] Indicates how much power to provide to PRU (1 bytes). */
    uint16_t          vrect_min_stat;     /**< [mandatory] Lowest Vrect value restriction (2 bytes). */
	uint16_t	      vrect_high_stat;	  /**< [mandatory] Highest Vrect value restriction (2 byte). */
	uint16_t	      vrect_set;          /**< [mandatory] Set the desired voltage output for Vrect value (2 byte). */
	uint16_t		  company_id;	      /**< [optional] Company ID for PRU (2 byte). */
	uint32_t	      RFU2;		          /**< [N/A] Reserved for future use (4 byte). */
} wpt_static_payload_t;

//todo: implement charge_complete and wired_charger
/**@brief Alert characteristic structure. This contains elements necessary for alert payload. */
typedef union
{
	struct {
		uint8_t           mode_transition:2;    /**< [mandatory] Defines which optional fields are populated (1 byte). */
		uint8_t           wired_charger:1;      /**< [mandatory] Defines which optional fields are populated (1 byte). */
		uint8_t           charge_complete:1;    /**< [mandatory] Defines which optional fields are populated (1 byte). */
		uint8_t           self_protection:1;    /**< [mandatory] Defines which optional fields are populated (1 byte). */
		uint8_t           overtemperature:1;    /**< [mandatory] Defines which optional fields are populated (1 byte). */
		uint8_t           overcurrent:1;        /**< [mandatory] Defines which optional fields are populated (1 byte). */
		uint8_t           overvoltage:1;        /**< [mandatory] Defines which optional fields are populated (1 byte). */
	};
	uint8_t internal;
} alert_field_t;

typedef struct
{
	alert_field_t     alert_field;
} wpt_alert_payload_t;


/**@brief PTU static characteristic structure. This contains elements necessary for PTU static payload. */
typedef struct
{
	uint8_t           optional_fields;    /**< [mandatory] Defines which optional fields are populated (1 byte). */
    uint8_t           ptu_power;          /**< [mandatory] Power output of PTU (1 bytes). */
    uint8_t           max_impedance;      /**< [DEPRECATED] Unused (1 bytes). */
	uint8_t		      max_load;           /**< [optional] Maximum load_resistance of PTU (1 bytes). */
	uint16_t          RFU1;               /**< [N/A] Undefined (2 bytes). */
    uint8_t           ptu_class;          /**< [mandatory] Hardware class of PTU (1 bytes). */
	uint8_t           hard_rev;           /**< [mandatory] Hardware revision for PTU (1 bytes). */
    uint8_t           firm_rev;           /**< [mandatory] Firmware revision for PTU (1 bytes). */
    uint8_t           protocol_rev;       /**< [mandatory] Airfuel resonant supported revision (1 bytes). */
    uint8_t           max_num_devices;    /**< [mandatory] Maximum number of connected devices the PTU can support (1 bytes). */
	uint16_t	      company_id;	      /**< [mandatory] Company ID for PTU provider (2 byte). */
	uint32_t	      RFU2;		          /**< [N/A] Reserved for future use (4 byte). */
} wpt_ptu_static_payload_t;

/* Instanciation of characteristic values available for Airfuel. */
WPT_DYNAMIC_PAYLOAD_DEF(dyn_payload);
WPT_STATIC_PAYLOAD_DEF(static_payload);
WPT_PTU_STATIC_PAYLOAD_DEF(ptu_static_payload);
WPT_ALERT_PAYLOAD_DEF(alert_payload);


//* BLE FUNCTIONS
int gatt_svr_init(void);

#endif
