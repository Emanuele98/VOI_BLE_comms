/**
 * /////////////////////////// Description ///////////////////////////////
 * Internal structure for CTU peer module. The peer is the main container
 * for any CRU context. All things relating to CRUs are incorporated into
 * this module. It also implements all the functions necessary to create,
 * fill and destroy peers dynamically.
 * 
*/


#ifndef __PEER_H__
#define __PEER_H__

#include <stdint.h>
#include <sys/queue.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"

#include "ble_central.h"
#include"wifi.h"

/** Peer. */
struct peer_dsc {
    SLIST_ENTRY(peer_dsc) next;
    struct ble_gatt_dsc dsc;
};
SLIST_HEAD(peer_dsc_list, peer_dsc);

struct peer_chr {
    SLIST_ENTRY(peer_chr) next;
    struct ble_gatt_chr chr;

    struct peer_dsc_list dscs;
};
SLIST_HEAD(peer_chr_list, peer_chr);

struct peer_svc {
    SLIST_ENTRY(peer_svc) next;
    struct ble_gatt_svc svc;

    struct peer_chr_list chrs;
};
SLIST_HEAD(peer_svc_list, peer_svc);

struct peer;
typedef void peer_disc_fn(const struct peer *peer, int status, void *arg);

typedef enum { 
    VOI_3PAU, 
    VOI_6F35, 
    VOI_CE8J, 
    VOI_D8X5,
    EMPTY
} voi_code_enum;

//idea (use this for all the calls except switch on-off - need a string field)

union i2c {
	float f;
	uint8_t b[4];
};

/** @brief Dynamic characteristic structure. This contains elements necessary for dynamic payload. */
typedef struct
{
    union i2c         vrect;              /**< [mandatory] Vrect value from I2C (4 bytes). */
    union i2c         irect;              /**< [mandatory] Irect value from I2C (4 bytes). */
    union i2c         temp1;              /**< [optional] Temperature value from I2C (4 bytes). */
    union i2c         temp2;              /**< [optional] Temperature value from I2C (4 bytes). */
	uint8_t	          alert;		      /**< [mandatory] CRU alert field for warnings to send to CTU (1 byte). */
	uint8_t	          RFU;		          /**< [optional] Reserved for future use (1 byte). */
    float             rx_power;           /* Calculate received power if peer is CRU */
    float             tx_power;           /* Calculate transmitted power if peer is CTU */
    struct tm         dyn_time;           /** Time */
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
   		uint8_t           FOD:1;                /**< [mandatory] Defines which optional fields are populated (1 byte). */
	};
	uint8_t internal;
} alert_field_t;

typedef struct
{
	alert_field_t     alert_field;
    struct tm         alert_time;           /** Time */
} wpt_alert_payload_t;

/**@brief Control characteristic structure. This contains elements necessary for control payload. */
typedef struct
{
	uint8_t           enable;              /**< [mandatory] Enable command for PTU (1 byte). */
	uint8_t           full_power;          /**< [mandatory] Full power mode or not (1 byte). */
	uint8_t           led;           	   /**< [optional] Set LED state */
	uint16_t          RFU;                 /**< [N/A] Undefined (2 byte). */
} wpt_control_payload_t;

struct peer {
    SLIST_ENTRY(peer) next;

    uint16_t conn_handle;

    /** List of discovered GATT services. */
    struct peer_svc_list svcs;

    /** Keeps track of where we are in the service discovery process. */
    uint16_t disc_prev_chr_val;
    struct peer_svc *cur_svc;

    /** Callback that gets executed when service discovery completes. */
    peer_disc_fn *disc_cb;
    void *disc_cb_arg;

    /** Task handle for dynamic char read */
    TaskHandle_t task_handle;

    /** Peripheral payloads. */
    wpt_alert_payload_t alert_payload;
    wpt_dynamic_payload_t dyn_payload;
    wpt_static_payload_t stat_payload;
    wpt_control_payload_t contr_payload;

    /* POSITION OF THE PAD UPON WHICH THE PEER IS PLACED */
    int8_t position;

    /* CRU VOI CODE FOR ID */
    voi_code_enum voi_code;
    char voi_code_string[5];

    /* bool to detect if localization process is currently going */
    bool localization_process;

    /* bool to check wheter the power is received correctly */
    bool correct;

    /* bool to know whether the peer is a CRU (1) or A-CTU (0) */
    bool CRU;
};

/* Linked list Head definition */
SLIST_HEAD(, peer) peers;

int peer_disc_all(uint16_t conn_handle, peer_disc_fn *disc_cb,
                  void *arg);
const struct peer_dsc *
peer_dsc_find_uuid(const struct peer *peer, const ble_uuid_t *svc_uuid,
                   const ble_uuid_t *chr_uuid, const ble_uuid_t *dsc_uuid);
const struct peer_chr *
peer_chr_find_uuid(const struct peer *peer, const ble_uuid_t *svc_uuid,
                   const ble_uuid_t *chr_uuid);
const struct peer_svc *
peer_svc_find_uuid(const struct peer *peer, const ble_uuid_t *uuid);
int peer_delete(uint16_t conn_handle);
int peer_add(uint16_t conn_handle, bool CRU);
int peer_init(int max_peers, int max_svcs, int max_chrs, int max_dscs);
struct peer *peer_find(uint16_t conn_handle);
uint8_t peer_get_NUM_CRU(void);
uint8_t peer_get_NUM_AUX_CTU(void);
bool all_AUX_CTU_set(void);
bool is_peer_alone(void);
bool CTU_is_peer_charging(struct peer *peer);
bool CTU_is_charging(void);
uint8_t loc_pad_find(void);
bool all_low_power_off(void);
bool low_power_alone(uint8_t pos);
void set_scooters_tobechecked(void);
bool all_scooters_checked(void);

// find the aux CTU
struct peer *Aux_CTU_find(uint16_t pos);
// find the CRU
struct peer *CRU_find(uint16_t pos);


//localization functions
bool current_localization_process(void);
int get_peer_position(uint16_t conn_handle);


#endif