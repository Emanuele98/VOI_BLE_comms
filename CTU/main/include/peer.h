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

#include "ble_central.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"

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

union i2c {
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
    uint8_t           mac_0;    /**< [mandatory] MAC address field 0 (1 byte). */
    uint8_t           mac_1;    /**< [mandatory] MAC address field 1 (1 byte). */
    uint8_t           mac_2;    /**< [mandatory] MAC address field 2 (1 byte). */
    uint8_t           mac_3;    /**< [mandatory] MAC address field 3 (1 byte). */
    uint8_t           mac_4;    /**< [mandatory] MAC address field 4 (1 byte). */
    uint8_t           mac_5;    /**< [mandatory] MAC address field 5 (1 byte). */
} wpt_static_payload_t;

/**@brief Alert characteristic structure. This contains elements necessary for alert payload. */
typedef union
{
	struct {
		uint8_t           charge_complete:1;    /**< [mandatory] Defines which optional fields are populated (1 byte). */
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

/**@brief Control characteristic structure. This contains elements necessary for control payload. */
typedef struct
{
	uint8_t           enable;              /**< [mandatory] Enable command for PTU (1 byte). */
	uint8_t           full_power;          /**< [mandatory] Full power mode or not (1 byte). */
	uint8_t           critical;            /**< [optional] Critical transition (1 byte). Do not involve the OR gate */
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
	SemaphoreHandle_t sem_handle;

    /** CRU payloads. */
    wpt_alert_payload_t alert_payload;
    wpt_dynamic_payload_t dyn_payload;
    wpt_static_payload_t stat_payload;
    wpt_control_payload_t contr_payload;

    /* POSITION OF THE PAD UPON WHICH THE PEER IS PLACED */
    int8_t position;

    /* keep track of comms error of the peer */
    int8_t error;
    
    /* bool to detect if localization process is currently going */
    bool localization_process;

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
bool is_peer_alone(void);
bool CTU_is_peer_charging(struct peer *peer);
bool CTU_is_charging(void);
uint8_t low_power_find(void);

// find the aux CTU
struct peer *Aux_CTU_find(uint16_t pos);

//localization functions
bool current_localization_process(void);
int get_peer_position(uint16_t conn_handle);


#endif