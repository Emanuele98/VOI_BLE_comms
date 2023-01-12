/**
 * /////////////////////////// Description ///////////////////////////////
 * Internal structure for CTU BLE central module. BLE central is the core 
 * for all BLE static and dynamic functions. It communicates directly with
 * the BLE stack via the GAP and the GATT protocols. It also implements
 * all NimBLE related callbacks as well as all interactions with the peer
 * structures (CRUs). It is mainly run by the "host" task with the
 * exception of the multiple CRU tasks. The "host" task runs on CPU0
 * conjointly with the "main" task. Therefor it remains important to handle
 * delays with care to allow better handling of CPU time.
 * 
*/


#ifndef __BLE_CENTRAL_H__
#define __BLE_CENTRAL_H__

#include <stdio.h>

#include "esp_nimble_hci.h"

#include "esp_task_wdt.h"

#include "nimble/hci_common.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_uuid.h"
#include "host/ble_gatt.h"
#include "host/ble_gap.h"
#include "esp_bt.h"

#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_err.h"
#include "esp_timer.h"
#include "esp_sleep.h"

#include "CTU_states.h"
#include "peer.h"
#include "sd_card.h"
#include "wifi.h"

// number of aux unit
#define MAX_AUX_CTU                         4

//BLE
#define BLE_AGENT_COMMAND_QUEUE_LENGTH      50
#define COMMS_ERROR_LIMIT                   60

#define N_CHAR_UUIDS                        5
#define N_BYTES_IN_UUID                     16

#define BLE_WPT_INITIAL_CONN_ITVL_MIN       (10 * 1000 / BLE_HCI_CONN_ITVL)
#define BLE_WPT_INITIAL_CONN_ITVL_MAX       (30 * 1000 / BLE_HCI_CONN_ITVL)

#define CTU_TIMER_PERIOD                    pdMS_TO_TICKS(3000)
#define CRU_TIMER_PERIOD                    pdMS_TO_TICKS(3000)
#define LOC_CTU_TIMER_PERIOD               	pdMS_TO_TICKS(300)
#define LOC_CRU_TIMER_PERIOD                pdMS_TO_TICKS(100)

#define BLE_PERIODIC_SCAN_ITVL				100	
#define BLE_PERIODIC_SCAN_WIND				100

#define BLE_SCAN_TIMEOUT                    1000     
#define BLE_FIRST_SCAN_ITVL                 30          /**< The scanning interval (in units of 0.625 ms). */
#define BLE_FIRST_SCAN_WIND					30          /**< The scanning window   (in units of 0.625 ms). */
                                                        /**< The scan window must be less than 256 (160 ms) to coexist with WiFi */
#define N_BYTES_IN_CTU_STATIC               17

#define MINIMUM_ADV_RSSI                    -90
#define MINIMUM_FULLY_CHARGED_RSSI          -88

#define WPT_SVC_UUID16                      0xFFFE

#define PRU_CONTROL_CHAR_SIZE               5

// RECONNECTION TIMES
#define RECONNECTION_LOC_FAIL               300           //30 sec
#define RECONNECTION_COMMS_FAIL             30           //30 sec
#define RECONNECTION_SCOOTER_LEFT           10           //30 sec
//#define RX_RECONNECTION_AFTER_PAD_KILLED    30         //30 min

/* ASSUMPTION - scooter is in the same pad until it is disconnected */
//#define RX_RECONNECTION_CHARGE_COMPLETE     120          //2 min  

#define RX_RECONNECTION_OVERCURRENT         300          //5 min
#define RX_RECONNECTION_OVERTEMPERATURE     300          //5 min
#define RX_RECONNECTION_OVERVOLTAGE         300          //5 min

#define TX_RECONNECTION_FOD                 45           //45 sec
#define TX_RECONNECTION_OVERCURRENT         300          //5 min
#define TX_RECONNECTION_OVERTEMPERATURE     300          //5 min
#define TX_RECONNECTION_OVERVOLTAGE         300          //5 min

//BLE Agent
QueueHandle_t BLE_QueueHandle;

typedef enum{
    SCAN,
    CONNECT,
    READ,
    WRITE,
    DISCONNECT
} BLEAgentCommandType;

typedef struct {
    BLEAgentCommandType BLEAgentCommandType_t;
    uint16_t attr_handle;
    struct peer *peer;
    ble_gatt_attr_fn *cb;
} BLEAgentCommand;

/* Keeps power outputs state in memory */
uint8_t low_power_pads[4];
uint8_t full_power_pads[4];

struct ble_hs_adv_fields;
struct ble_gap_conn_desc;
struct ble_hs_cfg;

int ble_central_gap_event(struct ble_gap_event *event, void *arg);
uint8_t peer_addr[6];

struct peer;

void ble_central_scan_start(uint32_t timeout, uint16_t scan_itvl, uint16_t scan_wind);
uint8_t ble_central_update_control_enables(uint8_t enable, uint8_t full_power, uint8_t led, struct peer *peer);
void ble_central_kill_all_CRU(void);
void ble_central_kill_all_AUX_CTU(void);
void ble_central_kill_CRU(uint16_t conn_handle, TaskHandle_t task_handle);
void ble_central_kill_AUX_CTU(uint16_t conn_handle, TaskHandle_t task_handle);
uint8_t BLEAgent_Init(void);


#endif
