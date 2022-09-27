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

#include <stdint.h>

#include "esp_nimble_hci.h"

#include "esp_task_wdt.h"

#include "nimble/hci_common.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_uuid.h"
#include "host/ble_gatt.h"
#include "host/ble_gap.h"

#include <assert.h>
#include "freertos/FreeRTOS.h"

#include "esp_err.h"
#include "esp_timer.h"
#include "esp_sleep.h"

#include "CTU_states.h"
#include "airfuel_integration.h"
#include "peer.h"

#define N_CHAR_UUIDS                        5
#define N_BYTES_IN_UUID                     16

#define FIRST_SCAN_TIMEOUT                  80

#define BLE_WPT_INITIAL_CONN_ITVL_MIN       (8 * 1000 / BLE_HCI_CONN_ITVL)
#define BLE_WPT_INITIAL_CONN_ITVL_MAX       (9 * 1000 / BLE_HCI_CONN_ITVL)

#define DYNAMIC_TIMER_PERIOD               	(pdMS_TO_TICKS(100))

#define BLE_PERIODIC_SCAN_ITVL				100	
#define BLE_PERIODIC_SCAN_WIND				100

#define BLE_FIRST_SCAN_ITVL                 80
#define BLE_FIRST_SCAN_WIND					80

#define BASE_CONN_HANDLE 					0

#define N_CTU_PARAMS                        12
#define MAX_N_CHAR_IN_CONFIG                15

#define N_BYTES_IN_CTU_STATIC               17

#define MINIMUM_ADV_RSSI                    -100

#define WPT_SVC_UUID16                      0xFFFE

/* Keeps power outputs state in memory */
uint8_t low_power_pads[4];
uint8_t full_power_pads[4];


struct ble_hs_adv_fields;
struct ble_gap_conn_desc;
struct ble_hs_cfg;


int ble_central_gap_event(struct ble_gap_event *event, void *arg);
uint8_t peer_addr[6];

uint8_t CTU_static_data[N_BYTES_IN_CTU_STATIC];

struct peer;

void ble_central_scan_start(uint32_t timeout, uint16_t scan_itvl, uint16_t scan_wind);
void ble_central_update_control_enables(uint8_t enable, uint8_t full_power, struct peer *peer);
void ble_central_kill_all_CRU(void);
void ble_central_kill_all_AUX_CTU(void);
void ble_central_kill_CRU(TaskHandle_t task_handle, SemaphoreHandle_t sem_handle, uint16_t conn_handle);
void ble_central_kill_AUX_CTU(TaskHandle_t task_handle, SemaphoreHandle_t sem_handle, uint16_t conn_handle);
esp_err_t ble_central_get_CTU_static(void);

#endif
