#include "include/ble_central.h"
#include "host/ble_att.h"
#include "nvs_flash.h"
#include "nvs.h"

/**********************************************************************************/
/**                            Restricted globals                                **/
/**********************************************************************************/

/* Defines what type of task the CRU will run on */
#define TASKS_CORE        1
#define TASK_STACK_SIZE   2000
#define TASK_PRIORITY     16

/* NVS fields that have to be set at boot */
const char ch_arr[N_CTU_PARAMS][MAX_N_CHAR_IN_CONFIG] = {
    "optional fields",
    "CTU power",
    "max impedance",
    "max load",
    "RFU 1",
    "CTU class",
    "HW revision",
    "FW revision",
    "Prot revision",
    "Max devices",
    "Company ID",
    "RFU 2"
};

int count = 0;
uint8_t last_pad1 = 0, last_pad2 = 0, last_pad3 = 0, last_pad4 = 0;



/* Debug tag */
static const char* TAG = "BLE_CENTRAL";

/**********************************************************************************/
/**                       Static function declarations                           **/
/**********************************************************************************/

/* Starts task that handles a peer connection */
static void ble_central_CRU_task_handle(void *arg);

/* Function handling subscription completion */
static int ble_central_on_subscribe(uint16_t conn_handle, void *arg);

/* Function handling static read completion */
static int ble_central_on_static_chr_read(uint16_t conn_handle,
                const struct ble_gatt_error *error,
                struct ble_gatt_attr *attr,
                void *arg);

/* Function handling write cccd completion */
static int ble_central_on_write_cccd(uint16_t conn_handle, void *arg);

/* Function handling first dynamic read completion */
static int ble_central_on_control_enable(uint16_t conn_handle,
                const struct ble_gatt_error *error,
                struct ble_gatt_attr *attr,
                void *arg);

/* Function handling localization process (dynamic readings) */
static int ble_central_on_localization_process(uint16_t conn_handle,
                const struct ble_gatt_error *error,
                struct ble_gatt_attr *attr,
                void *arg);

/* Starts the WPT Service for a specified peer */
static int ble_central_wpt_start(uint16_t conn_handle, void *arg);

/* Function handling the completion of a discovery event */
static void ble_central_on_disc_complete(const struct peer *peer, int status, void *arg);
/* Function determining if a connection should happen with a specified peer */
static int ble_central_should_connect(const struct ble_gap_disc_desc *disc);
/* Function enabling a connection if the peer device corresponds to peer framework */
static void ble_central_connect_if_interesting(const struct ble_gap_disc_desc *disc);

/* Unpacks the CRU static parameter */
static void ble_central_unpack_static_param(const struct ble_gatt_attr *attr, uint16_t conn_handle);
/* Unpacks the alert parameter for CRU */
static void ble_central_unpack_CRU_alert_param(struct os_mbuf* om, uint16_t conn_handle);
/* Unpacks the alert parameter for Auxiliary CTU */
static void ble_central_unpack_AUX_CTU_alert_param(struct os_mbuf* om, uint16_t conn_handle);

/* Unpacks the CRU dynamic parameter */
static void ble_central_unpack_dynamic_param(const struct ble_gatt_attr *attr, uint16_t conn_handle);

/** 
 * @brief Struct for customized connection parameters
 * @details Initializes all relevant connection parameters except those
 *          which are set to default intentionally
 * 
 * @param scan_itvl            Scan interval in units of 0.625ms
 * @param scan_window          Scan window in units of 0.625ms
 * @param itvl_min             Minimum connection interval supported by CTU in units of 1.25ms    
 * @param itvl_max             Maximum connection interval supported by CTU in units of 1.25ms         
 * @param latency              Slave latency supported by CTU (defaults to 0)
 * @param supervision_timeout  Link supervision timeout used internally for connecion in units of 10ms
 * @param min_ce_len           Minimum length of connection event (default)
 * @param max_ce_len           Maximum length of connection event (default)
 */

static const struct ble_gap_conn_params conn_params = {
    .scan_itvl = 0x0010,
    .scan_window = 0x000f,
    .itvl_min = BLE_WPT_INITIAL_CONN_ITVL_MIN,
    .itvl_max = BLE_WPT_INITIAL_CONN_ITVL_MAX,
    .latency = BLE_GAP_INITIAL_CONN_LATENCY,
    .supervision_timeout = 0x0020,
    .min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN,
    .max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN,
};

// Declare ALL the WPT service and its characteristics UUIDs 

const ble_uuid_t *wpt_svc_uuid = 
    BLE_UUID128_DECLARE(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0xFE, 0xFF, 0x55, 0x64);

const ble_uuid_t *wpt_char_control_uuid = 
    BLE_UUID128_DECLARE(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x70, 0xE6, 0x55, 0x64);

const ble_uuid_t *wpt_char_CTU_stat_uuid = 
    BLE_UUID128_DECLARE(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x71, 0xE6, 0x55, 0x64);

const ble_uuid_t *wpt_char_alert_uuid = 
    BLE_UUID128_DECLARE(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x72, 0xE6, 0x55, 0x64);

const ble_uuid_t *wpt_char_CRU_stat_uuid = 
    BLE_UUID128_DECLARE(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x73, 0xE6, 0x55, 0x64);

const ble_uuid_t *wpt_char_CRU_dyn_uuid = 
    BLE_UUID128_DECLARE(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x74, 0xE6, 0x55, 0x64);

/**********************************************************************************/
/**                         Misc function definitions                            **/
/**********************************************************************************/

/** 
 * @brief Kill a Auxiliary CTU task currently running on CPU1
 * @details This function only removes the peer identified by the conn_handle parameter.
 * 
 */
void ble_central_kill_AUX_CTU(TaskHandle_t task_handle, SemaphoreHandle_t sem_handle, uint16_t conn_handle)
{

    struct peer *Aux_CTU = peer_find(conn_handle);

    //disable realtive interface
    enabled[Aux_CTU->position -1] = 0;
    ble_central_update_control_enables(0, 0, Aux_CTU);    

    ESP_LOGI(TAG, "Disconnection with conn_handle=%d", conn_handle);
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    // use this to avoid reconnection
    // ble_gap_terminate(conn_handle, BLE_HS_EAPP);
    
    if (sem_handle != NULL)
    {
        vSemaphoreDelete(sem_handle);
    }

    if (task_handle != NULL)
    {

        esp_task_wdt_delete(task_handle);
        vTaskDelete(task_handle);
    }
}


/** 
 * @brief Kill a CRU task currently running on CPU1
 * @details This function only removes the peer identified by the conn_handle parameter.
 * 
 */
void ble_central_kill_CRU(TaskHandle_t task_handle, SemaphoreHandle_t sem_handle, uint16_t conn_handle)
{
    struct peer *peer = peer_find(conn_handle);

    //DISABLE localization process timer if it was ongoing
    if ((peer->localization_process) && (xTimerIsTimerActive(localization_switch_pads_t_handle) == pdTRUE))
    {
        xTimerStop(localization_switch_pads_t_handle, 10);
    }

    ESP_LOGI(TAG, "Disconnection with conn_handle=%d", conn_handle);
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    // use this to avoid reconnection
    // ble_gap_terminate(conn_handle, BLE_HS_EAPP);
    
    if (sem_handle != NULL)
    {
        vSemaphoreDelete(sem_handle);
    }

    if (task_handle != NULL)
    {

        esp_task_wdt_delete(task_handle);
        vTaskDelete(task_handle);
    }

    if (peer_get_NUM_CRU() == 0)
    {
        CTU_state_change(CTU_POWER_SAVE_STATE, NULL);
    }

}

/** 
 * @brief Kill ALL cru task currently running on CPU1
 * 
 */
void ble_central_kill_all_CRU(void)
{
    for (uint16_t conn_handle=0; conn_handle!=MYNEWT_VAL(BLE_MAX_CONNECTIONS) + 4; conn_handle++)
    {
        struct peer* peer = peer_find(conn_handle);
        if ((peer != NULL) && (peer->CRU))
        {
            ble_central_kill_CRU(peer->task_handle, peer->sem_handle, conn_handle);
        }
    }
}

/** 
 * @brief Kill ALL Auxiliary CTU task currently running on CPU1
 * 
 */
void ble_central_kill_all_AUX_CTU(void)
{
    for (uint16_t conn_handle=0; conn_handle!=MYNEWT_VAL(BLE_MAX_CONNECTIONS) + 4; conn_handle++)
    {
        struct peer* peer = peer_find(conn_handle);
        if ((peer != NULL) && (!peer->CRU))
        {
            ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, conn_handle);
        }
    }
}

/**
 * @brief Function for handling CRU position detection 
 * @details This function localize the right pad to be turned on.
 *          It switches the diffent pads sequentially for a reasonable time 
 *          to detect which one is located under the connected CRU
 *          The detection alorithm is based on the CRU Vrect 
 * 
 *          Missing: how to know one CRU is on a pad and not just nearby? Accelerometer?
 * 
 * @param conn_handle  Connection handle of the peer that is being read
 * @param error        Error structure containing return codes and status
 * @param attr         Attribute structure containing procedure attribute
 * @param arg          Additional argument
 * @return 0 on success
 */
static int ble_central_on_localization_process(uint16_t conn_handle,
                const struct ble_gatt_error *error,
                struct ble_gatt_attr *attr,
                void *arg)
{
    struct peer *peer = (struct peer *) arg;

    // Give the semaphore immediately 
    if (peer->sem_handle == NULL)
    {
        goto err;
    }
    xSemaphoreGive(peer->sem_handle);

    if (error->status != 0)
    {
        ESP_LOGE(TAG, "Unable to read (ON LOCALIZATION PROCESS): status=%d", error->status);
        // Check if the task has not already been deleted
        if (eTaskGetState(peer->task_handle) != eDeleted)
        {
            ble_central_kill_CRU(peer->task_handle, peer->sem_handle, conn_handle);
        }
        goto err;
    }

    // Unpack CRU Dynamic Characteristic
    ble_central_unpack_dynamic_param(attr, conn_handle);

    //*DETECT POSITION OF THE CRU USING THE DYN CHR 
    //* DO NOT GO TO POWER TRANSFER STATE UNTIL A POSITION IS DETECTED

    // is Vrect value above the Treshold?
    if( peer->dyn_payload.vrect.f > VOLTAGE_LOC_THRESH )
    {
        //number of pads
        uint8_t size = peer_get_NUM_AUX_CTU();
        uint8_t pads_already_on[size];
        memset(pads_already_on, 0, size);

        get_pads_already_on(pads_already_on);

        if ((!pads_already_on[0]) && (enabled[0]))
        {
            peer->position = 1;
        } else if ((!pads_already_on[1]) && (enabled[1]))
            {
                peer->position = 2;
            } else if ((!pads_already_on[2]) && (enabled[2]))
            {
                peer->position = 3;
            } else if ((!pads_already_on[3]) && (enabled[3]))
            {
                peer->position = 4;
            } else {
                goto err;
            }
        
        ESP_LOGI(TAG, " CRU is in position %d", peer->position );
        
        //CONTROL ENABLE
        struct peer *Aux_CTU = Aux_CTU_find(peer->position);
        if(Aux_CTU == NULL) {
            goto err;
        }
        ble_central_update_control_enables(1, 1, Aux_CTU);

        if (is_peer_alone())
        {
            CTU_state_change(CTU_POWER_TRANSFER_STATE, (void *)peer);
        }
            
        //change the peer variable to end the localization process
        peer->localization_process = false;

        //stop timer
        xTimerStop(localization_switch_pads_t_handle, 10);
        count = 0;
    } 
    return 0;

    err:
        ESP_LOGE(TAG, "on_localization_process");    
        return ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

/** 
 * @brief Dynamic read callback function for Auxiliary CTU 
 * @details Application callback.  Called when the read of the CRU Dynamic
 *          Parameter characteristic has completed.
 *          This reads the I2C measurements and switches on/off the pads when enabled[] changes writing the Control characteristic.
 * 
 * @param conn_handle  Connection handle of the peer that is being read
 * @param error        Error structure containing return codes and status
 * @param attr         Attribute structure containing procedure attribute
 * @param arg          Additional argument
 * @return 0 on success
 */
static int ble_central_on_AUX_CTU_dyn_read(uint16_t conn_handle,
                const struct ble_gatt_error *error,
                struct ble_gatt_attr *attr,
                void *arg)
{

    struct peer *peer = (struct peer *) arg;
    
    // Give the semaphore immediately 
    if (peer->sem_handle == NULL) {
        goto err;
    }
    xSemaphoreGive(peer->sem_handle);
    
    // If the characteristic has not been read correctly
    if (error->status != 0)
    {
        ESP_LOGE(TAG, "Unable to read (ON AUX CTU DYN): status=%d", error->status);
        // Check if the task has not already been deleted
        if (eTaskGetState(peer->task_handle) != eDeleted)
        {
            ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, conn_handle);
        }
        goto err;
    }
   
    // Unpack peer Static Characteristic
    ble_central_unpack_dynamic_param(attr, conn_handle);

    //SEND SWITCH COMMAND THROUGH CONTROL CHR WHEN ENABLED[pad position] changes
    switch(peer->position)
    {
        case 1:
            if(last_pad1 != enabled[0])
            {
                if (last_pad1 == 0) {
                    ble_central_update_control_enables(1, 0, peer);
                } else {
                    ble_central_update_control_enables(0, 0, peer);
                }
                last_pad1 = enabled[0];
            }
            break;
        case 2:
            if(last_pad2 != enabled[1])
            {
                if (last_pad2 == 0) {
                    ble_central_update_control_enables(1, 0, peer);
                } else {
                    ble_central_update_control_enables(0, 0, peer);
                }
                last_pad2 = enabled[1];
            }
            break;
        case 3:
            if(last_pad3 != enabled[2])
            {
                if (last_pad3 == 0) {
                    ble_central_update_control_enables(1, 0, peer);
                } else {
                    ble_central_update_control_enables(0, 0, peer);
                }
                last_pad3 = enabled[2];
            }
            break;
        case 4:
            if(last_pad4 != enabled[3])
            {
                if (last_pad4 == 0) {
                    ble_central_update_control_enables(1, 0, peer);
                } else {
                    ble_central_update_control_enables(0, 0, peer);
                }
                last_pad4 = enabled[3];
            }
            break;
    }

    return 0;

    err:
        ESP_LOGE(TAG, "on_AUX_CTU_dyn_read");    
        return ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);

}

/** 
 * @brief Dynamic read callback function for CRU
 * @details Application callback.  Called when the read of the CRU Dynamic
 *          Parameter characteristic has completed. 
 * 
 * @param conn_handle  Connection handle of the peer that is being read
 * @param error        Error structure containing return codes and status
 * @param attr         Attribute structure containing procedure attribute
 * @param arg          Additional argument
 * @return 0 on success
 */
static int ble_central_on_CRU_dyn_read(uint16_t conn_handle,
                const struct ble_gatt_error *error,
                struct ble_gatt_attr *attr,
                void *arg)
{
    struct peer *peer = (struct peer *) arg;
    
    // Give the semaphore immediately 
    if (peer->sem_handle == NULL) {
        goto err;
    }
    xSemaphoreGive(peer->sem_handle);
    
    // If the characteristic has not been read correctly
    if (error->status != 0)
    {
        ESP_LOGE(TAG, "Unable to read (ON CRU DYN): status=%d", error->status);
        // Check if the task has not already been deleted
        if (eTaskGetState(peer->task_handle) != eDeleted)
        {
            ble_central_kill_CRU(peer->task_handle, peer->sem_handle, conn_handle);
        }
        goto err;
    }
   
    // Unpack peer Static Characteristic
    ble_central_unpack_dynamic_param(attr, conn_handle);
    return 0;

    err:
        ESP_LOGE(TAG, "on_CRU_dyn_read");    
        return ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

/** 
 * @brief Alert read callback function
 * @details Application callback.  Called when the read of the Alert
 *          Parameter characteristic has completed. This will trigger the callback 
 *          on the peripheral, which will send a notification. 
 * 
 * @param conn_handle  Connection handle of the peer that is being read
 * @param error        Error structure containing return codes and status
 * @param attr         Attribute structure containing procedure attribute
 * @param arg          Additional argument
 * @return 0 on success
 */

static int ble_central_on_alert_read(uint16_t conn_handle,
                const struct ble_gatt_error *error,
                struct ble_gatt_attr *attr,
                void *arg)
{
    struct peer *peer = (struct peer *) arg;
    
    // Give the semaphore immediately 
    if (peer->sem_handle == NULL)
    {
        goto err;
    }
    xSemaphoreGive(peer->sem_handle);
    
    // If the characteristic has not been read correctly
    if (error->status != 0)
    {
        ESP_LOGE(TAG, "Unable to read (ON CRU ALERT): status=%d", error->status);
        // Check if the task has not already been deleted
        if (eTaskGetState(peer->task_handle) != eDeleted)
        {
            if(peer->CRU) {
                ble_central_kill_CRU(peer->task_handle, peer->sem_handle, conn_handle);
            } else {
                ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, conn_handle);
            }
        }
        goto err;
    }
   
    return 0;

    err:
        ESP_LOGE(TAG, "on_alert_read");    
        return ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);

}

/** 
 * @brief AUXILIARY CTU WPT service task handle function
 * @details Contains the loop over which all A-CTU service tasks will run. It will mainly read
 *          the Dynamic characteristic (containing I2C measurements) while also making sure the CRU is still
 *          connected. If the CRU disconnects or encounters any issues the task will
 *          end, the semaphore will be deleted and the connection will be ended.
 *          Additionally, it will switch on/off the pads when enabled[] changes writing the Control characteristic.
 * 
 * @param arg A void pointer which contains the peer structure
 */
static void ble_central_AUX_CTU_task_handle(void *arg)
{
    int rc = 0;
    esp_err_t err_code;
    const struct peer_chr *dynamic_chr = NULL;
    const struct peer_chr *alert_chr = NULL;

    //enabled for testing (when no CRU connected)
    //xTimerStart(localization_switch_pads_t_handle, 0);


    struct peer *peer = (struct peer *) arg;

    dynamic_chr = peer_chr_find_uuid(peer,
                            wpt_svc_uuid,
                            wpt_char_CRU_dyn_uuid);
    if (dynamic_chr == NULL)
    {
        ESP_LOGE(TAG, "Error: Failed to read dynamic characteristic; rc=%d", rc);
        ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        return;
    }

    alert_chr = peer_chr_find_uuid(peer,
                        wpt_svc_uuid,
                        wpt_char_alert_uuid);

    if (alert_chr == NULL)
    {
        ESP_LOGE(TAG, "ALERT chr not found!");
        ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        return;
    }

    err_code = esp_task_wdt_add(peer->task_handle);
    ESP_ERROR_CHECK(err_code);

    
    /* WPT task loop */
    while (1)
    {
        /* Fringe case which might happen in case the disconnection is not completed properly */
        if (peer == NULL || peer->sem_handle == NULL)
        {
            esp_task_wdt_delete(NULL);
            vTaskDelete(NULL);
            return;
        }

        //check alert is fine
        if (peer->dyn_payload.alert == 0)
        {
            {   
                //* POWER TRANSFER PROCESS (keep reading dyn chr)
                if (xSemaphoreTake(peer->sem_handle, DYNAMIC_TIMER_PERIOD) == pdTRUE)
                {
                    /* Initiate Read procedure */        
                    rc = ble_gattc_read(peer->conn_handle, dynamic_chr->chr.val_handle,
                                    ble_central_on_AUX_CTU_dyn_read, (void *)peer);
                }
            }
        } else 
            {
                rc = ble_gattc_read(peer->conn_handle, alert_chr->chr.val_handle,
                                ble_central_on_alert_read, (void *)peer);
            }
        if (rc != ESP_OK)
        {
            if (rc == BLE_HS_ETIMEOUT || rc == BLE_HS_ENOMEM)
            {
                /* Restart the task */
                ESP_LOGW(TAG, "WPT task error but loop anyway rc=%d", rc);
                vTaskDelay(10);
            }
            else
            {
                ESP_LOGE(TAG, "Error: Failed to read dynamic of Auxiliary CTU; rc=%d", rc);
                ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, peer->conn_handle);
            }
        }

        /* Feed watchdog */
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

/** 
 * @brief CRU WPT service task handle function
 * @details Contains the loop over which all CRU WPT service tasks will run. It will mainly read
 *          the dynamic characteristic while also making sure the CRU is still
 *          connected. If the CRU disconnects or encounters any issues the task will
 *          end, the semaphore will be deleted and the connection will be ended.
 * 
 * @param arg A void pointer which contains the peer structure
 */
static void ble_central_CRU_task_handle(void *arg)
{
    int rc = 0;
    esp_err_t err_code;
    const struct peer_chr *dynamic_chr = NULL;
    const struct peer_chr *alert_chr = NULL;

    struct peer *peer = (struct peer *) arg;

    dynamic_chr = peer_chr_find_uuid(peer,
                            wpt_svc_uuid,
                            wpt_char_CRU_dyn_uuid);
    if (dynamic_chr == NULL)
    {
        ESP_LOGE(TAG, "Error: Failed to read dynamic characteristic; rc=%d", rc);
        ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        return;
    }

    alert_chr = peer_chr_find_uuid(peer,
                        wpt_svc_uuid,
                        wpt_char_alert_uuid);

    if (alert_chr == NULL)
    {
        ESP_LOGE(TAG, "ALERT chr not found!");
        ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        return;
    }

    err_code = esp_task_wdt_add(peer->task_handle);
    ESP_ERROR_CHECK(err_code);

    //FIRST LOCALIZATION PROCESS --> only one a time! need global variable
    //enable power of one pad 
    //wait a reasonable time to see it in the rx
    //cycle through pads to see if which CRU has the Vrect value above the Treshold
    //ENSURE only one localization process per time!
    //discard pad which are already on!
    //check alert is fine
    //need a timeout for when the localization is not detected (probably another rx is in the process)
    //NO POWER STATE UNTIL THE POSITION IS DETECTED

    //discard pad which are already on!
    uint8_t size = peer_get_NUM_AUX_CTU();
    uint8_t pads_already_on[size];
    memset(pads_already_on, 0, size);

    get_pads_already_on(pads_already_on);
    
    /* WPT task loop */
    while (1)
    {
        /* Fringe case which might happen in case the disconnection is not completed properly */
        if (peer == NULL || peer->sem_handle == NULL)
        {
            esp_task_wdt_delete(NULL);
            vTaskDelete(NULL);
            return;
        }

        //*LOCALIZATION PROCESS (find position of CRU)
        //! Enable power output in LOW POWER MODE (avoid charging of faulty battery placed on a pad)
        //check alert is fine
        if (peer->dyn_payload.alert == 0)
        {
            // ENSURE only one localization process per time!
            if ((peer->position == 0) && ((!current_localization_process())))
            {
                switch (count)
                {
                    case 0:
                        peer->localization_process = true;
                        count++; 
                        //start timer which takes care of consequentially switching the pads in low power mode
                        xTimerStart(localization_switch_pads_t_handle, 0);
                        break;
                    
                    //TIMEOUT FOR DETECTING LOCALIZATION (200*50ms) -- 10s
                    case 200:
                        ESP_LOGW(TAG, "Localization timer expires! Disconnecting");
                        count = 0;
                        peer->localization_process = false;
                        ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);
                        break;

                    // read dyn chr
                    default:
                        if (xSemaphoreTake(peer->sem_handle, DYNAMIC_TIMER_PERIOD) == pdTRUE)
                        {
                            count++;
                            rc = ble_gattc_read(peer->conn_handle, dynamic_chr->chr.val_handle,
                                ble_central_on_localization_process, (void *)peer);
                        }
                        break;
                }
            } else
            {   
                //* POWER TRANSFER PROCESS (keep reading dyn chr)
                if (xSemaphoreTake(peer->sem_handle, DYNAMIC_TIMER_PERIOD) == pdTRUE)
                {
                    /* Initiate Read procedure */        
                    rc = ble_gattc_read(peer->conn_handle, dynamic_chr->chr.val_handle,
                                    ble_central_on_CRU_dyn_read, (void *)peer);
                }
            }
        } else 
            {
                rc = ble_gattc_read(peer->conn_handle, alert_chr->chr.val_handle,
                                ble_central_on_alert_read, (void *)peer);
            }
        if (rc != ESP_OK)
        {
            if (rc == BLE_HS_ETIMEOUT || rc == BLE_HS_ENOMEM)
            {
                /* Restart the task */
                ESP_LOGW(TAG, "WPT task error but loop anyway rc=%d", rc);
                vTaskDelay(10);
            }
            else
            {
                ESP_LOGE(TAG, "Error: Failed to read dynamic chr of CRU; rc=%d", rc);
                peer->localization_process = false;
                ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);
            }
        }

        /* Feed watchdog */
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief WPT service start function
 * @details Officially starts the WPT service by adding task handlers and 
 *          semaphore handlers to peer structure.
 * 
 * @param conn_handle  Connection handle of the peer that is being read
 * @param arg          Additional argument (peer)
 * 
 * @return 1 on succes, 0 otherwise
 */
static int ble_central_wpt_start(uint16_t conn_handle, void *arg)
{
    struct peer *peer = (struct peer *)arg;

    if (peer == NULL)
    {
        return 0;
    }

    TaskHandle_t task_handle;
    SemaphoreHandle_t sem_handle;
    BaseType_t err_code;

    sem_handle = xSemaphoreCreateBinary();
    assert(sem_handle != NULL);

    if(peer->CRU)
    {
        //initialize peer position as not detected
        peer->position = 0;

        char task_name[] = "CRU";
        char num = conn_handle + '0';
        strncat(task_name, &num, 1);

        /* Creates one CRU task for every success and completes connection */
        err_code = xTaskCreatePinnedToCore((void *)ble_central_CRU_task_handle,
                        task_name, TASK_STACK_SIZE, (void *)peer, TASK_PRIORITY,
                        &task_handle, TASKS_CORE);
    }
    else {
        char task_name[] = "AUX_CTU";
        char num = conn_handle + '0';
        strncat(task_name, &num, 1);

        /* Creates one AUX CTU task for every success and completes connection */
        err_code = xTaskCreatePinnedToCore((void *)ble_central_AUX_CTU_task_handle,
                        task_name, TASK_STACK_SIZE, (void *)peer, TASK_PRIORITY,
                        &task_handle, TASKS_CORE);
    }

    if (err_code != pdPASS)
    {
        goto err;
    }

    configASSERT(sem_handle);
    configASSERT(task_handle);

    /* Adds the task handle to the peer structure to allow modifications in runtime */
    peer->task_handle = task_handle;
    peer->sem_handle = sem_handle;

    /* XXX DO NOT REMOVE OR IT BREAKS */
    xSemaphoreGive(peer->sem_handle);

    return 1;
    err:
        ESP_LOGW(TAG, "on_wpt_start");
        return ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

/** @brief On control write handle function
 * @details Unpacks 1 CRU dynamic read procedure and check the CRU.alert field
 *          to finally start the WPT service 
 *          
 * 
 * @param conn_handle  Connection handle of the peer that is being read
 * @param error        Error structure containing return codes and status
 * @param attr         Attribute structure containing procedure attribute
 * @param arg          Additional argument
 * @return 0 on success
 */
static int ble_central_on_control_enable(uint16_t conn_handle,
                const struct ble_gatt_error *error,
                struct ble_gatt_attr *attr,
                void *arg)
{
    struct peer *peer = (struct peer *) arg;
    const struct peer_chr *control_chr;

    if (error->status == 0)
    {
        if(!peer->CRU)
        {
            // Get attribute value of CRU control characteristic
            control_chr = peer_chr_find_uuid((const struct peer *)peer,
                                wpt_svc_uuid,
                                wpt_char_control_uuid);
            if (control_chr == NULL)
            {
                goto err;
            }
        }
        
        // Unpack CRU Dynamic Characteristic
        ble_central_unpack_dynamic_param(attr, conn_handle);

        if (peer->dyn_payload.alert == 0)
        {
            ble_central_wpt_start(conn_handle, (void *)peer);
        }
    }
    else
    {
        goto err;
    }

    return 0;

    err:
        ESP_LOGW(TAG, "on_control_enable");
        return ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

/** 
 * @brief On discovery completion handle
 * @details Reads the CRU static characteristic and validates status of
 *          connection procedure.
 * 
 * @param peer         Peer structure containing all CRU information
 * @param error        Current connection procedure status
 * @param arg          Additional argument
 */
static void ble_central_on_disc_complete(const struct peer *peer, int status, void *arg)
{
    /* Service discovery has completed successfully.  Now we have a complete
     * list of services, characteristics, and descriptors that the peer
     * supports.
     */

    if (status != 0)
    {
        // Service discovery failed.  Terminate the connection.
        goto err;
    }

    const struct peer_chr *static_chr;
    int rc;

    if (peer == NULL)
    {
        goto err;
    }

    // Read the CRU Static parameter characteristic.
    static_chr = peer_chr_find_uuid(peer,
                             wpt_svc_uuid,
                             wpt_char_CRU_stat_uuid);
                             
    if (static_chr == NULL)
    {
        rc = BLE_HS_ENOTCONN;
        goto err;
    }

    rc = ble_gattc_read(peer->conn_handle, static_chr->chr.val_handle,
                        ble_central_on_static_chr_read, (void *)peer);
    if (rc != ESP_OK)
    {
        goto err;
    }
    
    return;
    err:
        ESP_LOGW(TAG, "on_disc_complete");
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

/** 
 * @brief On subscribe handle function
 * @details Reads the CRU dynamic characteristic once while also starting the
 *          whole WPT procedure on the APP level.
 * 
 * @param conn_handle  Connection handle of the peer that is being read
 * @param error        Error structure containing return codes and status
 * @param attr         Attribute structure containing procedure attribute
 * @param arg          Additional argument
 * @return 0 on success
 */
static int ble_central_on_subscribe(uint16_t conn_handle, void *arg)
{
    
    int rc = 0;
    struct peer *peer = peer_find(conn_handle);
    const struct peer_chr *dynamic_chr = NULL;

    // Get chr value of CRU dynamic 
    dynamic_chr = peer_chr_find_uuid(peer,
                        wpt_svc_uuid,
                        wpt_char_CRU_dyn_uuid);
    if (dynamic_chr == NULL)
    {
        rc = BLE_HS_ENOTSUP;
        goto err;
    }

    rc = ble_gattc_read(peer->conn_handle, dynamic_chr->chr.val_handle,
                            ble_central_on_control_enable, (void *)peer);
    if (rc != ESP_OK)
    {
        goto err;
    }
    
    return 0;

    err:
        ESP_LOGW(TAG, "on_subscribe");
        return ble_gap_terminate(peer->conn_handle, rc);
}


/** @brief On CCCD write handle function
 * @details Attempts to subscribe to the Alert characterisitc by writing 2 
 *          enable bytes to the CCCD.
 * 
 * @param conn_handle  Connection handle of the peer that is being read
 * @param error        Error structure containing return codes and status
 * @param attr         Attribute structure containing procedure attribute
 * @param arg          Additional argument
 * @return 0 on success
 */
static int ble_central_on_write_cccd(uint16_t conn_handle, void *arg)
{
    /* Subscribe to notifications for the Alert characteristic. A central enables
     * notifications by writing two bytes (1, 0) to the characteristic's client-
     * characteristic-configuration-descriptor (CCCD).
     */
    
    int rc = 0;
    const struct peer_dsc *dsc = NULL;

    uint8_t value[2];
    const struct peer *peer = (const struct peer *) arg;


    dsc = peer_dsc_find_uuid(peer,
                            wpt_svc_uuid,
                            wpt_char_alert_uuid,
                            BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16));
    if (dsc == NULL)
    {
        ESP_LOGE(TAG, "dsc not found!");
        rc = BLE_HS_ENOTCONN;
        goto err;
    }

    value[0] = 1;
    value[1] = 0;
    
    //doesn't seem to change anything but theoretically required
    rc = ble_gattc_write_no_rsp_flat(conn_handle, dsc->dsc.handle,
                            value, sizeof value);


    if (rc != ESP_OK)
    {
        goto err;
    }
    rc = ble_central_on_subscribe(conn_handle, (void *)peer);
    return 0;

    err:
        ESP_LOGW(TAG, "on_write_cccd");
        return ble_gap_terminate(peer->conn_handle, rc);
}

/** 
 * @brief On CRU Static read handle function
 * @details Once the CRU static characteristic is read, this function will attempt 
 *          to unpack the data coming from said characteristic. Next, it will try
 *          to write on the CTU static characteristic.
 * 
 * @param conn_handle  Connection handle of the peer that is being read
 * @param error        Error structure containing return codes and status
 * @param attr         Attribute structure containing procedure attribute
 * @param arg          Additional argument
 * @return 0 on success
 */
static int ble_central_on_static_chr_read(uint16_t conn_handle,
                const struct ble_gatt_error *error,
                struct ble_gatt_attr *attr,
                void *arg)
{
    const struct peer *peer = (const struct peer *) arg;
    const struct peer_chr *CTU_stat_chr;
    int rc;

    if (error->status == 0)
    {
        // Unpack CRU Static Characteristic
        ble_central_unpack_static_param(attr, conn_handle);
    }
    else
    {
        if (error->status == BLE_HS_ENOTCONN)
        {
            return error->status;
        }
        rc = error->status;
        goto err;
    }

    // Write CTU Static Characteristic
    CTU_stat_chr = peer_chr_find_uuid(peer,
                             wpt_svc_uuid,
                             wpt_char_CTU_stat_uuid);
    if (CTU_stat_chr == NULL)
    {
        ESP_LOGW(TAG, "no chr -CTU stat- found");
        rc = BLE_HS_ENOTCONN;
        goto err;
    }
    rc = ble_gattc_write_no_rsp_flat(peer->conn_handle, CTU_stat_chr->chr.val_handle,
                              (void *)CTU_static_data, (sizeof CTU_static_data)/(sizeof CTU_static_data[0]));

    rc = 0;
    if (rc != ESP_OK)
    {
        goto err;
    }
    rc = ble_central_on_write_cccd(conn_handle, (void*)peer);
    return 0;

    err:
        ESP_LOGW(TAG, "on_CRU_static_read");
        return ble_gap_terminate(peer->conn_handle, rc);        
}

/**********************************************************************************/
/**                       Compound BLE Central functions                         **/
/**********************************************************************************/

/** 
 * @brief Function starting the scan procedure
 * @details Starts the GAP scan procedure which also starts the rest of the 
 *          connection procedure.
 * 
 * 
 * @param timeout time allocated before end of scan
 */
void ble_central_scan_start(uint32_t timeout, uint16_t scan_itvl, uint16_t scan_wind)
{
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != ESP_OK)
    {
        return;
    }

    /* Tell the controller to do not filter duplicates; 
       we want to connect only when the device is close enough to the platform.
     */
    disc_params.filter_duplicates = 0;

    /**
     * Perform a passive scan.  I.e., don't send follow-up scan requests to
     * each advertiser.
     */
    disc_params.passive = 1;

    /* Use defaults for the rest of the parameters. */
    disc_params.itvl = scan_itvl;
    disc_params.window = scan_wind;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(own_addr_type, timeout, &disc_params,
                      ble_central_gap_event, NULL);
    if (rc != ESP_OK && rc != BLE_HS_EALREADY)
    {
        ESP_LOGE(TAG, "Error initiating GAP discovery procedure; rc=%d\n", rc);
    }
}


/**
 * Indicates whether we should try to connect to the sender of the specified
 * advertisement.  The function returns a positive result if the device
 * advertises connectability and match the uuid service characteristic.
 */
static int ble_central_should_connect(const struct ble_gap_disc_desc *disc)
{
    int rc;
    struct ble_hs_adv_fields fields;

    rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
    if (rc != ESP_OK) {
        return 0;
    }

    // The device has to be advertising connectability.
    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
            disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND)
    {
        return 0;
    }

    //strenght of the received signal
    if (disc->rssi < MINIMUM_ADV_RSSI)
    {
        //ESP_LOGE(TAG, "too weak! \n");
        return 0;
    }

   // The device has to advertise support for the WPT service (0xFFFE).

   for (int i = 0; i < fields.num_uuids16; i++) {
        if (ble_uuid_u16(&fields.uuids16[i].u) == WPT_SVC_UUID16) {
            if(fields.appearance == 1) {
                return 1;
            } /* ALLOW CONNECTION TO CRU ONLY AFTER AT LEAST ONE SUCCESSFULL CONNECTION TO A-CTU */
            else if((fields.appearance == 2)/* && (peer_get_NUM_AUX_CTU() > 0)*/) {
                return 2;
            }
        }
    }
    
   return 0;
}

/**
 * Connects to the sender of the specified advertisement if it looks
 * interesting.  A device is "interesting" if it advertises connectability and
 * support for the Alert Notification service.
 */
static void ble_central_connect_if_interesting(const struct ble_gap_disc_desc *disc)
{
    int rc;
    struct ble_gap_conn_desc *out_desc = NULL;
    uint8_t own_addr_type;
    int slave_type = 0;

    /* Don't do anything if we don't care about this advertiser. */
    if (!ble_central_should_connect(disc)) {
        return;
    } else if(ble_central_should_connect(disc) == 1) {
        ESP_LOGI(TAG, "AUX CTU found!");
        slave_type = 1;
    } else if(ble_central_should_connect(disc) == 2) {
        ESP_LOGI(TAG, "CRU found!");
        slave_type = 2;
    }

    /* Scanning must be stopped before a connection can be initiated. */
    rc = ble_gap_disc_cancel();
    if (rc != ESP_OK)
    {
        return;
    }

    if (peer_get_NUM_CRU() == 0)
    {
        CTU_state_change(CTU_LOW_POWER_STATE, NULL);
        vTaskDelay(10);
    }

    /* Figure out address to use for connect (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Try to connect the advertiser.  Allow 100 ms for
     * timeout.
     */
    rc = ble_gap_connect(own_addr_type, &disc->addr, 100, &conn_params,
                         ble_central_gap_event, (void *)slave_type);

    /* Signal Main context that the peer has been discovered
    * and matches what the WPT Service prescribes
    */
    if (rc == BLE_HS_EDONE)
    {
        ble_gap_conn_find_by_addr(&disc->addr, out_desc);
        return;
    }
    else if (rc != 0)
    {
        return;
    }
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that is
 * established.  ble_central uses the same callback for all connections.
 *
 * @param event                 The event being signalled.
 * @param arg                   Application-specified argument; unused by
 *                                  ble_central.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
int ble_central_gap_event(struct ble_gap_event *event, void *arg)
{
    int rc = 0;
    struct peer *peer;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        
        /* Try to connect to the advertiser if it looks interesting. */
        ble_central_connect_if_interesting(&event->disc);
        break;

    case BLE_GAP_EVENT_CONNECT:

        /* A new connection was established or a connection attempt failed. */
        if (event->connect.status == ESP_OK)
        {
            const uint8_t* slave_type = (const uint8_t *)arg;

            //convert pointer to integer
            uintptr_t type = (uintptr_t) slave_type;

            if(type == 1) {
                ESP_LOGI(TAG, "Connection EVT with AUX CTU");
                /* Adds AUX CTU to list of connections */
                rc = peer_add(event->connect.conn_handle, 0);
            } else if (type == 2) {
                ESP_LOGI(TAG, "Connection EVT with CRU");
                /* Adds CRU to list of connections */
                rc = peer_add(event->connect.conn_handle, 1);
            }

            if (rc != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to add peer; rc=%d\n", rc);
                break;
            }

            /* Perform service discovery. */
            rc = peer_disc_all(event->connect.conn_handle,
                               ble_central_on_disc_complete, NULL);
            if (rc != ESP_OK)
            {
                peer_delete(event->connect.conn_handle);
                ESP_LOGE(TAG, "Failed to discover services; rc=%d\n", rc);
            }

            break;
        }
        else 
        {
            /* Connection attempt failed; resume scanning. */
            ESP_LOGE(TAG, "Error: Connection failed; status=%d\n",
                           event->connect.status);
            if (peer_get_NUM_CRU() == 0)
            {
                CTU_state_change(CTU_POWER_SAVE_STATE, NULL);
            }
            break;
        }

    case BLE_GAP_EVENT_DISCONNECT:

        /* Connection terminated. */
        ESP_LOGW(TAG, "Disconnect EVT; reason=%d ", event->disconnect.reason);
        
        ESP_LOGW(TAG, "Delete peer structure for conn_handle=%d",event->disconnect.conn.conn_handle);
        peer_delete(event->disconnect.conn.conn_handle);

        if (peer_get_NUM_CRU() == 0)
        {
            /* Reinitialize the whole peer structure */
            peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS) + 4, 64, 64, 64);

            if (m_CTU_task_param.state != CTU_LATCHING_FAULT_STATE ||
                m_CTU_task_param.state != CTU_LOCAL_FAULT_STATE ||
                m_CTU_task_param.state != NULL_STATE)
            {
                /* Goto the power save state */
                CTU_state_change(CTU_POWER_SAVE_STATE, NULL);
            }
        }
        break;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "Discovery complete EVT; reason=%d", event->disc_complete.reason);
        break;
    
    case BLE_GAP_EVENT_L2CAP_UPDATE_REQ:
        ESP_LOGI(TAG, "L2CAP update req EVT");
        break;
    
    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(TAG, "Connection param update req EVT");
        break;

    case BLE_GAP_EVENT_NOTIFY_RX:
        /* Peer sent us a notification or indication. */
      /*ESP_LOGI(TAG, "%s EVT; conn_handle=%d attr_handle=%d "
                    "attr_len=%d\n",
                    event->notify_rx.indication ?
                    "Indication" :
                    "Notification",
                    event->notify_rx.conn_handle,
                    event->notify_rx.attr_handle,
                    OS_MBUF_PKTLEN(event->notify_rx.om));*/

        peer = peer_find(event->notify_rx.conn_handle);
        if(peer->CRU)
        {
            ble_central_unpack_CRU_alert_param(event->notify_rx.om, event->notify_rx.conn_handle); 
        } else {
            ble_central_unpack_AUX_CTU_alert_param(event->notify_rx.om, event->notify_rx.conn_handle);
        }        
        break;   

    case BLE_GAP_EVENT_NOTIFY_TX:
        ESP_LOGI(TAG, "Notify TX");
        break;

    default:
        ESP_LOGW(TAG, "Other EVT of type %d", event->type);
        break;
    }
    if (m_CTU_task_param.state == CTU_POWER_TRANSFER_STATE)
    {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return 0;
}

/**********************************************************************************/
/**                       Payload function definitions                           **/
/**********************************************************************************/
static void ble_central_unpack_AUX_CTU_alert_param(struct os_mbuf* om, uint16_t conn_handle)
{
    //todo: receive switch on/off state from A-CTU
    struct peer *peer = peer_find(conn_handle);

    peer->alert_payload.alert_field.internal = om->om_data[0];

    if(peer->alert_payload.alert_field.overvoltage)
    {
        ESP_LOGW(TAG, "ALERT -- OVERVOLTAGE");
        CTU_state_change(CTU_LOCAL_FAULT_STATE, (void *)peer);
    } else if(peer->alert_payload.alert_field.overcurrent)
        {
            ESP_LOGW(TAG, "ALERT -- OVERCURRENT");
            CTU_state_change(CTU_LOCAL_FAULT_STATE, (void *)peer);
        } else if(peer->alert_payload.alert_field.overtemperature)
            {
                ESP_LOGW(TAG, "ALERT -- OVERTEMPERATURE");
                CTU_state_change(CTU_LOCAL_FAULT_STATE, (void *)peer);
            }

    vTaskDelay(50);
}


static void ble_central_unpack_CRU_alert_param(struct os_mbuf* om, uint16_t conn_handle)
{
    struct peer *peer = peer_find(conn_handle);

    peer->alert_payload.alert_field.internal = om->om_data[0];

    if(peer->alert_payload.alert_field.overvoltage)
    {
        ESP_LOGW(TAG, "ALERT -- OVERVOLTAGE");
        CTU_state_change(CTU_LATCHING_FAULT_STATE, (void *)peer);
    } else if(peer->alert_payload.alert_field.overcurrent)
        {
            ESP_LOGW(TAG, "ALERT -- OVERCURRENT");
            CTU_state_change(CTU_LATCHING_FAULT_STATE, (void *)peer);
        } else if(peer->alert_payload.alert_field.overtemperature)
            {
                ESP_LOGW(TAG, "ALERT -- OVERTEMPERATURE");
                CTU_state_change(CTU_LATCHING_FAULT_STATE, (void *)peer);
            }  else if (peer->alert_payload.alert_field.charge_complete || peer->alert_payload.alert_field.wired_charger)
                {
                    ESP_LOGW(TAG, "CHARGE COMPLETE / WIRED ALERT");
                    //Disable charge of relative pad
                    if(peer->position) {
                        struct peer *Aux_CTU = Aux_CTU_find(peer->position);
                        if(Aux_CTU != NULL) {
                            ble_central_update_control_enables(0, 0, Aux_CTU);    
                        }
                    }
                    if (is_peer_alone())
                    {
                        CTU_state_change(CTU_LOW_POWER_STATE, NULL);
                    }
                } 

    vTaskDelay(50);
}

/* Successivelly parse attribute data buffer to find appropriate values */
static void ble_central_unpack_static_param(const struct ble_gatt_attr *attr, uint16_t conn_handle)
{
    struct peer *peer = peer_find(conn_handle);

    /* Single byte values (bytes 1 through 8) */
    peer->stat_payload.optional_fields = attr->om->om_data[0];
    peer->stat_payload.protocol_rev = attr->om->om_data[1];
    peer->stat_payload.RFU1 = attr->om->om_data[2];
    peer->stat_payload.CRU_cat = attr->om->om_data[3];
    peer->stat_payload.CRU_info = attr->om->om_data[4];
    peer->stat_payload.hard_rev = attr->om->om_data[5];
    peer->stat_payload.firm_rev = attr->om->om_data[6];
    peer->stat_payload.prect_max = attr->om->om_data[7];

    /* Double byte values (bytes 9 through 16) */
    peer->stat_payload.vrect_min_stat = attr->om->om_data[8];
    peer->stat_payload.vrect_min_stat <<= 8;
    peer->stat_payload.vrect_min_stat += attr->om->om_data[9] & 0xF0;

    peer->stat_payload.vrect_high_stat = attr->om->om_data[10];
    peer->stat_payload.vrect_high_stat <<= 8;
    peer->stat_payload.vrect_high_stat += attr->om->om_data[11] & 0xF0;

    peer->stat_payload.vrect_set = attr->om->om_data[12];
    peer->stat_payload.vrect_set <<= 8;
    peer->stat_payload.vrect_set += attr->om->om_data[13] & 0xF0;
 
    peer->stat_payload.company_id = attr->om->om_data[14];
    peer->stat_payload.company_id <<= 8;
    peer->stat_payload.company_id += attr->om->om_data[15] & 0xF0;

    /* Quadruple byte value of RFU (bytes 17 through 20) */
    peer->stat_payload.RFU2 = attr->om->om_data[16];
    peer->stat_payload.RFU2 <<= 8;
    peer->stat_payload.RFU2 += attr->om->om_data[17] & 0xFFF0;
    peer->stat_payload.RFU2 <<= 8;
    peer->stat_payload.RFU2 += attr->om->om_data[18] & 0xFFF0;
    peer->stat_payload.RFU2 <<= 8;
    peer->stat_payload.RFU2 += attr->om->om_data[19] & 0xFFF0;
    //ESP_LOGI(TAG, "CRU STATIC PARAM: Vrect Min = %d    /    Vrect High = %d    /    Vrect Set = %d", 
    //         peer->stat_payload.vrect_min_stat, peer->stat_payload.vrect_high_stat, peer->stat_payload.vrect_set);
}

static void ble_central_unpack_dynamic_param(const struct ble_gatt_attr *attr, uint16_t conn_handle)
{
    struct peer *peer = peer_find(conn_handle);

    /* voltage i2c measurement */
    peer->dyn_payload.vrect.b[0] = attr->om->om_data[0];
    peer->dyn_payload.vrect.b[1] = attr->om->om_data[1];
    peer->dyn_payload.vrect.b[2] = attr->om->om_data[2];
    peer->dyn_payload.vrect.b[3] = attr->om->om_data[3];
    /* current i2c measurement */
    peer->dyn_payload.irect.b[0] = attr->om->om_data[4];
    peer->dyn_payload.irect.b[1] = attr->om->om_data[5];
    peer->dyn_payload.irect.b[2] = attr->om->om_data[6];
    peer->dyn_payload.irect.b[3] = attr->om->om_data[7];
    /* temperature i2c measurement */
    peer->dyn_payload.temp.b[0] = attr->om->om_data[8];
    peer->dyn_payload.temp.b[1] = attr->om->om_data[9];
    peer->dyn_payload.temp.b[2] = attr->om->om_data[10];
    peer->dyn_payload.temp.b[3] = attr->om->om_data[11];
    /* alert */
    peer->dyn_payload.alert = attr->om->om_data[12];
    /* reserved for future use */
    peer->dyn_payload.RFU = attr->om->om_data[13];

    //ESP_LOGW(TAG, "- DYN CHR - rx voltage = %.02f", peer->dyn_payload.vrect.f);
    //ESP_LOGW(TAG, "- DYN CHR - rx current = %.02f", peer->dyn_payload.irect.f);
    //ESP_LOGW(TAG, "- DYN CHR - rx temperature = %.02f", peer->dyn_payload.temp.f);
}

esp_err_t ble_central_get_CTU_static(void)
{
    esp_err_t err = ESP_OK;
    uint16_t data_16 = 0;
    uint32_t data_32 = 0;

    int iter = 0;

    /* Create new NVS handle */
    nvs_handle_t nvs_handle;
    err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } 
    else
    {
        for (int i=0; i!=N_CTU_PARAMS; i++)
        {
            if (!strcmp(ch_arr[i], "RFU 1") || !strcmp(ch_arr[i], "Company ID"))
            {
                err = nvs_get_u16(nvs_handle, ch_arr[i], &data_16);
                CTU_static_data[i] = data_16;
                CTU_static_data[i+1] = data_16 >> 8;
                iter++;
            }
            else if (!strcmp(ch_arr[i], "RFU 2"))
            {
                err = nvs_get_u32(nvs_handle, ch_arr[i], &data_32);
                CTU_static_data[i+iter] = data_32;
                iter++;
                CTU_static_data[i+iter] = data_32 >> 8;
                iter++;
                CTU_static_data[i+iter] = data_32 >> 16;
                iter++;
                CTU_static_data[i+iter] = data_32 >> 24;
                iter++;
            }
            else
            {
                err = nvs_get_u8(nvs_handle, ch_arr[i], &CTU_static_data[i]);
            }
            if (err != ESP_OK)
            {
                printf("Error (%s) reading (%s)!\n", esp_err_to_name(err), ch_arr[i]);
                return err;
            }
        }
    }
    return err;
}

void ble_central_update_control_enables(uint8_t enable, uint8_t full_power, struct peer *peer)
{
    int rc;
    uint8_t value[4];
    const struct peer_chr *control_chr;

    /* Get attribute value of CRU control characteristic */
    control_chr = peer_chr_find_uuid(peer,
                        wpt_svc_uuid,
                        wpt_char_control_uuid);
    if (control_chr == NULL)
    {
        return;
    }
        
    peer->contr_payload.enable = enable;
    peer->contr_payload.full_power = full_power;
    peer->contr_payload.RFU = 0;

    value[0] = enable;
    value[1] = full_power;
    value[2] = 0;
    value[3] = 0;

    rc = ble_gattc_write_no_rsp_flat(peer->conn_handle, control_chr->chr.val_handle,
                              (void *)value, sizeof value);
    
    //ENABLE OR DISABLE CHARGING 

    if (enable == 1)
    {
        //enabled[(peer->position)-1] = 1;
        ESP_LOGW(TAG, "Enable charge on pad=%d", peer->position);
    }
    else if(enable == 0)
    {
        //enabled[(peer->position)-1] = 0;
        ESP_LOGW(TAG, "Disable charge on pad=%d!", peer->position);
    }

    if (rc > 0)
    {
        ESP_LOGE(TAG, "Unable to write CRU Control reason=%d", rc);
    }
}