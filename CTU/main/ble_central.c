#include "include/ble_central.h"
#include "host/ble_att.h"
#include "nvs_flash.h"
#include "nvs.h"

/**********************************************************************************/
/**                            Restricted globals                                **/
/**********************************************************************************/

/* Defines what type of task the unit will run on */
#define TASKS_CORE            1
#define TASK_STACK_SIZE       10000
#define CRU_TASK_PRIORITY     20
#define AUX_CTU_TASK_PRIORITY 20

//timer
struct timeval tv_loc;
struct timeval tv_stop;
struct timeval tv_low_on;
struct timeval tv_switch;
float time_sec, time_lowpower_on;

//varaible to pass between the aux ctu processes for sequential switching during the localization process
uint8_t baton = 1;
//variable to know which pad is actually on in Low Power mode
uint8_t current_low_power = 0;

/* keep count of comms_error */
uint8_t cru_comms_error = 0;
uint8_t ctu_comms_error = 0;

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
    .supervision_timeout = 0x00f0,
    .min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN,
    .max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN,
};

// Declare ALL the WPT service and its characteristics UUIDs 

const ble_uuid_t *wpt_svc_uuid = 
    BLE_UUID128_DECLARE(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0xFE, 0xFF, 0x55, 0x64);

const ble_uuid_t *wpt_char_control_uuid = 
    BLE_UUID128_DECLARE(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x70, 0xE6, 0x55, 0x64);

//const ble_uuid_t *wpt_char_CTU_stat_uuid = 
//    BLE_UUID128_DECLARE(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x71, 0xE6, 0x55, 0x64);

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

    ESP_LOGW(TAG, "Killing AUX CTU");

    //disable realtive interface
    if(Aux_CTU != NULL) {
        if (Aux_CTU->position)
        {
            ble_central_update_control_enables(0, 1, 0, Aux_CTU);         
            ble_central_update_control_enables(0, 0, 0, Aux_CTU); 
        }
        low_power_pads[Aux_CTU->position-1] = 0;
        full_power_pads[Aux_CTU->position-1] = 0;
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        // use this to avoid reconnection
        // ble_gap_terminate(conn_handle, BLE_HS_EAPP);
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

    ESP_LOGW(TAG, "Killing CRU");
    
    if (peer!=NULL)
    {
        //IF WAS DOING THE LOCALIZATION PROCESS
        if(peer->localization_process)
        {
            uint8_t n = loc_pad_find();
            low_power_pads[n] = 0;
            struct peer *Aux_CTU = Aux_CTU_find(n+1);
            if(Aux_CTU != NULL) {
                ble_central_update_control_enables(0, 0, 0, Aux_CTU);
            }
            peer->localization_process = false;
        }
        //IF IT WAS BEING CHARGED IN FULL-POWER
        if(peer->position)
        {
            full_power_pads[peer->position-1] = 0;
            struct peer *Aux_CTU = Aux_CTU_find(peer->position);
            if(Aux_CTU != NULL) {
                ble_central_update_control_enables(0, 1, 0, Aux_CTU);
            }
            strip_enable[peer->position-1] = true;
            peer->position = 0;
        }
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        // use this to avoid reconnection
        // ble_gap_terminate(conn_handle, BLE_HS_EAPP);
    }
}

/** 
 * @brief Kill ALL cru task currently running on CPU1
 * 
 */
void ble_central_kill_all_CRU(void)
{
    for (uint16_t conn_handle=0; conn_handle!=MYNEWT_VAL(BLE_MAX_CONNECTIONS); conn_handle++)
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
    for (uint16_t conn_handle=0; conn_handle!=MYNEWT_VAL(BLE_MAX_CONNECTIONS); conn_handle++)
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
    int rc = 1;

    // Give the semaphore immediately 
    xSemaphoreGive(peer->sem_handle);

    if (error->status != 0)
    {
        ESP_LOGE(TAG, "Unable to read (ON LOCALIZATION PROCESS): status=%d", error->status);
        return 1;
    }

    // Unpack CRU Dynamic Characteristic
    ble_central_unpack_dynamic_param(attr, conn_handle);

    // DETECT POSITION OF THE CRU USING THE DYN CHR 
    // DO NOT GO TO POWER TRANSFER STATE UNTIL A POSITION IS DETECTED
    // is Vrect value above the Treshold?
    if (peer->dyn_payload.vrect.f > VOLTAGE_LOW_THRESH ) //todo: this should be redundant
    {
        ESP_LOGE(TAG, " VOLTAGE TRESHOLD PASSED!");
        //change the peer variable to end the localization process
        peer->localization_process = false;        
        
        struct peer *Aux_CTU = Aux_CTU_find(current_low_power);
        if ((Aux_CTU == NULL) && (peer->conn_handle))
        {
            ESP_LOGE(TAG, "No AUX CTU found in position %d", current_low_power);
            return 1;
        }

        //!CRITICAL
        rc = ble_central_update_control_enables(1, 1, 1, Aux_CTU);
        //make sure the command goes through
        if (!rc)
        {
            low_power_pads[3] = low_power_pads[2] = low_power_pads[1] = low_power_pads[0] = 0;
            full_power_pads[Aux_CTU->position-1] = 1;
            //SET RX POSITION
            peer->position = current_low_power;
            ESP_LOGI(TAG, " CRU is in position %d", peer->position );
            //exit the led default state
            strip_enable[peer->position-1] = false;
            //store time
            gettimeofday(&tv_loc, NULL);      
            if (m_CTU_task_param.state == CTU_LOW_POWER_STATE)
                CTU_state_change(CTU_POWER_TRANSFER_STATE, (void *)peer);
        } else
        {
            ESP_LOGE(TAG, "error sending the critical command");
        }
    }

    return rc;
}

/** 
 * @brief Dynamic read callback function for Auxiliary CTU 
 * @details Application callback.  Called when the read of the CRU Dynamic
 *          Parameter characteristic has completed.
 *          This reads the I2C measurements and switches on/off the pads when switch_loc_pads[] changes writing the Control characteristic.
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
    int rc = 0;


    // Give the semaphore immediately 
    xSemaphoreGive(peer->sem_handle);
    
    // If the characteristic has not been read correctly
    if (error->status != 0)
    {
        ESP_LOGE(TAG, "Unable to read (ON AUX CTU DYN): status=%d", error->status);
        return 1;
    }
   
    // Unpack peer Static Characteristic
    ble_central_unpack_dynamic_param(attr, conn_handle);


    //SEND SWITCH COMMAND THROUGH CONTROL CHR DURING A LOCALIZATION PROCESS
    //LOW POWER mode
    //double check if full_power is on
    //all_low_power_off check to make sure first it switches off and then on


    if (current_localization_process())
    {
        //SWITCH ON
        if (all_low_power_off())
        {
            if (baton == peer->position)
            {
                if(!full_power_pads[baton-1])
                {
                    rc = ble_central_update_control_enables(1, 0, 0, peer);
                    if (!rc)
                    {
                        current_low_power = baton;
                        gettimeofday(&tv_low_on, NULL);
                        if (baton == 4)
                            baton = 1;
                        else    
                            baton++;
                    }
                } else 
                {
                    if (baton == 4)
                        baton = 1;
                    else    
                        baton++;
                }
            
            }
        } else
        {
            //SWITCH OFF
            if (peer->position == loc_pad_find()+1)
            {
                gettimeofday(&tv_switch, NULL);
                time_lowpower_on = tv_switch.tv_sec - tv_low_on.tv_sec + 1e-6f * (tv_switch.tv_usec - tv_low_on.tv_usec);
                if (time_lowpower_on > MIN_LOW_POWER_ON)
                {
                    ESP_LOGE(TAG, "Time LOW POWER ON %.02f", time_lowpower_on);
                    rc = ble_central_update_control_enables(0, 0, 0, peer);
                    if (rc)
                        low_power_pads[peer->position-1] = 1;
                }
            }
        }
    }
    
    return rc;
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
    xSemaphoreGive(peer->sem_handle);
    
    // If the characteristic has not been read correctly
    if (error->status != 0)
    {
        //todo: add error handling here
        ESP_LOGE(TAG, "Unable to read (ON CRU DYN): status=%d", error->status);
        return 1;
    }
   
    // Unpack peer Static Characteristic
    ble_central_unpack_dynamic_param(attr, conn_handle);


    //todo: only when it changes (keep the state in memory)
    //MISALIGNMENT CHECK
    if(peer->dyn_payload.vrect.f < VOLTAGE_MIS_THRESH )
        set_strip(peer->position, 255, 128, 0);
    else 
        set_strip(peer->position, 0, 255, 0);  
     

    struct peer *Aux_CTU = Aux_CTU_find(peer->position);
    if (Aux_CTU == NULL)
    {
        ESP_LOGE(TAG, "No AUX CTU found in position %d", peer->position);
        return 1;
    }

    time(&now);
    localtime_r(&now, &info);
    ESP_LOGI(TAG, "Time is %s", asctime(&info));

    //todo: attach timestamps to create a reliable value
    //GET EFFICIENCY
    float efficiency = (peer->dyn_payload.irect.f * peer->dyn_payload.vrect.f) / (Aux_CTU->dyn_payload.irect.f * Aux_CTU->dyn_payload.vrect.f);
    if ((efficiency < 1) && (efficiency > 0))
        ESP_LOGI(TAG, "Efficiency on position %d = %.02f", peer->position, efficiency);

    ESP_LOGI(TAG, "Vtx = %.02f, Itx = %.02f", Aux_CTU->dyn_payload.vrect.f, Aux_CTU->dyn_payload.irect.f);
    ESP_LOGI(TAG, "Vrx = %.02f, Irx = %.02f", peer->dyn_payload.vrect.f, peer->dyn_payload.irect.f);

    // Check the charge has actually started (during the first 3 seconds) -- kind of double check on the localization algorithm
    gettimeofday(&tv_stop, NULL);
    time_sec = tv_stop.tv_sec - tv_loc.tv_sec + 1e-6f * (tv_stop.tv_usec - tv_loc.tv_usec);
    if ( time_sec < BATTERY_REACTION_TIME )
    {
        if(peer->dyn_payload.vrect.f > VOLTAGE_FULL_THRESH)
        {
            peer->correct = true;
        }
    } 

    if ((!peer->correct) && (time_sec > BATTERY_REACTION_TIME ))
    {
        ESP_LOGE(TAG, "Voltage not received during the first 10 seconds!");
        ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        return 1;
    }

    //DOUBLE CHECK THE IT IS STILL RECEIVING VOLTAGE
    if ((peer->dyn_payload.vrect.f < VOLTAGE_FULL_THRESH) && (time_sec > BATTERY_REACTION_TIME))
    {
        peer->correct = false;
        ESP_LOGE(TAG, "Voltage no longer received! --> SCOOTER LEFT THE PLATFORM");
        ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        return 1;
    }

    return 0;
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

    ESP_LOGE(TAG, "ALERT!");
    
    // Give the semaphore immediately 
    xSemaphoreGive(peer->sem_handle);
    
    // If the characteristic has not been read correctly
    if (error->status != 0)
    {
        ESP_LOGE(TAG, "Unable to read (ON CRU ALERT): status=%d", error->status);
        return 1;
    }
   
    return 0;
}

/** 
 * @brief AUXILIARY CTU WPT service task handle function
 * @details Contains the loop over which all A-CTU service tasks will run. It will mainly read
 *          the Dynamic characteristic (containing I2C measurements) while also making sure the CRU is still
 *          connected. If the CRU disconnects or encounters any issues the task will
 *          end, the semaphore will be deleted and the connection will be ended.
 *          Additionally, it will switch on/off the pads when switch_loc_pads[] changes writing the Control characteristic.
 * 
 * @param arg A void pointer which contains the peer structure
 */
static void ble_central_AUX_CTU_task_handle(void *arg)
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
        ESP_LOGE(TAG, "Error: Failed to read dynamic characteristic");
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
    if (err_code)
    {
        ESP_LOGE(TAG, "AUX CTU task could not be added!");
        ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        return;
    }

    peer->last_rc = -1;
    int task_delay = CTU_TIMER_PERIOD;
    
    /* WPT task loop */
    while (1)
    {
        //check alert is fine
        if (peer->dyn_payload.alert == 0)
        {
            if (xSemaphoreTake(peer->sem_handle, task_delay) == pdTRUE)
            {
                /* Initiate Read procedure */        
                rc = ble_gattc_read(peer->conn_handle, dynamic_chr->chr.val_handle,
                                ble_central_on_AUX_CTU_dyn_read, (void *)peer);
                if(rc)
                    ESP_LOGE(TAG, "error");
            }
        } else 
            {
                if (xSemaphoreTake(peer->sem_handle, task_delay) == pdTRUE)
                {
                    rc = ble_gattc_read(peer->conn_handle, alert_chr->chr.val_handle,
                                    ble_central_on_alert_read, (void *)peer);
                }
            }

        //*ERROR HANDLING - DISCONNECT ONLY AFTER 60 CONSECUTIVE COMMS ERRORS        
        if (rc != ESP_OK)
        {
            if (peer->last_rc == rc)
            {
                if (ctu_comms_error < COMMS_ERROR_LIMIT)
                {
                    ctu_comms_error++;
                } else {
                    ESP_LOGW(TAG, "AUX CTU 60th consecutive task error - disconnect");
                    ctu_comms_error = 0;
                    /* Delete watchdog */
                    esp_task_wdt_delete(NULL);
                    ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, peer->conn_handle);         
                }
            } 
            ESP_LOGW(TAG, "AUX CTU in position %d, task error number %d, error code rc=%d", peer->position, ctu_comms_error, rc);
            //RC = https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_err.html#_CPPv49esp_err_t
            // Give the semaphore immediately 
            xSemaphoreGive(peer->sem_handle);
        } else if ((peer->last_rc != -1) && (peer->last_rc))
        {
            ESP_LOGI(TAG, "Comms RECOVERY with AUX CTU %d", peer->position);
            ctu_comms_error = 0;
        }
        
        peer->last_rc = rc;

        // set Task Delay
        if (current_localization_process())
            task_delay = LOC_CTU_TIMER_PERIOD;
        else 
            task_delay = CTU_TIMER_PERIOD;

        /* Feed watchdog */
        esp_task_wdt_reset();

        vTaskDelay(task_delay);
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

    if (peer==NULL)
    {
        return;
    }

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
    if(err_code)
    {
        ESP_LOGE(TAG, "CRU task could not be added!");
        ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle); 
        return;
    }

    //FIRST LOCALIZATION PROCESS --> only one a time! need global variable
    //enable power of one pad 
    //wait a reasonable time to see it in the rx
    //cycle through pads to see if which CRU has the Vrect value above the Treshold
    //ENSURE only one localization process per time!
    //discard pad which are already on!
    //check alert is fine
    //need a timeout for when the localization is not detected (probably another rx is in the process)
    //NO POWER STATE UNTIL THE POSITION IS DETECTED

    peer->localization_process = false;
    peer->correct = false;
    peer->count = 0;
    peer->last_rc = -1;

    int task_delay = CRU_TIMER_PERIOD;
    
    /* WPT task loop */
    while (1)
    {
        //*LOCALIZATION PROCESS (find position of CRU)
        //check alert is fine
        if (peer->dyn_payload.alert == 0)
        {
            // ENSURE only one localization process per time!
            if ((peer->position == 0) && ((!current_localization_process()) || (peer->localization_process)))
            {
                switch (peer->count)
                {
                    case 0:
                        current_low_power = 0;
                        peer->localization_process = true;
                        peer->count++; 
                        break;
                    
                    //TIMEOUT FOR DETECTING LOCALIZATION (100*75ms) -- 7.5s 
                    case 100:
                        ESP_LOGE(TAG, "Localization timer expires! Disconnecting");
                        ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);
                        break;

                    // read dyn chr
                    default:
                        if (xSemaphoreTake(peer->sem_handle, task_delay) == pdTRUE)
                        {
                            peer->count++;
                            rc = ble_gattc_read(peer->conn_handle, dynamic_chr->chr.val_handle,
                                ble_central_on_localization_process, (void *)peer);
                        }
                        break;
                }
            } else if (peer->position)
            {   
                //* POWER TRANSFER PROCESS (keep reading dyn chr)
                if (xSemaphoreTake(peer->sem_handle, task_delay) == pdTRUE)
                {
                    /* Initiate Read procedure */        
                    rc = ble_gattc_read(peer->conn_handle, dynamic_chr->chr.val_handle,
                                    ble_central_on_CRU_dyn_read, (void *)peer);
                }
            }
        } else 
            {
                if (xSemaphoreTake(peer->sem_handle, task_delay) == pdTRUE)
                {
                    rc = ble_gattc_read(peer->conn_handle, alert_chr->chr.val_handle,
                                    ble_central_on_alert_read, (void *)peer);
                }
            }
        
        //*ERROR HANDLING - DISCONNECT ONLY AFTER 60 CONSECUTIVE COMMS ERRORS
        if (rc != ESP_OK)
        {
            if (peer->last_rc == rc)
            {
                if (cru_comms_error < COMMS_ERROR_LIMIT)
                {
                    cru_comms_error++;
                } else {
                    ESP_LOGW(TAG, "CRU 60th  consecutive task error - disconnect"); 
                    cru_comms_error = 0;
                    /* Delete watchdog */
                    esp_task_wdt_delete(NULL);
                    ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);         
                }
            } 
            ESP_LOGW(TAG, "CRU %d task error but loop anyway rc=%d", cru_comms_error, rc);
            //RC = https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_err.html#_CPPv49esp_err_t
            xSemaphoreGive(peer->sem_handle);
        } else if ((peer->last_rc != -1) && (peer->last_rc))
        {
            ESP_LOGI(TAG, "Comms RECOVERY with CRU");
            cru_comms_error = 0;
        }

        peer->last_rc = rc;

        // set Task Delay
        if (current_localization_process())
            task_delay = LOC_CRU_TIMER_PERIOD;
        else 
            task_delay = CRU_TIMER_PERIOD;

        /* Feed watchdog */
        esp_task_wdt_reset();

        vTaskDelay(task_delay);
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

    if(peer->CRU)
    {
        //initialize peer position as not detected
        peer->position = 0;

        char task_name[] = "CRU";
        char num = conn_handle + '0';
        strncat(task_name, &num, 1);

        /* Creates one CRU task for every success and completes connection */
        err_code = xTaskCreatePinnedToCore((void *)ble_central_CRU_task_handle,
                        task_name, TASK_STACK_SIZE, (void *)peer, CRU_TASK_PRIORITY,
                        &task_handle, TASKS_CORE);
        if ((err_code != pdPASS))
        {
            ESP_LOGE(TAG, "ERROR creating CRU task!");
            ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle); 
        }
    }
    else {
        char task_name[] = "AUX_CTU";
        char num = conn_handle + '0';
        strncat(task_name, &num, 1);

        /* Creates one AUX CTU task for every success and completes connection */
        err_code = xTaskCreatePinnedToCore((void *)ble_central_AUX_CTU_task_handle,
                        task_name, TASK_STACK_SIZE, (void *)peer, AUX_CTU_TASK_PRIORITY,
                        &task_handle, TASKS_CORE);
        if ((err_code != pdPASS))
        {
            ESP_LOGE(TAG, "ERROR creating AUX CTU task!");
            ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, peer->conn_handle); 
        } 
    }

    /* Adds the task handle to the peer structure to allow modifications in runtime */
    peer->task_handle = task_handle;
    peer->sem_handle = sem_handle;

    /* XXX DO NOT REMOVE OR IT BREAKS */
    xSemaphoreGive(peer->sem_handle);

    return 1;
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
                ESP_LOGE(TAG, "CONTROL chr not found!");
                ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);      
            }
            /* Enter low power state only when the A-CTUs are connected and their position have been found*/
            if ((peer_get_NUM_AUX_CTU() == 4) && (m_CTU_task_param.state == CTU_CONFIG_STATE))
            {
                CTU_state_change(CTU_LOW_POWER_STATE, (void *)NULL);
            }
        }
        
        // Unpack CRU Dynamic Characteristic
        ble_central_unpack_dynamic_param(attr, conn_handle);

        //START THE WPT TASK
        ble_central_wpt_start(conn_handle, (void *)peer);
    }
    else
    {
        ESP_LOGE(TAG, "on_control_enable");
        if(peer->CRU)
        {
            ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        }
        else {
            ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        }
    }
    return 0;
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

    const struct peer_chr *static_chr;
    int rc;

    if ((status != 0) || (peer == NULL))
    {
        goto err;
    }

    // Read the CRU Static parameter characteristic.
    static_chr = peer_chr_find_uuid(peer,
                             wpt_svc_uuid,
                             wpt_char_CRU_stat_uuid);
                             
    if (static_chr == NULL)
    {
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
        if(peer->CRU)
        {
            ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        }
        else {
            ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        }
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

    if (peer==NULL)
    {
        goto err;
    }

    // Get chr value of CRU dynamic 
    dynamic_chr = peer_chr_find_uuid(peer,
                        wpt_svc_uuid,
                        wpt_char_CRU_dyn_uuid);
    if (dynamic_chr == NULL)
    {
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
        if(peer->CRU)
        {
            ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        }
        else {
            ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        }

    return 1;
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

    if (peer==NULL)
    {
        goto err;
    }

    dsc = peer_dsc_find_uuid(peer,
                            wpt_svc_uuid,
                            wpt_char_alert_uuid,
                            BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16));
    if (dsc == NULL)
    {
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

    ble_central_on_subscribe(conn_handle, (void *)peer);

    return 0;

    err:
        ESP_LOGW(TAG, "on_write_cccd");
       if(peer->CRU)
        {
            ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        }
        else {
            ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        }

    return 1;
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

    if (error->status == 0)
    {
        // Unpack CRU Static Characteristic
        ble_central_unpack_static_param(attr, conn_handle);
    }
    else
    {
        goto err;
    }

    ble_central_on_write_cccd(conn_handle, (void*)peer);

    return 0;

    err:
        ESP_LOGW(TAG, "on_CRU_static_read");
        if(peer->CRU)
        {
            ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        }
        else {
            ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, peer->conn_handle);
        }
    
    return 1;
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

    //todo: improve the scanning - faster

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
    //todo: try to remove this?
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
        return 0;
    }

   // The device has to advertise support for the WPT service (0xFFFE).

   for (int i = 0; i < fields.num_uuids16; i++) {
        if (ble_uuid_u16(&fields.uuids16[i].u) == WPT_SVC_UUID16) {
            if(fields.appearance == 1) {
                return 1;
            } /* ALLOW CONNECTION TO CRU ONLY AFTER AT LEAST ONE SUCCESSFULL CONNECTION TO A-CTU */
            else if((fields.appearance == 2) && (peer_get_NUM_AUX_CTU() > 3)) {
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

    /* Figure out address to use for connect (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Try to connect the advertiser.  Allow 1000 ms for
     * timeout.
     */
    rc = ble_gap_connect(own_addr_type, &disc->addr, 1000, &conn_params,
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
        ESP_LOGE(TAG, "no you can't");
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
            //SET MAX TX POWER
            esp_ble_tx_power_set(event->connect.conn_handle, ESP_PWR_LVL_P9); 
            
            time(&now);
            localtime_r(&now, &info);
            ESP_LOGI(TAG, "Time is %s", asctime(&info));
            
            const uint8_t* slave_type = (const uint8_t *)arg;

            //convert pointer to integer
            uintptr_t type = (uintptr_t) slave_type;

            if(type == 1) {
                //ESP_LOGI(TAG, "Connection EVT with AUX CTU");
                /* Adds AUX CTU to list of connections */
                rc = peer_add(event->connect.conn_handle, 0);
            } else if (type == 2) {
                ESP_LOGI(TAG, "Connection EVT with CRU");
                /* Adds CRU to list of connections */
                rc = peer_add(event->connect.conn_handle, 1);
            }

            peer = peer_find(event->connect.conn_handle);

            if (rc != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to add peer; rc=%d\n", rc);
                if(type == 1) {
                    ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, peer->conn_handle);
                } else if (type == 2) {
                    ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);
                }
                break;
            }

            /* Perform service discovery. */
            rc = peer_disc_all(event->connect.conn_handle,
                               ble_central_on_disc_complete, NULL);
            if (rc != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to discover services; rc=%d\n", rc);
                if(type == 1) {
                    ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, peer->conn_handle);
                } else if (type == 2) {
                    ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);
                }
            }
            break;
        }
        else 
        {
            /* Connection attempt failed; resume scanning. */
            ESP_LOGE(TAG, "Error: Connection failed; status=%d\n",
                           event->connect.status);
            break;
        }

    case BLE_GAP_EVENT_DISCONNECT:
        
        time(&now);
        localtime_r(&now, &info);
        ESP_LOGI(TAG, "Time is %s", asctime(&info));

        /* Connection terminated. */
        ESP_LOGW(TAG, "Disconnect EVT; reason=%d ", event->disconnect.reason);
        //ESP_LOGW(TAG, "Delete peer structure for conn_handle=%d",event->disconnect.conn.conn_handle);

        peer = peer_find(event->disconnect.conn.conn_handle);

        if(peer->CRU)
        {
            ESP_LOGE(TAG, "CRU");
            ble_central_kill_CRU(peer->task_handle, peer->sem_handle, event->disconnect.conn.conn_handle);
        } else {
            if (peer->position) 
                {
                    ESP_LOGE(TAG, "A-CTU in position: %d", peer->position);
                    strip_enable[peer->position-1] = false;
                    set_strip(peer->position, 244, 244, 244);
                }
        }

        peer_delete(event->disconnect.conn.conn_handle);

        if ((peer_get_NUM_AUX_CTU() != 4) && (m_CTU_task_param.state != CTU_CONFIG_STATE))
        {
            //restart configuration
            ble_central_kill_all_AUX_CTU();
            CTU_state_change(CTU_CONFIG_STATE, (void *)peer);
        }
    
        if ((peer_get_NUM_AUX_CTU() + peer_get_NUM_CRU()) == 0)
        {
            /* Reinitialize the whole peer structure */
            peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
        }
        break;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        //ESP_LOGI(TAG, "Discovery complete EVT; reason=%d", event->disc_complete.reason);
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
    return 0;
}

/**********************************************************************************/
/**                       Payload function definitions                           **/
/**********************************************************************************/
//todo: fault state: tv_fault --> save on each peer the time to wait before reconnection (it should be kept in memory even after disconnection)
static void ble_central_unpack_AUX_CTU_alert_param(struct os_mbuf* om, uint16_t conn_handle)
{
    time(&now);
    localtime_r(&now, &info);
    ESP_LOGI(TAG, "Time is %s", asctime(&info));

    struct peer *peer = peer_find(conn_handle);

    if (peer!=NULL)
    {
        //reset the alert field in the dynamic payload
        peer->dyn_payload.alert = 0;
        //read the alert payload
        peer->alert_payload.alert_field.internal = om->om_data[0];

        if(peer->alert_payload.alert_field.overvoltage)
        {
            ESP_LOGE(TAG, "ALERT -- OVERVOLTAGE -- aux ctu position %d", peer->position);
            peer->alert_payload.alert_field.overvoltage = 0;
            CTU_state_change(CTU_LOCAL_FAULT_STATE, (void *)peer);
        } else if(peer->alert_payload.alert_field.overcurrent)
            {
                ESP_LOGE(TAG, "ALERT -- OVERCURRENT -- aux ctu position %d", peer->position);
                peer->alert_payload.alert_field.overcurrent = 0;
                CTU_state_change(CTU_LOCAL_FAULT_STATE, (void *)peer);
            } else if(peer->alert_payload.alert_field.overtemperature)
                {
                    ESP_LOGE(TAG, "ALERT -- OVERTEMPERATURE -- aux ctu position %d", peer->position);
                    peer->alert_payload.alert_field.overtemperature = 0;
                    CTU_state_change(CTU_LOCAL_FAULT_STATE, (void *)peer);
                } else if(peer->alert_payload.alert_field.FOD)
                {
                    ESP_LOGE(TAG, "ALERT -- FOD -- aux ctu position %d", peer->position);
                    peer->alert_payload.alert_field.FOD = 0;
                    CTU_state_change(CTU_LOCAL_FAULT_STATE, (void *)peer);
                } 
    }
}

static void ble_central_unpack_CRU_alert_param(struct os_mbuf* om, uint16_t conn_handle)
{
    struct peer *peer = peer_find(conn_handle);

    time(&now);
    localtime_r(&now, &peer->alert_payload.alert_time);

    if (peer!=NULL)
    {
        //reset the alert field in the dynamic payload
        peer->dyn_payload.alert = 0;
        //read the alert payload
        peer->alert_payload.alert_field.internal = om->om_data[0];

        if(peer->alert_payload.alert_field.overvoltage)
        {
            ESP_LOGE(TAG, "ALERT -- OVERVOLTAGE -- cru");
            CTU_state_change(CTU_REMOTE_FAULT_STATE, (void *)peer);
        } else if(peer->alert_payload.alert_field.overcurrent)
            {
                ESP_LOGE(TAG, "ALERT -- OVERCURRENT -- cru");
                CTU_state_change(CTU_REMOTE_FAULT_STATE, (void *)peer);
            } else if(peer->alert_payload.alert_field.overtemperature)
                {
                    ESP_LOGE(TAG, "ALERT -- OVERTEMPERATURE -- cru");
                    CTU_state_change(CTU_REMOTE_FAULT_STATE, (void *)peer);
                }  else if (peer->alert_payload.alert_field.charge_complete)
                    {
                        ESP_LOGE(TAG, "ALERT -- CHARGE COMPLETE  -- cru");
                        //Disconnect and wait for a while
                        CTU_state_change(CTU_REMOTE_FAULT_STATE, (void *)peer);
                        if ((!CTU_is_charging()) && (m_CTU_task_param.state != CTU_LOW_POWER_STATE))
                        {
                            CTU_state_change(CTU_LOW_POWER_STATE, (void *)peer);
                        }
                    }   
    }
}

/* Successivelly parse attribute data buffer to find appropriate values */
static void ble_central_unpack_static_param(const struct ble_gatt_attr *attr, uint16_t conn_handle)
{
    struct peer *peer = peer_find(conn_handle);

    if (peer!=NULL)
    {
        peer->stat_payload.mac_0 = attr->om->om_data[0];
        peer->stat_payload.mac_1 = attr->om->om_data[1];
        peer->stat_payload.mac_2 = attr->om->om_data[2];
        peer->stat_payload.mac_3 = attr->om->om_data[3];
        peer->stat_payload.mac_4 = attr->om->om_data[4];
        peer->stat_payload.mac_5 = attr->om->om_data[5];
    }

    //ASSIGN CORRECT POSITION OF A-CTU USING THEIR MAC ADDRESSES
    if(!peer->CRU)
    {
        if ((peer->stat_payload.mac_5 == 0x70) && (peer->stat_payload.mac_4 == 0x81) && (peer->stat_payload.mac_3 == 0x24) 
            && (peer->stat_payload.mac_2 == 0xfb) && (peer->stat_payload.mac_1 == 0x0b) && (peer->stat_payload.mac_0 == 0xac))
        {
            peer->position = 1;

        } else if ((peer->stat_payload.mac_5 == 0x84) && (peer->stat_payload.mac_4 == 0x68) && (peer->stat_payload.mac_3 == 0x24) 
                   && (peer->stat_payload.mac_2 == 0xfb) && (peer->stat_payload.mac_1 == 0x0b) && (peer->stat_payload.mac_0 == 0xac))
            {
                peer->position = 2;

            } else if ((peer->stat_payload.mac_5 == 0x44) && (peer->stat_payload.mac_4 == 0x0b) && (peer->stat_payload.mac_3 == 0x27) 
                       && (peer->stat_payload.mac_2 == 0xfb) && (peer->stat_payload.mac_1 == 0x0b) && (peer->stat_payload.mac_0 == 0xac))
                {
                    peer->position = 3;

                } else if ((peer->stat_payload.mac_5 == 0x98) && (peer->stat_payload.mac_4 == 0xbb) && (peer->stat_payload.mac_3 == 0x25) 
                           && (peer->stat_payload.mac_2 == 0xfb) && (peer->stat_payload.mac_1 == 0x0b) && (peer->stat_payload.mac_0 == 0xac))
                    {
                        peer->position = 4;
                
                    } else {
                            peer->position = 0;
                            ESP_LOGE(TAG, "ERROR");
                            }
        //enable led default state
        strip_enable[peer->position-1] = true;
        ESP_LOGI(TAG, "AUX-CTU POSITION = %d", peer->position);
    } else //SAVE CRU VOI CODE FOR IDENTIFICATION (use SD card to write times before reconnection) OR assumption that the MASTER never resets
    {
        if ((peer->stat_payload.mac_5 == 0xac) && (peer->stat_payload.mac_4 == 0x0b) && (peer->stat_payload.mac_3 == 0xfb) 
            && (peer->stat_payload.mac_2 == 0x25) && (peer->stat_payload.mac_1 == 0x63) && (peer->stat_payload.mac_0 == 0x5a))
        { 
            strcpy(peer->voi_code, "CE8J");

        } else if ((peer->stat_payload.mac_5 == 0xac) && (peer->stat_payload.mac_4 == 0x0b) && (peer->stat_payload.mac_3 == 0xfb) 
                   && (peer->stat_payload.mac_2 == 0x25) && (peer->stat_payload.mac_1 == 0x98) && (peer->stat_payload.mac_0 == 0x16))
            {
                strcpy(peer->voi_code, "6F35");

            }
        
    }
}

static void ble_central_unpack_dynamic_param(const struct ble_gatt_attr *attr, uint16_t conn_handle)
{
    struct peer *peer = peer_find(conn_handle);

    if ((peer!=NULL) && (attr!=NULL))
    { 
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
        peer->dyn_payload.temp1.b[0] = attr->om->om_data[8];
        peer->dyn_payload.temp1.b[1] = attr->om->om_data[9];
        peer->dyn_payload.temp1.b[2] = attr->om->om_data[10];
        peer->dyn_payload.temp1.b[3] = attr->om->om_data[11];
        peer->dyn_payload.temp2.b[0] = attr->om->om_data[12];
        peer->dyn_payload.temp2.b[1] = attr->om->om_data[13];
        peer->dyn_payload.temp2.b[2] = attr->om->om_data[14];
        peer->dyn_payload.temp2.b[3] = attr->om->om_data[15];
        /* alert */
        peer->dyn_payload.alert = attr->om->om_data[16];
        /* reserved for future use */
        peer->dyn_payload.RFU = attr->om->om_data[17];

        time(&now);
        localtime_r(&now, &peer->dyn_payload.dyn_time);

        //stamp the time
        //ESP_LOGE(TAG, "Time is %s", asctime(&peer->dyn_payload.dyn_time));


/*
        if(!peer->CRU)
        {
            ESP_LOGW(TAG, " pad number - %d", peer->position);
            ESP_LOGW(TAG, "- DYN CHR - rx voltage = %.02f", peer->dyn_payload.vrect.f);
            ESP_LOGW(TAG, "- DYN CHR - rx current = %.02f", peer->dyn_payload.irect.f);
            ESP_LOGW(TAG, "- DYN CHR - rx temperature = %.02f", peer->dyn_payload.temp.f);
        } else
        {
            if(peer->position != 0)
            {
                ESP_LOGI(TAG, " pad number - %d", peer->position);
            }
            ESP_LOGI(TAG, "- DYN CHR - rx voltage = %.02f", peer->dyn_payload.vrect.f);
            ESP_LOGI(TAG, "- DYN CHR - rx current = %.02f", peer->dyn_payload.irect.f);
            ESP_LOGI(TAG, "- DYN CHR - rx temperature = %.02f", peer->dyn_payload.temp.f);
        }
        */
    }
}

uint8_t ble_central_update_control_enables(uint8_t enable, uint8_t full_power, uint8_t critical, struct peer *peer)
{
    int rc = 0;
    uint8_t value[PRU_CONTROL_CHAR_SIZE];
    const struct peer_chr *control_chr;

    time(&now);
    localtime_r(&now, &info);
    ESP_LOGI(TAG, "Time is %s", asctime(&info));

    /* Get attribute value of CRU control characteristic */
    control_chr = peer_chr_find_uuid(peer,
                        wpt_svc_uuid,
                        wpt_char_control_uuid);
    if (control_chr == NULL)
    {
        ESP_LOGE(TAG, "Error: Failed to read control characteristic");
        return 1;
    }
        
    peer->contr_payload.enable = enable;
    peer->contr_payload.full_power = full_power;
    peer->contr_payload.critical = critical;
    peer->contr_payload.RFU = 0;

    value[0] = enable;
    value[1] = full_power;
    value[2] = critical;
    value[3] = 0;
    value[4] = 0;

    rc = ble_gattc_write_no_rsp_flat(peer->conn_handle, control_chr->chr.val_handle,
                              (void *)value, sizeof value);

    if (!rc)
    {    
        //ENABLE OR DISABLE CHARGING 
        if(critical)
        {
            ESP_LOGW(TAG, "CRITICAL TRANSITION FROM LOW-POWER TO FULL-POWER on pad=%d", peer->position);
        } else {
            if (enable == 1)
            {
                if(full_power)
                {
                    ESP_LOGW(TAG, "FULL MODE");
                    full_power_pads[peer->position-1] = 1;
                } else {
                    ESP_LOGW(TAG, "HALF MODE");
                    low_power_pads[(peer->position)-1] = 1;
                }
                ESP_LOGW(TAG, "Enable charge on pad=%d", peer->position);
            }
            else if(enable == 0)
            {
                if(full_power)
                {
                    ESP_LOGW(TAG, "FULL MODE");
                    full_power_pads[peer->position-1] = 0;
                } else {
                    ESP_LOGW(TAG, "HALF MODE");
                    low_power_pads[(peer->position)-1] = 0;
                }
                ESP_LOGW(TAG, "Disable charge on pad=%d!", peer->position);
            }
        }
    }
    return rc;
}