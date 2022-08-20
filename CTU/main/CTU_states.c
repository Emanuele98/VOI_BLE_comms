#include "include/CTU_states.h"

/* State duration constants (in ticks) */
#define CONFIG_MAIN_ITVL         (pdMS_TO_TICKS(100))
#define POWER_SAVE_MAIN_ITVL     (pdMS_TO_TICKS(100))
#define LOW_POWER_MAIN_ITVL      (pdMS_TO_TICKS(100))
#define POWER_TRANSFER_MAIN_ITVL (pdMS_TO_TICKS(100))
#define LATCHING_FAULT_MAIN_ITVL (pdMS_TO_TICKS(100))
#define LOCAL_FAULT_MAIN_ITVL    (pdMS_TO_TICKS(100))

static const char* TAG = "STATES";
static uint8_t latching_fault_count;

/* State handle functions */
static void CTU_configuration_state(void *arg);
static void CTU_power_save_state(void *arg);
static void CTU_low_power_state(void *arg);
static void CTU_power_transfer_state(void *arg);

/* Fault state handle functions */
static void CTU_local_fault_state(void *arg);
static void CTU_latching_fault_state(void *arg);

//defined in ble_central.c
extern const ble_uuid_t *wpt_svc_uuid;
extern const ble_uuid_t *wpt_char_CRU_dyn_uuid;

static int counter = 0;

/**
 * @brief The Apps main function
 * @details This function handles the Main tasks flow. It begins by waiting for
 *          synchronization with the Host and procedes to loop with varying state
 *          functions. It ends only when the state is invalid.
*/
void CTU_states_run(void)
{
    enabled[0] = 0;
    enabled[1] = 0;
    enabled[2] = 0;
    enabled[3] = 0;

    //ESP_LOGI(TAG, "States main fct is run by task %s on core CPU%d", pcTaskGetTaskName(xTaskGetCurrentTaskHandle()),
    //                                                                xTaskGetAffinity(xTaskGetCurrentTaskHandle()));

    /* IMPORTANT: Must wait for host synchronisation to end before starting runtime */
    while (m_CTU_task_param.state != CTU_CONFIG_STATE)
    {
        ESP_LOGI(TAG, "Waiting on sync...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    /* State management loop (Main context) */
    /* Needs to feed watchdog as well */
    while(1)
    {
        if (m_CTU_task_param.state > 0)
        {
            /* Run state configured during runtime */
            m_CTU_task_param.state_fn(m_CTU_task_param.state_fn_arg);
        }
        else
        {
            ESP_LOGW(TAG, "Idle Null State");
        }

        /* Feed watchdog and allocate resources */
        vTaskDelay(m_CTU_task_param.itvl);
    }
}

/**
 * @brief Change function for the current state used by Main
 * @details This function will change the state in which the Main loop will run.
 *          It is protected by a Mutex which ensures only one call can be made at 
 *          the same time.
 * 
 * @param p_state The new state that will be used by the Main loop
 * @param arg void argument for certain fringe cases
 * @return pdFALSE if state is Null, pdTRUE otherwise
*/
BaseType_t CTU_state_change(CTU_state_t p_state, void *arg)
{
    if (xSemaphoreTake(m_set_state_sem, portMAX_DELAY) == pdTRUE)
    {
        m_CTU_task_param.state_fn_arg = arg;
        if (p_state == CTU_CONFIG_STATE) 
        {

            m_CTU_task_param.state_fn = CTU_configuration_state;
            m_CTU_task_param.state = CTU_CONFIG_STATE;
            m_CTU_task_param.itvl = CONFIG_MAIN_ITVL;

            ESP_LOGI(TAG,"Configuration State");
        }
        else if (p_state == CTU_POWER_SAVE_STATE)
        {
            // Keep power interface closed
            enabled[0] = 0;
            enabled[1] = 0;
            enabled[2] = 0;
            enabled[3] = 0;

            m_CTU_task_param.state_fn = CTU_power_save_state;
            m_CTU_task_param.state = CTU_POWER_SAVE_STATE;
            m_CTU_task_param.itvl = POWER_SAVE_MAIN_ITVL;

            ESP_LOGI(TAG,"Power Save State");
        }
        else if (p_state == CTU_LOW_POWER_STATE)
        {

            m_CTU_task_param.state_fn = CTU_low_power_state;
            m_CTU_task_param.state = CTU_LOW_POWER_STATE;
            m_CTU_task_param.itvl = LOW_POWER_MAIN_ITVL;

            ESP_LOGI(TAG,"Low Power State");
        }
        else if (p_state == CTU_POWER_TRANSFER_STATE)
        {            

            /* Set power to nominal value */
            m_CTU_task_param.state_fn = CTU_power_transfer_state;
            m_CTU_task_param.state = CTU_POWER_TRANSFER_STATE;
            m_CTU_task_param.itvl = POWER_TRANSFER_MAIN_ITVL;

            ESP_LOGI(TAG,"Power Transfer State");
        }
        else if (p_state == CTU_LOCAL_FAULT_STATE)
        {
            m_CTU_task_param.state_fn = CTU_local_fault_state;
            m_CTU_task_param.state = CTU_LOCAL_FAULT_STATE;
            m_CTU_task_param.itvl = LOCAL_FAULT_MAIN_ITVL;

            ESP_LOGI(TAG,"Local Fault State");
        }
        else if (p_state == CTU_LATCHING_FAULT_STATE)
        {
            struct peer *peer = (struct peer *)arg;

            m_CTU_task_param.state_fn = CTU_latching_fault_state;
            m_CTU_task_param.state = CTU_LATCHING_FAULT_STATE;
            m_CTU_task_param.itvl = LATCHING_FAULT_MAIN_ITVL;

            ESP_LOGI(TAG,"Peer %d causes Latching Fault State",peer->conn_handle);
        }
        else
        {
            ESP_LOGW(TAG, "NULL STATE\n");
            xSemaphoreGive(m_set_state_sem);
            return pdFALSE;
        }
        xSemaphoreGive(m_set_state_sem);
    }
    return pdTRUE;
}

/**
 * @brief Handle function for the configuration state
 * @details This function will handle the processes needed in the CTU
 *          configuration state.
 * 
 * @param arg void argument (not currently used)
*/
static void CTU_configuration_state(void *arg)
{

    /* Start local check timer */
    xTimerStart(periodic_local_check_t_handle, 0);  

    //scan other CRUs
    ble_central_scan_start(BLE_HS_FOREVER, BLE_PERIODIC_SCAN_ITVL, BLE_PERIODIC_SCAN_WIND);
    xTimerStart(periodic_scan_t_handle, pdMS_TO_TICKS(1000));

    /* Enter power save state only when at least one A-CTU is connected */
    if(peer_get_NUM_AUX_CTU()) {
        CTU_state_change(CTU_POWER_SAVE_STATE, NULL);
    }
}

/**
 * @brief Handle function for the power saving state
 * @details This function will handle the processes needed in the CTU
 *          power save state. It will make sure any flag change either
 *          from BLE central module.
 *          Generally, this handle is the default state the Main should
 *          have if no CRU is in reach.
 * 
 * @param arg void argument (not currently used)
*/
static void CTU_power_save_state(void *arg)
{
    /**
     * It is a repeating state
     *
    */
}

/**
 * @brief Handle function for the low power state
 * @details This function will handle all processes needed in the Low power state.
 *          Here the connection is enstablished and the loccalization process
 *          of the first connected peer happens.
 * 
 * @param arg void argument (not currently used)
*/
static void CTU_low_power_state(void *arg)
{
    /**
     * The low power state is mostly handled in CTU_state_change(...).
     *
    */
}

/** 
 * @brief Power transfer state handle function per AFA specifications
 * @details The general idea behind this function is to allow power to be transfered
 *          to all CRUs in range. All registered peers will go through an analysis of their 
 *          dynamic characteristics and, if necessary, their alert characteristics.
 * 
 * @param arg void argument (not currently used)
*/
static void CTU_power_transfer_state(void *arg)
{
    //sending power!
    //todo: repeating state -> calculate efficiency and send data over mqtt!
}


/** 
 * @brief Handle function for the CTU local fault
 * @details Currently, this function handles any local fault the same way.
 *          It starts by closing the power interface and then procedes to reset the
 *          BLE Stack which will then allow the CTU to restart without
 *          rebooting unnecessarily.
 * 
 * @param arg void argument which contains the peer 
 */

static void CTU_local_fault_state(void *arg)
{
    //disable power interface
    struct peer *peer = (struct peer *)arg;
    if(peer != NULL) {
        ble_central_update_control_enables(0, 0, peer);
    }       

    //disconnect from the Auxiliary CTU
    ble_central_kill_AUX_CTU(peer->task_handle, peer->sem_handle, peer->conn_handle);

//todo: add exponential timer for peer->alert_payload.alert_field.overtemperature

    //if no A-CTU connected anymore --> reset BLE stack and go back to configuration state
    if (peer_get_NUM_AUX_CTU() == 0)
    {
        if (i2c_sem != NULL)
        {
            vSemaphoreDelete(i2c_sem);
        }

        /* Stop all tasks and all timers currently running */
        ble_central_kill_all_AUX_CTU();
        CTU_states_stop_timers();

        /* Idle until BLE stack resets */
        CTU_state_change(NULL_STATE,NULL);
        
        /* Reset BLE stack */
        ble_hs_sched_reset(BLE_HS_EAPP);
    }
}

/** 
 * @brief Handle function for the CTU latching fault
 * @details Currently, this function handles any alert driven latching fault
 *          the same way. A global variable (the number of latching faults
 *          since boot) is incremented. Also, the state changes to the Power
 *          save state to allow for reconnection. The latching fault glocal
 *          variable will be reset if the CRU connections reach the power transfer
 *          state without any issues.
 * 
 * @param arg void argument which contains the peer 
 */

static void CTU_latching_fault_state(void *arg)
{
    //disable power interface
    struct peer *peer = (struct peer *)arg;
    if(peer->position) {
        struct peer *Aux_CTU = Aux_CTU_find(peer->position);
        if(Aux_CTU != NULL) {
            ble_central_update_control_enables(0, 0, Aux_CTU);    
        }
    }

    if ( peer_get_NUM_CRU() == 0)
    {
        /* Kill any CRU task remaining */
        ble_central_kill_all_CRU();

        /* Happens when the argument provided is linked to a CRU that was not connected
        prior to call */
        CTU_state_change(CTU_POWER_SAVE_STATE, NULL);
    }


    latching_fault_count++;
    if (peer->alert_payload.alert_field.overcurrent +
        peer->alert_payload.alert_field.overvoltage +
        peer->alert_payload.alert_field.overtemperature > 0)
    {
        ble_central_kill_CRU(peer->task_handle, peer->sem_handle, peer->conn_handle);

        /* Wait for alert to resolve by itself */
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    if (latching_fault_count == 3)
    {
        /* Reset number of latching faults */
        latching_fault_count = 0;

        /* Kill any CRU task remaining */
        ESP_LOGE(TAG, "Local manteinance needed!");
        //todo: send email to dave@bumblebee.com
    }
}

//! DOUBLE CHECK 
//todo: this function does what it is already done locally by each A-CTU! Probably to be removed 

/** 
 * @brief Periodic timer handle function for local faults
 * @details This handle function will diagnose the A-CTU I2C values 
 *          
 * 
 * @param arg void argument (not currently used)
 */
void CTU_periodic_local_check(void *arg)
{
    //check voltage, current and temperature for each peer
    struct peer *peer;

    SLIST_FOREACH(peer, &peers, next) {
        if(!peer->CRU)
        {
            if (peer->dyn_payload.vrect.f >= LOCAL_OVP)
                {
                    ESP_LOGE(TAG, "Voltage on pad %d too high!! -> value=%.02f",peer->conn_handle + 1, peer->dyn_payload.vrect.f);
                    CTU_state_change(CTU_LOCAL_FAULT_STATE, (void *)peer);
                }
            if (peer->dyn_payload.irect.f >= LOCAL_OCP)
                {
                    ESP_LOGE(TAG, "Current on pad %d too high!! -> value=%.02f", peer->conn_handle + 1, peer->dyn_payload.irect.f);
                    CTU_state_change(CTU_LOCAL_FAULT_STATE, (void *)peer);
                }
            if (peer->dyn_payload.temp.f >= LOCAL_OTP)
                {
                    ESP_LOGE(TAG, "Temperature on pad %d too high!! -> value=%.02f", peer->conn_handle + 1, peer->dyn_payload.temp.f);
                    CTU_state_change(CTU_LOCAL_FAULT_STATE, (void *)peer);
                }
        }
    }
   return;
}

/** 
 * @brief Periodic timer function to scan other CRUs
 * @details This handle function will scan periodically for other CRUs in the charging
 *          area. As long as the total number of peers does not exceed BLE_MAX_CONNECTIONS,
 *          and as long as there is at least one peer on the charging area, the timer will
 *          be in effect.
 * 
 * @param arg void argument (not currently used)
 */
void CTU_periodic_scan_timeout(void *arg)
{
    
    if ((peer_get_NUM_CRU() < MYNEWT_VAL(BLE_MAX_CONNECTIONS)) || (peer_get_NUM_AUX_CTU() < 5))
    {
        ble_gap_disc_cancel();
        ble_central_scan_start(BLE_HS_FOREVER, BLE_FIRST_SCAN_ITVL, BLE_FIRST_SCAN_WIND);
    }
    else
    {
        if (ble_gap_disc_active())
        {
            ble_gap_disc_cancel();
        }
        xTimerStop(periodic_scan_t_handle, 0);
    }
}

/** 
 * @brief Periodic timer function to switch on/off pads in low power mode during localization process
 * @details This handle function will check which are the pads not currently in used, and turn them on alternatevely.
 * 
 * @param arg void argument (not currently used)
 */
void CTU_periodic_pad_switch(void *arg)
{
    //number of pads
    uint8_t size = peer_get_NUM_AUX_CTU();
    //uint8_t size = 2;
    uint8_t pads_already_on[size];
    memset(pads_already_on, 0, size);


    get_pads_already_on(pads_already_on);

    uint8_t i = 0;

    //check if the pads is already charging another CRU (pads_already_on) 
    //and also if it is already enable in low power mode ( enabled )
                               
    switch(counter)
    {
        retry:
            case 0:
            //cannot happen but keep for testing
                if(size == 0) {
                    break;
                }
                if (size > 1) 
                {
                    counter++;
                } 
                if (!pads_already_on[0] && !enabled[0])
                {
                    enabled[0] = 1;
                    enabled[1] = enabled[2] = enabled[3] = 0;
                    ESP_LOGI(TAG, "Switch on first pad");
                    break;
                }
                if (size == 1) {
                    break;
                }
            
            case 1:
                if (size > 2)
                {
                    counter++;
                } else if (size == 2) 
                    {
                        counter = 0;
                    }
                if(!pads_already_on[1] && !enabled[1])
                {
                    enabled[1] = 1;
                    enabled[0] = enabled[2] = enabled[3] = 0; 
                    ESP_LOGI(TAG, "Switch on second pad");
                    break;
                }

            case 2:
                if (size > 3)
                {
                    counter++;
                } else if (size == 3) 
                    {
                        counter = 0;
                    }
                if(!pads_already_on[2] && !enabled[2])
                {
                    enabled[2] = 1;
                    enabled[1] = enabled[0] = enabled[3] = 0;  
                    ESP_LOGI(TAG, "Switch on third pad");
                    break;
                }

            case 3:
                if (size > 4)
                {
                    counter++;
                } else if (size == 4) 
                    {
                        counter = 0;
                    }
                if(!pads_already_on[3] && !enabled[3])
                {
                    enabled[3] = 1;
                    enabled[1] = enabled[2] = enabled[0] = 0; 
                    ESP_LOGI(TAG, "Switch on fourth pad");
                    break;
                }

            default:
                //after 1 retry, it means all pads are on. just break
                if (i==0) {
                    i = 1;
                    goto retry;
                } 
                break;
    }   
    
    //ESP_LOGI(TAG,  "%d - %d - %d - %d", enabled[0], enabled[1], enabled[2], enabled[3]);
                                                                                                                 
}

/** 
 * @brief Periodic timer function to scan other CRUs
 * @details This handle function whill scan periodically for other CRUs in the charging
 *          area. As long as the total number of peers does not exceed BLE_MAX_CONNECTIONS,
 *          and as long as there is at least one peer on the charging area, the timer will
 *          be in effect.
 */
void CTU_states_stop_timers(void)
{
    if (xTimerIsTimerActive(localization_switch_pads_t_handle) == pdTRUE)
    {
        xTimerStop(localization_switch_pads_t_handle, 10);
    }
    if (xTimerIsTimerActive(periodic_scan_t_handle) == pdTRUE)
    {
        xTimerStop(periodic_scan_t_handle, 10);
    }
    if (xTimerIsTimerActive(periodic_local_check_t_handle) == pdTRUE)
    {
        xTimerStop(periodic_local_check_t_handle, 10);
    }
}
