#include "include/CTU_states.h"

/* State duration constants (in ticks) */
#define CONFIG_MAIN_ITVL            pdMS_TO_TICKS(500)
#define LOW_POWER_MAIN_ITVL         pdMS_TO_TICKS(500)
#define POWER_TRANSFER_MAIN_ITVL    pdMS_TO_TICKS(500)
#define LATCHING_FAULT_MAIN_ITVL    pdMS_TO_TICKS(500)
#define LOCAL_FAULT_MAIN_ITVL       pdMS_TO_TICKS(500)

static const char* TAG = "STATES";
//static uint8_t latching_fault_count;

/* State handle functions */
static void CTU_configuration_state(void *arg);
static void CTU_low_power_state(void *arg);
static void CTU_power_transfer_state(void *arg);

/* Fault state handle functions */
static void CTU_local_fault_state(void *arg);
static void CTU_remote_fault_state(void *arg);

//defined in ble_central.c
extern const ble_uuid_t *wpt_svc_uuid;
extern const ble_uuid_t *wpt_char_CRU_dyn_uuid;

time_t conf_time;

/**
 * @brief The Apps main function
 * @details This function handles the Main tasks flow. It begins by waiting for
 *          synchronization with the Host and procedes to loop with varying state
 *          functions. It ends only when the state is invalid.
*/
void CTU_states_run(void)
{
    //ESP_LOGI(TAG, "States main fct is run by task %s on core CPU%d", pcTaskGetTaskName(xTaskGetCurrentTaskHandle()),
    //                                                                xTaskGetAffinity(xTaskGetCurrentTaskHandle()));

    /* IMPORTANT: Must wait for host synchronisation to end before starting runtime */
    while (m_CTU_task_param.state != CTU_CONFIG_STATE) {}

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
    //if (xSemaphoreTake(m_set_state_sem, pdMS_TO_TICKS(2000)) == pdTRUE)  //if GURU //change core //happens anyway
    //{
        m_CTU_task_param.state_fn_arg = arg;
        if (p_state == CTU_CONFIG_STATE) 
        {          
            m_CTU_task_param.state_fn = CTU_configuration_state;
            m_CTU_task_param.state = CTU_CONFIG_STATE;
            m_CTU_task_param.itvl = CONFIG_MAIN_ITVL;

            xTimerStart(periodic_scan_t_handle, 0);

            ESP_LOGI(TAG,"Configuration State!");
        }
        else if (p_state == CTU_LOW_POWER_STATE)
        {
            // Reset power interfaces
            low_power_pads[0] = 0;
            low_power_pads[1] = 0;
            low_power_pads[2] = 0;
            low_power_pads[3] = 0;
            full_power_pads[0] = 0;
            full_power_pads[1] = 0;
            full_power_pads[2] = 0;
            full_power_pads[3] = 0;

            m_CTU_task_param.state_fn = CTU_low_power_state;
            m_CTU_task_param.state = CTU_LOW_POWER_STATE;
            m_CTU_task_param.itvl = LOW_POWER_MAIN_ITVL;

            ESP_LOGI(TAG,"Low Power State!");
        }
        else if (p_state == CTU_POWER_TRANSFER_STATE)
        {            

            /* Set power to nominal value */
            m_CTU_task_param.state_fn = CTU_power_transfer_state;
            m_CTU_task_param.state = CTU_POWER_TRANSFER_STATE;
            m_CTU_task_param.itvl = POWER_TRANSFER_MAIN_ITVL;

            ESP_LOGI(TAG,"Power Transfer State!");
        }
        else if (p_state == CTU_LOCAL_FAULT_STATE)
        {
            m_CTU_task_param.state_fn = CTU_local_fault_state;
            m_CTU_task_param.state = CTU_LOCAL_FAULT_STATE;
            m_CTU_task_param.itvl = LOCAL_FAULT_MAIN_ITVL;

            ESP_LOGI(TAG,"Local Fault State!");
        }
        else if (p_state == CTU_REMOTE_FAULT_STATE)
        {
            struct peer *peer = (struct peer *)arg;

            m_CTU_task_param.state_fn = CTU_remote_fault_state;
            m_CTU_task_param.state = CTU_REMOTE_FAULT_STATE;
            m_CTU_task_param.itvl = LATCHING_FAULT_MAIN_ITVL;

            ESP_LOGI(TAG,"Peer %d causes Remote Fault State!",peer->conn_handle);
        }
        else
        {
            ESP_LOGW(TAG, "NULL STATE\n");
            //xSemaphoreGive(m_set_state_sem);
            return pdFALSE;
        }
        //xSemaphoreGive(m_set_state_sem);
    //}
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
    struct peer *peer;
    //set power interfaces to 0
    low_power_pads[0] = 0;
    low_power_pads[1] = 0;
    low_power_pads[2] = 0;
    low_power_pads[3] = 0;
    full_power_pads[0] = 0;
    full_power_pads[1] = 0;
    full_power_pads[2] = 0;
    full_power_pads[3] = 0;

    SLIST_FOREACH(peer, &peers, next) {
        if(peer->CRU)
        {
            ble_central_kill_CRU(peer->conn_handle);
        }
    }

    time(&conf_time);
    float time_sec = abs(difftime(conf_time, aux_found));

    /* Enter low power state only when the A-CTUs are connected and no more were found in the last 10 s*/
    if (((peer_get_NUM_AUX_CTU() == MAX_AUX_CTU) && (m_CTU_task_param.state == CTU_CONFIG_STATE)) && (time_sec > CONF_STATE_TIMEOUT))
        CTU_state_change(CTU_LOW_POWER_STATE, (void *)NULL);

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
    //todo: check again
    ESP_LOGE(TAG, "LOCAL FAULT STATE");

    struct peer *peer = (struct peer *)arg;
    if(peer)
    {
        //disconnect from the Auxiliary CTU
        ble_central_kill_AUX_CTU(peer->conn_handle);
        CTU_state_change(CTU_CONFIG_STATE, NULL);
    }

    //todo: if no A-CTU connected anymore --> reset BLE stack and go back to configuration state

/*
    if (peer_get_NUM_AUX_CTU() == 0)
    {
        // Stop all tasks and all timers currently running
        ble_central_kill_all_AUX_CTU();
        //todo: this function is deprecated
        CTU_states_stop_timers();

        // Idle until BLE stack resets
        CTU_state_change(NULL_STATE,NULL);
        
        // Reset BLE stack
        ble_hs_sched_reset(BLE_HS_EAPP);
    }
*/
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

static void CTU_remote_fault_state(void *arg)
{
    ESP_LOGE(TAG, "REMOTE FAULT STATE");

    //disable power interface
    struct peer *peer = (struct peer *)arg;

    if (peer)
    {
        //todo: use identification and add the time to wait before reconnection
        ble_central_kill_CRU(peer->conn_handle);
        if ((!CTU_is_charging()) && (m_CTU_task_param.state != CTU_LOW_POWER_STATE))
            CTU_state_change(CTU_LOW_POWER_STATE, (void *)peer);
        else
            CTU_state_change(CTU_CONFIG_STATE, NULL);
    }

    /*
    if (latching_fault_count == 3)
    {
        // Reset number of latching faults
        latching_fault_count = 0;

        // Kill any CRU task remaining
        ESP_LOGE(TAG, "Local manteinance needed!");
        //todo: send email to dave@bumblebee.com
    }
    */
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
    if ((peer_get_NUM_CRU() + peer_get_NUM_AUX_CTU()) < 9)
    {
        ble_gap_disc_cancel();
        ble_central_scan_start(BLE_SCAN_TIMEOUT, BLE_FIRST_SCAN_ITVL, BLE_FIRST_SCAN_WIND);
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

void pass_the_baton(void)
{
    int8_t next_bat = baton - 1;
    struct peer *peer;
    
    for (int i = 1; i < MAX_AUX_CTU; i++)
    {
        next_bat = ((next_bat + 1) % 4);
        peer = Aux_CTU_find(next_bat + 1);
        if (peer != NULL)
        {
            if (!full_power_pads[peer->position-1])
            {
                baton = peer->position;
                //ESP_LOGI(TAG, "baton %d", baton);
                return;
            }
        }
    }
}