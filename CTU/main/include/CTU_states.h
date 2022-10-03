/**
 * /////////////////////////// Description ///////////////////////////////
 * Internal structure for CTU state handling. It is mostly run by "main"
 * task but has some non-static functions with which some other task can
 * interact with "main". Also handles some important timers required to 
 * validate integrity of program and readings. State transitions are also
 * mostly handled by this module.
 * 
*/

#ifndef __CTU_STATES_H__
#define __CTU_STATES_H__

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include <sys/time.h>

#include "ble_central.h"
#include "peer.h"

/* Software timers duration */
#define PERIODIC_SCAN_TIMER_PERIOD    (pdMS_TO_TICKS(1000))
#define PERIODIC_SWITCH_TIMER_PERIOD  (pdMS_TO_TICKS(3000))

/* Voltage threshold during LOW-POWER mode */
#define VOLTAGE_LOW_THRESH 30

/*Voltage threshold during FULL-POWER mode */
#define VOLTAGE_FULL_THRESH 80

/*Voltage threshold for misalignment check */
#define VOLTAGE_MIS_THRESH 110

/* Type definition for state task parameters */
typedef struct CTU_task_params_s CTU_task_params_t;

/* Definition of state specific function */
typedef void task_state_fn_t(void *arg);

/* Redefinition of peer structure for use in CTU_states definition */
struct peer;

/* All states timer handles */
TimerHandle_t periodic_scan_t_handle;
TimerHandle_t localization_switch_pads_t_handle;

/**
 * @brief Possible states of CTU
 * 
 * @param NO_FAULT                0: No latching fault
 * @param ROGUE_OBJECT            1: Rogue object present
 * @param alert               2: Alert on CRU
*/
typedef enum {
    NO_FAULT = 0,
    ROGUE_OBJECT,
    alert
} CTU_fault_t;

/**
 * @brief Possible states of CTU
 * 
 * @param NULL_STATE                0: Not currently used
 * @param CTU_CONFIG_STATE          1: Configuration state
 * @param CTU_LOW_POWER_STATE       2: Low power state
 * @param CTU_POWER_TRANSFER_STATE  3: Power transfer state
 * @param CTU_LOCAL_FAULT_STATE     4: Local fault state
 * @param CTU_REMOTE_FAULT_STATE  5: Latching fault state
*/
typedef enum {
    NULL_STATE = 0,
    CTU_CONFIG_STATE,
    CTU_LOW_POWER_STATE,
    CTU_POWER_TRANSFER_STATE,
    CTU_LOCAL_FAULT_STATE,
    CTU_REMOTE_FAULT_STATE
} CTU_state_t;

/**
 * @brief CTU state handling task parmeter type
 * 
 * @param state          Type of state of the CTU
 * @param state_fn       Type specific function the CTU runs at the moment
 * @param state_fn_arg   Argument used in the state function
 * @param conn_handle    Connection handle of the priority peer (used sparsely)
 * @param itvl           Time interval between each iteration inside CTU_states_run(void)
*/
struct CTU_task_params_s
{
    CTU_state_t state;
    task_state_fn_t *state_fn;
    void *state_fn_arg;
    uint16_t conn_handle;
    uint32_t itvl;
};

/* Main function to run in Main context */
void CTU_states_run(void);

/* State setter (Semaphore protected) */
BaseType_t CTU_state_change(CTU_state_t p_state, void *arg);

/* Periodic scan timer used when CTU has more than 1 connexion simultaneously */
void CTU_periodic_scan_timeout(void *arg);

/* Timer to switch on/off pads in low power mode during localization process */
void CTU_periodic_pad_switch(void *arg);

/* Boolean function to check if peer is charging */
bool CTU_is_peer_charging(struct peer *peer);

/* Function used to stop any CTU_states specific timers */
void CTU_states_stop_timers(void);

/* Main global states structure accessible mostly read-only for some CTU modules */
volatile CTU_task_params_t m_CTU_task_param;

/* Semaphore used to protect against multiple switching of states simultaneously */
SemaphoreHandle_t m_set_state_sem;

/* Global declaration of latching fault reason */
CTU_fault_t latching_reason;

#endif