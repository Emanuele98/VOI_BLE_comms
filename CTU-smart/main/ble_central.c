#include "include/ble_central.h"
#include "host/ble_att.h"
#include "nvs_flash.h"
#include "nvs.h"

/**********************************************************************************/
/**                            Restricted globals                                **/
/**********************************************************************************/

/* Defines what type of task the unit will run on */
#define TASKS_CORE            0
#define TASK_STACK_SIZE       4200
#define CRU_TASK_PRIORITY     18
#define AUX_CTU_TASK_PRIORITY 18

//A-CTUs bluetooth addresses
static uint8_t Actu_addr1[6] = {0x72, 0x81, 0x24, 0xfb, 0x0b, 0xac};
static uint8_t Actu_addr2[6] = {0x5a, 0xe3, 0x26, 0xfb, 0x0b, 0xac};
static uint8_t Actu_addr3[6] = {0xda, 0x62, 0x25, 0xfb, 0x0b, 0xac};
static uint8_t Actu_addr4[6] = {0x9a, 0xbb, 0x25, 0xfb, 0x0b, 0xac};

//CRUs bluetooth addresses
static uint8_t cru_6F35[6] = {0x02, 0xec, 0x25, 0xfb, 0x0b, 0xac};
static uint8_t cru_3PAU[6] = {0x5a, 0x63, 0x25, 0xfb, 0x0b, 0xac};
static uint8_t cru_CE8J[6] = {0x86, 0x85, 0xed, 0x0c, 0x38, 0x90};
static uint8_t cru_D8X5[6] = {0xb2, 0x6d, 0x24, 0xfb, 0x0b, 0xac};

//timer 
static time_t switch_pad_ON = 0, loc_success = 0;
static float time_sec = 0, min_switch_time = 0;

//variable to know which pad is actually on in Low Power mode
uint8_t current_low_power = 0;

/* keep count of comms_error */
static uint16_t cru_comms_error = 0;
static uint16_t ctu_comms_error = 0;

/* keep track of comms error of the peer */
int16_t cru_last_rc[4];
int16_t aux_last_rc[4];

/* store last led state (1) GREEN (2) MISALIGNED */
uint8_t last_led[4];

/* Keep track of how many times one scooter has been checked to allow others to be prioritized (?) */
// --> will be resolved when BLE agent takes properly care of scanning
static uint8_t n_loc_checks[4] = {0, 0, 0, 0};

/* RSSI check during fully charged */
int8_t rssi;

int8_t rssiCheck[4];

/* keep track of leds state */
static uint8_t led_state[4] = {0, 0, 0, 0}; //default connected state

/* LIST OF TOPICS FOR MQTT AND SD CARD*/
 // TX
const char tx_voltage[4][80] =          {"warwicktrial/ctu/pad1/sensors/voltage", "warwicktrial/ctu/pad2/sensors/voltage",
                                         "warwicktrial/ctu/pad3/sensors/voltage", "warwicktrial/ctu/pad4/sensors/voltage"};
const char tx_current[4][80] =          {"warwicktrial/ctu/pad1/sensors/current", "warwicktrial/ctu/pad2/sensors/current",
                                         "warwicktrial/ctu/pad3/sensors/current", "warwicktrial/ctu/pad4/sensors/current"};
const char tx_temp1[4][80] =            {"warwicktrial/ctu/pad1/sensors/temperature1", "warwicktrial/ctu/pad2/sensors/temperature1",
                                         "warwicktrial/ctu/pad3/sensors/temperature1", "warwicktrial/ctu/pad4/sensors/temperature1"};
const char tx_temp2[4][80] =            {"warwicktrial/ctu/pad1/sensors/temperature2", "warwicktrial/ctu/pad2/sensors/temperature2",
                                         "warwicktrial/ctu/pad3/sensors/temperature2", "warwicktrial/ctu/pad4/sensors/temperature2"};
const char tx_power[4][80] =            {"warwicktrial/ctu/pad1/sensors/power", "warwicktrial/ctu/pad2/sensors/power",
                                         "warwicktrial/ctu/pad3/sensors/power", "warwicktrial/ctu/pad4/sensors/power"};
const char tx_efficiency[4][80] =       {"warwicktrial/ctu/pad1/sensors/efficiency", "warwicktrial/ctu/pad2/sensors/efficiency",
                                         "warwicktrial/ctu/pad3/sensors/efficiency", "warwicktrial/ctu/pad4/sensors/efficiency"};
const char tx_fod[4][80] =              {"warwicktrial/ctu/pad1/alerts/fod", "warwicktrial/ctu/pad2/alerts/fod",
                                         "warwicktrial/ctu/pad3/alerts/fod", "warwicktrial/ctu/pad4/alerts/fod"};
const char tx_overvoltage[4][80] =      {"warwicktrial/ctu/pad1/alerts/overvoltage", "warwicktrial/ctu/pad2/alerts/overvoltage",
                                         "warwicktrial/ctu/pad3/alerts/overvoltage", "warwicktrial/ctu/pad4/alerts/overvoltage"};
const char tx_overcurrent[4][80] =      {"warwicktrial/ctu/pad1/alerts/overcurrent", "warwicktrial/ctu/pad2/alerts/overcurrent",
                                         "warwicktrial/ctu/pad3/alerts/overcurrent", "warwicktrial/ctu/pad4/alerts/overcurrent"};
const char tx_overtemperature[4][80] =  {"warwicktrial/ctu/pad1/alerts/overtemperature", "warwicktrial/ctu/pad2/alerts/overtemperature",
                                         "warwicktrial/ctu/pad3/alerts/overtemperature", "warwicktrial/ctu/pad4/alerts/overtemperature"};
const char tx_status[4][80] =           {"warwicktrial/ctu/pad1/status", "warwicktrial/ctu/pad2/status",
                                         "warwicktrial/ctu/pad3/status", "warwicktrial/ctu/pad4/status"};
const char tx_scooter[4][80] =          {"warwicktrial/ctu/pad1/scooter", "warwicktrial/ctu/pad2/scooter",
                                         "warwicktrial/ctu/pad3/scooter", "warwicktrial/ctu/pad4/scooter"};
 // RX
const char rx_voltage[4][80] =          {"warwicktrial/cru/scooter3PAU/sensors/voltage", "warwicktrial/cru/scooter6F35/sensors/voltage",
                                         "warwicktrial/cru/scooterCE8J/sensors/voltage", "warwicktrial/cru/scooterD8X5/sensors/voltage"};
const char rx_current[4][80] =          {"warwicktrial/cru/scooter3PAU/sensors/current", "warwicktrial/cru/scooter6F35/sensors/current",
                                         "warwicktrial/cru/scooterCE8J/sensors/current", "warwicktrial/cru/scooterD8X5/sensors/current"};
const char rx_temp[4][80] =             {"warwicktrial/cru/scooter3PAU/sensors/temperature", "warwicktrial/cru/scooter6F35/sensors/temperature",
                                         "warwicktrial/cru/scooterCE8J/sensors/temperature", "warwicktrial/cru/scooterD8X5/sensors/temperature"};
const char rx_power[4][80] =            {"warwicktrial/cru/scooter3PAU/sensors/power", "warwicktrial/cru/scooter6F35/sensors/power",
                                         "warwicktrial/cru/scooterCE8J/sensors/power", "warwicktrial/cru/scooterD8X5/sensors/power"};
const char rx_charge_complete[4][80] =  {"warwicktrial/cru/scooter3PAU/alerts/chargecomplete", "warwicktrial/cru/scooter6F35/alerts/chargecomplete",
                                         "warwicktrial/cru/scooterCE8J/alerts/chargecomplete", "warwicktrial/cru/scooterD8X5/alerts/chargecomplete"};
const char rx_overvoltage[4][80] =      {"warwicktrial/cru/scooter3PAU/alerts/overvoltage", "warwicktrial/cru/scooter6F35/alerts/overvoltage",
                                         "warwicktrial/cru/scooterCE8J/alerts/overvoltage", "warwicktrial/cru/scooterD8X5/alerts/overvoltage"};
const char rx_overcurrent[4][80] =      {"warwicktrial/cru/scooter3PAU/alerts/overcurrent", "warwicktrial/cru/scooter6F35/alerts/overcurrent",
                                         "warwicktrial/cru/scooterCE8J/alerts/overcurrent", "warwicktrial/cru/scooterD8X5/alerts/overcurrent"};
const char rx_overtemperature[4][80] =  {"warwicktrial/cru/scooter3PAU/alerts/overtemperature", "warwicktrial/cru/scooter6F35/alerts/overtemperature",
                                         "warwicktrial/cru/scooterCE8J/alerts/overtemperature", "warwicktrial/cru/scooterD8X5/alerts/overtemperature"};

const char debug[80] = {"warwicktrial/debug"};

extern const char pads[4][20];
extern const char scooters[4][20];

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
    .scan_itvl = 0x0007,
    .scan_window = 0x0005,
    .itvl_min = BLE_WPT_INITIAL_CONN_ITVL_MIN,
    .itvl_max = BLE_WPT_INITIAL_CONN_ITVL_MAX,
    .latency = 0x0000,                              // allow to skip 5 scan_itvl times when no data to be sent
    .supervision_timeout = 0x01f0,                // allow 5s (496 * 10 ms)
    .min_ce_len = 0x0000,
    .max_ce_len = 0x0000,
};

// Declare ALL the WPT service and its characteristics UUIDs 
const ble_uuid_t *wpt_svc_uuid = 
    BLE_UUID128_DECLARE(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0xFE, 0xFF, 0x55, 0x64);

const ble_uuid_t *wpt_char_control_uuid = 
    BLE_UUID128_DECLARE(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x70, 0xE6, 0x55, 0x64);

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
void ble_central_kill_AUX_CTU(uint16_t conn_handle)
{
    struct peer *Aux_CTU = peer_find(conn_handle);

    ESP_LOGW(TAG, "Killing AUX CTU");

    // kill the scooter that was charging above it
    struct peer *peer = CRU_find(Aux_CTU->position);
    if (peer != NULL)
        ble_central_kill_CRU(peer->conn_handle, peer->task_handle);

    uint8_t rc = 1;
    //disable realtive interface
    if(Aux_CTU != NULL) 
    {
        if (Aux_CTU->position)
        {
            fully_charged[Aux_CTU->position-1] = 0;
            Aux_CTU->voi_code = EMPTY;
            if (low_power_pads[Aux_CTU->position-1])
            {
                while (rc)
                    rc = ble_central_update_control_enables(0, 0, 0, Aux_CTU); 
            }
            if (full_power_pads[Aux_CTU->position-1])
            {
                while (rc)
                    rc = ble_central_update_control_enables(0, 1, 0, Aux_CTU);   
            }
        }

        if (ble_gap_disc_active())
            ble_gap_disc_cancel();

        //BLE_HS_EAPP to disconnect permanently
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);

        if (Aux_CTU->task_handle != NULL)
        {
            esp_task_wdt_delete(Aux_CTU->task_handle);
            vTaskDelete(Aux_CTU->task_handle);           
            Aux_CTU->task_handle = NULL;
        }
    }
}


/** 
 * @brief Kill a CRU task currently running on CPU1
 * @details This function only removes the peer identified by the conn_handle parameter.
 * 
 */
void ble_central_kill_CRU(uint16_t conn_handle, TaskHandle_t task_handle)
{
    uint8_t rc = 1;
    struct peer *peer = peer_find(conn_handle);

    ESP_LOGW(TAG, "Killing CRU");
    
    if (peer!=NULL)
    {
        //IF WAS DOING THE LOCALIZATION PROCESS
        if(peer->localization_process)
        {
            peer->localization_process = false;
            uint8_t n = loc_pad_find();
            struct peer *Aux_CTU = Aux_CTU_find(n+1);
            if(Aux_CTU != NULL) 
            {
                while (rc)
                    rc = ble_central_update_control_enables(0, 0, 0, Aux_CTU);
            }
        }
        //IF IT WAS BEING CHARGED IN FULL-POWER
        if (peer->position)
        {
            full_power_pads[peer->position-1] = 0;
            struct peer *Aux_CTU = Aux_CTU_find(peer->position);
            if(Aux_CTU != NULL)
            {
                //CHANGE
                if (fully_charged[peer->voi_code])
                    led_state[peer->position-1] = 3;
                else
                    led_state[peer->position-1] = 0;
                while (rc)
                    rc = ble_central_update_control_enables(0, 1, led_state[peer->position-1], Aux_CTU);
            }
            time(&now);
            localtime_r(&now, &info);
            if(SD_CARD)
                write_sd_card(tx_status[peer->position-1], 0.00, &info);
            if (MQTT)
                esp_mqtt_client_publish(client, tx_status[peer->position-1], "0", 0, 0, 0);

            if (!peer->alert_payload.alert_field.charge_complete)
                peer->position = 0;
        }
        if (!peer->alert_payload.alert_field.charge_complete) //stay connected if fully charged until the scooter leaves the platform
        {
            fully_charged[peer->voi_code] = 0;
            if (ble_gap_disc_active())
                ble_gap_disc_cancel();

            //BLE_HS_EAPP to disconnect permanently
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        //DELETE THE TASK ALWAYS AT THE END
        if (task_handle != NULL)
        {
            esp_task_wdt_delete(peer->task_handle);
            vTaskDelete(peer->task_handle);
            peer->task_handle = NULL;
        }
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
            ble_central_kill_CRU(conn_handle, peer->task_handle);
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
            ble_central_kill_AUX_CTU(conn_handle);
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

    if (error->status != 0)
    {
        ESP_LOGE(TAG, "Unable to read (ON LOCALIZATION PROCESS): status=%d", error->status);
        ble_central_kill_CRU(peer->conn_handle, peer->task_handle);
        return 1;
    }

    // Unpack CRU Dynamic Characteristic
    ble_central_unpack_dynamic_param(attr, conn_handle);

    // DETECT POSITION OF THE CRU USING THE DYN CHR 
    // DO NOT GO TO POWER TRANSFER STATE UNTIL A POSITION IS DETECTED
    // is Vrect value above the Treshold?

    //ESP_LOGI(TAG, "Vrx = %.02f, Irx = %.02f", peer->dyn_payload.vrect.f, peer->dyn_payload.irect.f);

    time_t dyn_time = mktime(&peer->dyn_payload.dyn_time);
    min_switch_time = abs(difftime(dyn_time, switch_pad_ON));

    //if(peer->dyn_payload.vrect.f)
    //    ESP_LOGW(TAG, "voltage received %.02f", peer->dyn_payload.vrect.f);
    /*
    if (current_low_power)
    {
        struct peer *Aux_CTUx = Aux_CTU_find(current_low_power);
        //ESP_LOGI(TAG, "Current %.02f - Voltage %.02f", Aux_CTUx->dyn_payload.irect.f, Aux_CTUx->dyn_payload.vrect.f);
        char string_value[50];    
        sprintf(string_value, "Pad: %d - Voltage: %.02f and Current: %.02f", Aux_CTUx->position, Aux_CTUx->dyn_payload.vrect.f, Aux_CTUx->dyn_payload.irect.f);
        esp_mqtt_client_publish(client, debug, string_value, 0, 0, 0);
    }
    */

    if( (min_switch_time > MIN_SWITCH_TIME) && (!scooter_check[peer->voi_code]) && (current_low_power) )
    { 
        scooter_check[peer->voi_code] = true;
        n_loc_checks[peer->voi_code]++;
        
        //TEST LOC PROTOCOL
        char string_value[100];
        sprintf(string_value, "Scooter %s checked with pad: %d - Voltage: %.02f", peer->voi_code_string, current_low_power, peer->dyn_payload.vrect.f);
        if(MQTT)
            esp_mqtt_client_publish( client, debug, string_value, 0, 0, 0);
        ESP_LOGI(TAG, "%s", string_value);
        
        // CHECK VOLTAGE ONCE
        if (peer->dyn_payload.vrect.f > VOLTAGE_LOW_THRESH)
        {
            ESP_LOGE(TAG, " VOLTAGE TRESHOLD PASSED!");
            
            struct peer *Aux_CTU = Aux_CTU_find(current_low_power);
            if ((Aux_CTU == NULL) && (peer->conn_handle))
            {
                ESP_LOGE(TAG, "No AUX CTU found in position %d", current_low_power);
                return 1;
            }

            //make sure to send only once
            if (full_power_pads[current_low_power-1] == 0)
            {
                rc = ble_central_update_control_enables(1, 1, 1, Aux_CTU);
                //make sure the command goes through
                if (!rc)
                {
                    low_power_pads[3] = low_power_pads[2] = low_power_pads[1] = low_power_pads[0] = 0;
                    peer->localization_process = false;        
                    //attach scooter to the aux ctu
                    Aux_CTU->voi_code = peer->voi_code;
                    //SET RX POSITION
                    peer->position = current_low_power;

                    ESP_LOGI(TAG, " CRU is in position %d", peer->position );

                    //save led state
                    led_state[peer->position-1] = 1;

                    //store time
                    time(&loc_success);      
                    if (m_CTU_task_param.state == CTU_LOW_POWER_STATE)
                        CTU_state_change(CTU_POWER_TRANSFER_STATE, (void *)peer);
                } 
            }
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
    int rc = 1;
    
    // If the characteristic has not been read correctly
    if (error->status != 0)
    {
        ESP_LOGE(TAG, "Unable to read (ON AUX CTU DYN): status=%d", error->status);
        ble_central_kill_AUX_CTU(peer->conn_handle);
        return 1;
    }
   
    // Unpack peer Static Characteristic
    ble_central_unpack_dynamic_param(attr, conn_handle);

    // LOCALIZATION SWITCHING
    if (current_localization_process() && !full_power_pads[peer->position-1] && !fully_charged[peer->voi_code])
    {
        //led state
        led_state[peer->position-1] = 0;
        //SWITCH ON
        if (all_low_power_off())
        {
            if (baton == peer->position)
            {
                set_scooters_tobechecked();
                while (rc)
                    rc = ble_central_update_control_enables(1, 0, led_state[peer->position-1], peer);
                current_low_power = baton;
                time(&switch_pad_ON);
                //pass the baton to the next pad
                pass_the_baton(); 

                char string_value[50];
                sprintf(string_value, "low power ON pad: %d", peer->position); 
                if (MQTT)
                    esp_mqtt_client_publish(client, debug, string_value, 0, 0, 0);
            }
        } else
        {
            //SWITCH OFF 
            if ( (peer->position == loc_pad_find()+1) && (!low_power_alone(peer->position)) && (all_scooters_checked()) )
            {
                while (rc)
                    rc = ble_central_update_control_enables(0, 0, led_state[peer->position-1], peer);
                char string_value[50];
                sprintf(string_value, "low power OFF pad: %d", peer->position );
                if (MQTT)
                    esp_mqtt_client_publish(client, debug, string_value, 0, 0, 0);
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
    uint8_t rc = 1;

    struct peer *Aux_CTU = Aux_CTU_find(peer->position);
    if (Aux_CTU == NULL)
    {
        ESP_LOGE(TAG, "No AUX CTU found in position %d", peer->position);
        return 1;
    }

    // If the characteristic has not been read correctly
    if (error->status != 0)
    {
        ESP_LOGE(TAG, "Unable to read (ON CRU DYN): status=%d", error->status);
        // this happens only when the fully charged scooter leaves the platform 
        peer->alert_payload.alert_field.charge_complete = 0;
        ble_central_kill_CRU(peer->conn_handle, peer->task_handle);
        //vTaskDelay(500);
        ble_central_kill_AUX_CTU(Aux_CTU->conn_handle);
        return 1;
    }
   
    // Unpack peer Static Characteristic
    ble_central_unpack_dynamic_param(attr, conn_handle);     


    //CORRELATE TIMESTAMPS FOR EFFICIENCY
    time_t rx_time, tx_time;
    rx_time = mktime(&peer->dyn_payload.dyn_time);
    tx_time = mktime(&Aux_CTU->dyn_payload.dyn_time);
    time_sec = abs(difftime(rx_time, tx_time));
    if ((time_sec < CTU_TIMER_PERIOD) && (peer->dyn_payload.rx_power) && (Aux_CTU->dyn_payload.tx_power))
    {
        float eff;
        time(&now);
        time_sec = abs(difftime(now, rx_time));
        eff = ( peer->dyn_payload.rx_power / Aux_CTU->dyn_payload.tx_power ) * 100;
        if ((eff > 0) && (eff < 100))
        {
            ESP_LOGI(TAG, "Efficiency on position %d = %.02f", peer->position, eff);
            //WRITE EFFICIENCY
            char string_value[50];
            sprintf(string_value, "%.02f", eff);
            if (SD_CARD)
                write_sd_card(tx_efficiency[peer->position-1], eff, &peer->dyn_payload.dyn_time);
            if (MQTT)
                esp_mqtt_client_publish(client, tx_efficiency[peer->position-1], string_value, 0, 0, 0);               
        }
    }

    // print values in the logs
    //ESP_LOGI(TAG, "Vtx = %.02f, Itx = %.02f", Aux_CTU->dyn_payload.vrect.f, Aux_CTU->dyn_payload.irect.f);
    //ESP_LOGI(TAG, "Vrx = %.02f, Irx = %.02f", peer->dyn_payload.vrect.f, peer->dyn_payload.irect.f);

    // Check the charge has actually started (during the first 10 seconds) -- kind of double check on the localization algorithm
    time(&now);
    time_sec = abs(difftime(now, loc_success));
    if ( time_sec < BATTERY_REACTION_TIME )
    {
        if(peer->dyn_payload.vrect.f > VOLTAGE_FULL_THRESH_ON)
        {
            peer->correct = true;
            if (MQTT)
                esp_mqtt_client_publish(client, tx_scooter[peer->position-1], peer->voi_code_string, 0, 0, 0);
        }
    } 

    if ((!peer->correct) && (time_sec > BATTERY_REACTION_TIME ))
    {
        ESP_LOGE(TAG, "Voltage not received during the first 10 seconds!");
        ble_central_kill_CRU(peer->conn_handle, peer->task_handle);
        return 1;
    }

    // if the charge is not complete
    if (!peer->alert_payload.alert_field.charge_complete)
    {
        //DOUBLE CHECK THE IT IS STILL RECEIVING VOLTAGE
        if ((peer->dyn_payload.vrect.f < VOLTAGE_FULL_THRESH_OFF) && (time_sec > BATTERY_REACTION_TIME))
        {
            peer->correct = false;
            ESP_LOGE(TAG, "Voltage no longer received! --> SCOOTER LEFT THE PLATFORM");
            //NVS writing
            esp_err_t err = nvs_open("reconnection", NVS_READWRITE, &my_handle);
            if (err != ESP_OK) 
            {
                ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            } else 
            {
                time(&now);
                timeScooter[peer->voi_code] = now + RECONNECTION_SCOOTER_LEFT;
                nvs_set_i64(my_handle, scooters[peer->voi_code], timeScooter[peer->voi_code]);
                nvs_commit(my_handle);
                nvs_close(my_handle);
            }
            //disconnect the aux ctu to avoid FOD on roll off
            ble_central_kill_CRU(peer->conn_handle, peer->task_handle);
            //vTaskDelay(500);
            ble_central_kill_AUX_CTU(Aux_CTU->conn_handle);
            return 1;
        } 

        //send led command only if the charge is not complete
        if ((peer->dyn_payload.vrect.f > VOLTAGE_MIS_THRESH) || (!peer->correct))
            led_state[peer->position-1] = 1;
        else
            led_state[peer->position-1] = 2;

        if (last_led[peer->voi_code] != led_state[peer->position-1])
        {
            while (rc)
                rc = ble_central_update_control_enables(1, 1, led_state[peer->position-1], Aux_CTU);
            last_led[peer->voi_code] = led_state[peer->position-1];
        } 

    } else //if charge complete
    {
        time(&now);
        localtime_r(&now, &peer->alert_payload.alert_time);
        //RSSI CHECK
        int res = ble_gap_conn_rssi(conn_handle, &rssi);

        if (!res)
        {
            // check for a 3 times the RSSI (meaning scooter is still there)
            if (rssi > MINIMUM_FULLY_CHARGED_RSSI)
            {
                rssiCheck[peer->voi_code] = 0;
                // if yes, send the command to the leds (fully green)
                led_state[peer->position-1] = 3;
                while (rc)
                    rc = ble_central_update_control_enables(0, 0, led_state[peer->position-1], Aux_CTU);
                if (SD_CARD)
                    write_sd_card(rx_charge_complete[peer->voi_code], 1.00, &peer->alert_payload.alert_time);
                if (MQTT)
                    esp_mqtt_client_publish(client, rx_charge_complete[peer->voi_code], "1", 0, 0, 0);
            } else if (rssiCheck[peer->voi_code] >= RSSI_ATTEMPT)
                    {
                        // if no - disconnect both pad and scooter
                        if (SD_CARD)
                            write_sd_card(rx_charge_complete[peer->voi_code], 0.00, &peer->alert_payload.alert_time);
                        if (MQTT)
                        {
                            esp_mqtt_client_publish(client, debug, "Fully charged scooter left", 0, 0, 0);
                            esp_mqtt_client_publish(client, rx_charge_complete[peer->voi_code], "0", 0, 0, 0);
                        }
                        peer->alert_payload.alert_field.charge_complete = 0;
                        ble_central_kill_CRU(peer->conn_handle, peer->task_handle);
                        //vTaskDelay(500);
                        ble_central_kill_AUX_CTU(Aux_CTU->conn_handle);
                        return 1;
                    } else
                    {
                        rssiCheck[peer->voi_code]++;
                    }
        } 
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
        
    // If the characteristic has not been read correctly
    if (error->status != 0)
    {
        ESP_LOGE(TAG, "Unable to read (ON CRU ALERT): status=%d", error->status);
        if (peer->CRU)
                ble_central_kill_CRU(peer->conn_handle, peer->task_handle);
        else
                ble_central_kill_AUX_CTU(peer->conn_handle);
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
        ble_central_kill_AUX_CTU(peer->conn_handle);
        return;
    }

    alert_chr = peer_chr_find_uuid(peer,
                        wpt_svc_uuid,
                        wpt_char_alert_uuid);

    if (alert_chr == NULL)
    {
        ESP_LOGE(TAG, "ALERT chr not found!");
        ble_central_kill_AUX_CTU(peer->conn_handle);
        return;
    }

    err_code = esp_task_wdt_add(peer->task_handle);
    if (err_code)
    {
        ESP_LOGE(TAG, "AUX CTU task could not be added!");
        ble_central_kill_AUX_CTU(peer->conn_handle);
        return;
    }


    //attach the baton to an existing A-CTU
    baton = peer->position;
    // initialize variables
    peer->voi_code = EMPTY;
    fully_charged[peer->voi_code] = 0;
    aux_last_rc[peer->position-1] = -1;
    int task_delay = CTU_TIMER_PERIOD;

    // avoid red flags
    const struct peer_chr *static_chr;

    // Read the CRU Static parameter characteristic.
    static_chr = peer_chr_find_uuid(peer,
                             wpt_svc_uuid,
                             wpt_char_CRU_stat_uuid);
                             
    if (static_chr == NULL)
    {
        ESP_LOGE(TAG, "STATIC chr not found!");
        ble_central_kill_AUX_CTU(peer->conn_handle);
        return;
    }

    time(&loc_fail);
    
    /* WPT task loop */
    while (1)
    {
        //check alert is fine
        if (peer->dyn_payload.alert == 0)
        {
                /* Initiate Read procedure */        
                rc = ble_gattc_read(peer->conn_handle, dynamic_chr->chr.val_handle,
                                ble_central_on_AUX_CTU_dyn_read, (void *)peer);
        } else 
            {
                //!AVOID FOD DURING LOCALIZATION
                time(&now);                                 //CHANGE
                if (((current_localization_process() || (now < loc_fail + 10))  && !full_power_pads[peer->position-1] && !fully_charged[peer->voi_code])
                //!AVOID OV ALERTS FOR PAD 3
                    || ((peer->position == 3) && (peer->dyn_payload.vrect.f > 68) && (peer->dyn_payload.irect.f < 2.75) && (peer->dyn_payload.temp1.f < 55) && (peer->dyn_payload.temp2.f < 55)) )
                {
                    peer->dyn_payload.alert = 0;
                    /* Initiate Read procedure */        
                    rc = ble_gattc_read(peer->conn_handle, static_chr->chr.val_handle,
                        ble_central_on_alert_read, (void *)peer);
                }
                else
                {
                    rc = ble_gattc_read(peer->conn_handle, alert_chr->chr.val_handle,
                                    ble_central_on_alert_read, (void *)peer);
                }
            }

        //*ERROR HANDLING - DISCONNECT ONLY AFTER 60 CONSECUTIVE COMMS ERRORS        
        if (rc != ESP_OK)
        {
            if (aux_last_rc[peer->position-1] == rc)
            {
                if (ctu_comms_error < COMMS_ERROR_LIMIT)
                {
                    ctu_comms_error++;
                } else {
                    ESP_LOGE(TAG, "AUX CTU 60th consecutive task error - disconnect");
                    ctu_comms_error = 0;
                    //NVS writing
                    esp_err_t err = nvs_open("reconnection", NVS_READWRITE, &my_handle);
                    if (err != ESP_OK) 
                    {
                        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
                    } else 
                    {
                        time(&now);
                        timePad[peer->position-1] = now + RECONNECTION_COMMS_FAIL;
                        nvs_set_i64(my_handle, pads[peer->position-1], timePad[peer->position-1]);
                        nvs_commit(my_handle);
                        nvs_close(my_handle);
                    }
                    ble_central_kill_AUX_CTU(peer->conn_handle);        
                }
            } 
            //ESP_LOGW(TAG, "AUX CTU in position %d, task error number %d, error code rc=%d", peer->position, ctu_comms_error, rc);
            //RC = https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_err.html#_CPPv49esp_err_t
        } else if ((aux_last_rc[peer->position-1] != -1) && (aux_last_rc[peer->position-1]))
        {
            //ESP_LOGI(TAG, "Comms RECOVERY with AUX CTU %d", peer->position);
            ctu_comms_error = 0;
        }
        
        aux_last_rc[peer->position-1] = rc;

        //ESP_LOGW(TAG, "AUX-CTU high water mark: %d", uxTaskGetStackHighWaterMark( NULL ));

        // set Task Delay
        if ( current_localization_process() && !full_power_pads[peer->position-1] && !fully_charged[peer->voi_code] )
            task_delay = LOC_CTU_TIMER_PERIOD;
        else 
            task_delay = CTU_TIMER_PERIOD;

        /* Feed watchdog */
        esp_task_wdt_reset();

        vTaskDelay(task_delay);
    }

    //if for any reason the loop terminates
    esp_task_wdt_delete(NULL);
    vTaskDelete(NULL);   
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
        ble_central_kill_CRU(peer->conn_handle,peer->task_handle);
        return;
    }

    alert_chr = peer_chr_find_uuid(peer,
                        wpt_svc_uuid,
                        wpt_char_alert_uuid);

    if (alert_chr == NULL)
    {
        ESP_LOGE(TAG, "ALERT chr not found!");
        ble_central_kill_CRU(peer->conn_handle, peer->task_handle); 
        return;
    }

    err_code = esp_task_wdt_add(peer->task_handle);
    if(err_code)
    {
        ESP_LOGE(TAG, "CRU task could not be added!");
        ble_central_kill_CRU(peer->conn_handle, peer->task_handle); 
        return;
    }

    peer->localization_process = false;
    peer->correct = false;
    rssiCheck[peer->voi_code] = 0;
    cru_last_rc[peer->voi_code] = -1;
    last_led[peer->voi_code] = 0;
    fully_charged[peer->voi_code] = 0;
    n_loc_checks[peer->voi_code] = 0;

    int task_delay = CRU_TIMER_PERIOD;
    
    /* WPT task loop */
    while (1)
    {
        //check alert is fine
        if ((peer->dyn_payload.alert == 0) || (peer->alert_payload.alert_field.charge_complete))
        {
            if (peer->position == 0)
            {
                //check how many times the scooter has been checked - pause it for while to allow connection to other scooters
                if (n_loc_checks[peer->voi_code] < MAX_LOC_CHECKS)
                {
                    peer->localization_process = true;
                    rc = ble_gattc_read(peer->conn_handle, dynamic_chr->chr.val_handle, 
                               ble_central_on_localization_process, (void *)peer);
                } else
                    {
                        if(peer->localization_process)
                        {
                            time(&loc_fail);
                            peer->localization_process = false;
                        }
                        //reset variable after Min time //CHANGE
                        time(&now);
                        if (now > (loc_fail + MIN_TIME_AFTER_LOC))
                            n_loc_checks[peer->voi_code] = 0;
                        rc = 0;
                    }
            } else
            {   
                //* POWER TRANSFER PROCESS (keep reading dyn chr)
                /* Initiate Read procedure */        
                rc = ble_gattc_read(peer->conn_handle, dynamic_chr->chr.val_handle,
                                ble_central_on_CRU_dyn_read, (void *)peer);
            }
        } else 
            {
                rc = ble_gattc_read(peer->conn_handle, alert_chr->chr.val_handle,
                                ble_central_on_alert_read, (void *)peer);
            }
        
        //*ERROR HANDLING - DISCONNECT ONLY AFTER 60 CONSECUTIVE COMMS ERRORS
        if (rc != ESP_OK)
        {
            if (cru_last_rc[peer->voi_code] == rc)
            {
                if (cru_comms_error < COMMS_ERROR_LIMIT)
                {
                    cru_comms_error++;
                } else {
                    ESP_LOGE(TAG, "CRU 60th  consecutive task error - disconnect"); 
                    cru_comms_error = 0;
                    //NVS writing
                    esp_err_t err = nvs_open("reconnection", NVS_READWRITE, &my_handle);
                    if (err != ESP_OK) 
                    {
                        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
                    } else 
                    {
                        time(&now);
                        timeScooter[peer->voi_code] = now + RECONNECTION_COMMS_FAIL;
                        nvs_set_i64(my_handle, scooters[peer->voi_code], timeScooter[peer->voi_code]);
                        nvs_commit(my_handle);
                        nvs_close(my_handle);
                    }
                    ble_central_kill_CRU(peer->conn_handle, peer->task_handle);
                }
            } 
            //ESP_LOGW(TAG, "CRU %d task error but loop anyway rc=%d", cru_comms_error, rc);
            //RC = https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_err.html#_CPPv49esp_err_t
        } else if ((cru_last_rc[peer->voi_code] != -1) && (cru_last_rc[peer->voi_code]))
        {
            //ESP_LOGI(TAG, "Comms RECOVERY with CRU");
            cru_comms_error = 0;
        }

        cru_last_rc[peer->voi_code] = rc;

        //ESP_LOGW(TAG, "CRU high water mark: %d", uxTaskGetStackHighWaterMark( NULL ));

        // set Task Delay
        if (peer->localization_process)
            task_delay = LOC_CRU_TIMER_PERIOD;
        else 
            task_delay = CRU_TIMER_PERIOD;

        /* Feed watchdog */
        esp_task_wdt_reset();

        vTaskDelay(task_delay);
    }

    //if for any reason the loop terminates
    esp_task_wdt_delete(NULL);
    vTaskDelete(NULL);
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
    BaseType_t err_code;

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
            ble_central_kill_CRU(peer->conn_handle, peer->task_handle); 
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
            ble_central_kill_AUX_CTU(peer->conn_handle); 
        } 
    }

    /* Adds the task handle to the peer structure to allow modifications in runtime */
    peer->task_handle = task_handle;


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
                ble_central_kill_CRU(peer->conn_handle, NULL);      
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
            ble_central_kill_CRU(peer->conn_handle, NULL);
        }
        else {
            ble_central_kill_AUX_CTU(peer->conn_handle);
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
            ble_central_kill_CRU(peer->conn_handle, NULL);
        }
        else {
            ble_central_kill_AUX_CTU(peer->conn_handle);
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
            ble_central_kill_CRU(peer->conn_handle, NULL);
        }
        else {
            ble_central_kill_AUX_CTU(peer->conn_handle);
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
            ble_central_kill_CRU(peer->conn_handle, NULL);
        }
        else {
            ble_central_kill_AUX_CTU(peer->conn_handle);
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
            ble_central_kill_CRU(peer->conn_handle, NULL);
        }
        else {
            ble_central_kill_AUX_CTU(peer->conn_handle);
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

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != ESP_OK)
    {
        ESP_LOGE(TAG, "error retrieving type of address");
        return;
    }
    //ESP_LOGI(TAG, "Address type: %d", own_addr_type);
    //remove semaphores and leds // use only ble addresses
/*  uint8_t out_id_addr[6] = {0};

    rc = ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, out_id_addr, NULL);
    if (rc != ESP_OK)
    {
        ESP_LOGE(TAG, "error retrieving address");
        return;
    }
    ESP_LOGI(TAG, "Address: %02x:%02x:%02x:%02x:%02x:%02x", out_id_addr[5], out_id_addr[4], out_id_addr[3], out_id_addr[2], out_id_addr[1], out_id_addr[0]);
*/
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
 * ALSO, the device shall wait the pre-determined time set in the NVS during a previous alert state.
 */
static int ble_central_should_connect(const struct ble_gap_disc_desc *disc)
{
    // The device has to be advertising direct connectability.
    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND)
    { 
        return 0;
    }

    //ESP_LOGI(TAG, "Address: %02x:%02x:%02x:%02x:%02x:%02x", disc->addr.val[5], disc->addr.val[4], disc->addr.val[3], disc->addr.val[2], disc->addr.val[1], disc->addr.val[0]);

    //strenght of the received signal
    if (disc->rssi < MINIMUM_ADV_RSSI)
    {
        return 0;
    }

    uint8_t x = 0;

    //connect only to registered BLE peripheral addresses
    if (disc->addr.type == 0)
    {
        //CONNECT TO A-CTUs
        if ((disc->addr.val[5] == Actu_addr1[5]) && (disc->addr.val[4] == Actu_addr1[4]) && (disc->addr.val[3] == Actu_addr1[3]) && (disc->addr.val[2] == Actu_addr1[2]) && (disc->addr.val[1] == Actu_addr1[1]) && (disc->addr.val[0] == Actu_addr1[0]))
            x = 1;
        else if ((disc->addr.val[5] == Actu_addr2[5]) && (disc->addr.val[4] == Actu_addr2[4]) && (disc->addr.val[3] == Actu_addr2[3]) && (disc->addr.val[2] == Actu_addr2[2]) && (disc->addr.val[1] == Actu_addr2[1]) && (disc->addr.val[0] == Actu_addr2[0]))
            x = 2;
        else if ((disc->addr.val[5] == Actu_addr3[5]) && (disc->addr.val[4] == Actu_addr3[4]) && (disc->addr.val[3] == Actu_addr3[3]) && (disc->addr.val[2] == Actu_addr3[2]) && (disc->addr.val[1] == Actu_addr3[1]) && (disc->addr.val[0] == Actu_addr3[0]))
            x = 3;
        else if ((disc->addr.val[5] == Actu_addr4[5]) && (disc->addr.val[4] == Actu_addr4[4]) && (disc->addr.val[3] == Actu_addr4[3]) && (disc->addr.val[2] == Actu_addr4[2]) && (disc->addr.val[1] == Actu_addr4[1]) && (disc->addr.val[0] == Actu_addr4[0]))
            x = 4;

        if ((m_CTU_task_param.state != CTU_CONFIG_STATE) && (!current_localization_process()))
        {
            if ((disc->addr.val[5] == cru_3PAU[5]) && (disc->addr.val[4] == cru_3PAU[4]) && (disc->addr.val[3] == cru_3PAU[3]) && (disc->addr.val[2] == cru_3PAU[2]) && (disc->addr.val[1] == cru_3PAU[1]) && (disc->addr.val[0] == cru_3PAU[0]))
                x = 5;
            else if ((disc->addr.val[5] == cru_6F35[5]) && (disc->addr.val[4] == cru_6F35[4]) && (disc->addr.val[3] == cru_6F35[3]) && (disc->addr.val[2] == cru_6F35[2]) && (disc->addr.val[1] == cru_6F35[1]) && (disc->addr.val[0] == cru_6F35[0]))
                x = 6;
            else if ((disc->addr.val[5] == cru_CE8J[5]) && (disc->addr.val[4] == cru_CE8J[4]) && (disc->addr.val[3] == cru_CE8J[3]) && (disc->addr.val[2] == cru_CE8J[2]) && (disc->addr.val[1] == cru_CE8J[1]) && (disc->addr.val[0] == cru_CE8J[0]))
                x = 7;
            else if ((disc->addr.val[5] == cru_D8X5[5]) && (disc->addr.val[4] == cru_D8X5[4]) && (disc->addr.val[3] == cru_D8X5[3]) && (disc->addr.val[2] == cru_D8X5[2]) && (disc->addr.val[1] == cru_D8X5[1]) && (disc->addr.val[0] == cru_D8X5[0]))
                x = 8;
        }

        if (x==0)
            return 0;

        if (((x==1) && (reconn_time >= timePad[0])) || ((x==2) && (reconn_time >= timePad[1])) || ((x==3) && (reconn_time >= timePad[2])) || ((x==4) && (reconn_time >= timePad[3])))
            return 1;
        
        if (((x==5) && (reconn_time >= timeScooter[VOI_3PAU])) || ((x==6) && (reconn_time >= timeScooter[VOI_6F35])) || ((x==7) && (reconn_time >= timeScooter[VOI_CE8J])) || ((x==8) && (reconn_time >= timeScooter[VOI_D8X5])))
            return 2;
        
        /*
        if ((x==1) || (x==2) || (x==3) || (x==4))
            return 1;
        if ((x==5) || (x==6) || (x==7) || (x==8))
            return 2;
        */
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
    if (!ble_central_should_connect(disc)) 
    {
        return;
    } else if(ble_central_should_connect(disc) == 1) 
    {
        ESP_LOGI(TAG, "AUX CTU found!");
        slave_type = 1;
        time(&conf_time);    
    } else if(ble_central_should_connect(disc) == 2) 
    {
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
            
            //time(&now);
            //localtime_r(&now, &info);
            //ESP_LOGI(TAG, "Time is %s", asctime(&info));
            
            const uint8_t* slave_type = (const uint8_t *)arg;

            //convert pointer to integer
            uintptr_t type = (uintptr_t) slave_type;

            if(type == 1) {
                //ESP_LOGI(TAG, "Connection EVT with AUX CTU");
                /* Adds AUX CTU to list of connections */
                rc = peer_add(event->connect.conn_handle, 0);
            } else if (type == 2) {
                //ESP_LOGI(TAG, "Connection EVT with CRU");
                /* Adds CRU to list of connections */
                rc = peer_add(event->connect.conn_handle, 1);
            }

            peer = peer_find(event->connect.conn_handle);

            if (rc != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to add peer; rc=%d\n", rc);
                if(type == 1) {
                    ble_central_kill_AUX_CTU(peer->conn_handle);
                } else if (type == 2) {
                    ble_central_kill_CRU(peer->conn_handle, NULL);
                }
                break;
            }

            if (MQTT)
                esp_mqtt_client_publish( client, debug, "connection", 0, 0, 0);

            /* Perform service discovery. */
            rc = peer_disc_all(event->connect.conn_handle,
                               ble_central_on_disc_complete, NULL);
            if (rc != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to discover services; rc=%d\n", rc);
                if(type == 1) {
                    ble_central_kill_AUX_CTU(peer->conn_handle);
                } else if (type == 2) {
                    ble_central_kill_CRU(peer->conn_handle, NULL);
                }
            }
            //vTaskDelay(pdMS_TO_TICKS(200));
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
        
        //time(&now);
        //localtime_r(&now, &info);
        //ESP_LOGI(TAG, "Time is %s", asctime(&info));

        /* Connection terminated. */
        ESP_LOGW(TAG, "Disconnect EVT; reason=%d ", event->disconnect.reason);
        //ESP_LOGW(TAG, "Delete peer structure for conn_handle=%d",event->disconnect.conn.conn_handle);

        peer = peer_find(event->disconnect.conn.conn_handle);
        if (peer != NULL)
        {
            if(peer->CRU)
            {
                ESP_LOGE(TAG, "CRU");
                //this may happen when scooter leaves the platform after being fully charged
                peer->alert_payload.alert_field.charge_complete = 0;
                if (peer->conn_handle)
                    ble_central_kill_CRU(peer->conn_handle, NULL);
            } else 
            {
                if (peer->conn_handle)
                    ble_central_kill_AUX_CTU(peer->conn_handle);
                if (peer->position) 
                    ESP_LOGE(TAG, "A-CTU in position: %d", peer->position);
                if (!peer_get_NUM_AUX_CTU())
                    CTU_state_change(CTU_CONFIG_STATE, (void *)peer);
            }
        }

        peer_delete(event->disconnect.conn.conn_handle);
    
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
static void ble_central_unpack_AUX_CTU_alert_param(struct os_mbuf* om, uint16_t conn_handle)
{
    struct peer *peer = peer_find(conn_handle);

    if (peer!=NULL)
    {
        //reset the alert field in the dynamic payload
        peer->dyn_payload.alert = 0;
        //read the alert payload
        peer->alert_payload.alert_field.internal = om->om_data[0];

        //attach timestamps
        time(&now);
        localtime_r(&now, &peer->alert_payload.alert_time);

        const char string_value[2] = "1";

        if(peer->alert_payload.alert_field.overvoltage)
        {
            //ESP_LOGE(TAG, "ALERT -- OVERVOLTAGE -- aux ctu position %d", peer->position);
            CTU_state_change(CTU_LOCAL_FAULT_STATE, (void *)peer);
            if (SD_CARD)
                write_sd_card(tx_overvoltage[peer->position-1], peer->dyn_payload.vrect.f, &peer->alert_payload.alert_time);
            if (MQTT)
                esp_mqtt_client_publish(client, tx_overvoltage[peer->position-1], string_value, 0, 0, 0);
        } else if(peer->alert_payload.alert_field.overcurrent)
            {
                //ESP_LOGE(TAG, "ALERT -- OVERCURRENT -- aux ctu position %d", peer->position);
                CTU_state_change(CTU_LOCAL_FAULT_STATE, (void *)peer);
                if (SD_CARD)
                    write_sd_card(tx_overcurrent[peer->position-1], peer->dyn_payload.irect.f, &peer->alert_payload.alert_time);
                if (MQTT)
                    esp_mqtt_client_publish(client, tx_overcurrent[peer->position-1], string_value, 0, 0, 0);
            } else if(peer->alert_payload.alert_field.overtemperature)
                {
                    //ESP_LOGE(TAG, "ALERT -- OVERTEMPERATURE -- aux ctu position %d", peer->position);
                    CTU_state_change(CTU_LOCAL_FAULT_STATE, (void *)peer);
                    if (SD_CARD)
                        write_sd_card(tx_overtemperature[peer->position-1], peer->dyn_payload.temp1.f, &peer->alert_payload.alert_time);
                    if (MQTT)
                        esp_mqtt_client_publish(client, tx_overtemperature[peer->position-1], string_value, 0, 0, 0);
                } else if(peer->alert_payload.alert_field.FOD)
                {
                    //ESP_LOGE(TAG, "ALERT -- FOD -- aux ctu position %d", peer->position);
                    CTU_state_change(CTU_LOCAL_FAULT_STATE, (void *)peer);
                    if (SD_CARD)
                        write_sd_card(tx_fod[peer->position-1], 1.00, &peer->alert_payload.alert_time);
                    if (MQTT)
                       esp_mqtt_client_publish(client, tx_fod[peer->position-1], string_value, 0, 0, 0);
                } 
    }
}

static void ble_central_unpack_CRU_alert_param(struct os_mbuf* om, uint16_t conn_handle)
{
    struct peer *peer = peer_find(conn_handle);

    if (peer!=NULL)
    {
        //reset the alert field in the dynamic payload
        peer->dyn_payload.alert = 0;
        //read the alert payload
        peer->alert_payload.alert_field.internal = om->om_data[0];
        
        //attach timestamps
        time(&now);
        localtime_r(&now, &peer->alert_payload.alert_time);

        const char string_value[2] = "1";

        if(peer->alert_payload.alert_field.overvoltage)
        {
            //ESP_LOGE(TAG, "ALERT -- OVERVOLTAGE -- cru");
            CTU_state_change(CTU_REMOTE_FAULT_STATE, (void *)peer);
            if (SD_CARD)
                write_sd_card(rx_overvoltage[peer->voi_code], peer->dyn_payload.vrect.f, &peer->alert_payload.alert_time);
            if (MQTT)
                esp_mqtt_client_publish(client, rx_voltage[peer->voi_code], string_value, 0, 0, 0);
        } else if(peer->alert_payload.alert_field.overcurrent)
            {
                //ESP_LOGE(TAG, "ALERT -- OVERCURRENT -- cru");
                CTU_state_change(CTU_REMOTE_FAULT_STATE, (void *)peer);
                if (SD_CARD)
                    write_sd_card(rx_overcurrent[peer->voi_code], peer->dyn_payload.irect.f, &peer->alert_payload.alert_time);
                if (MQTT)
                    esp_mqtt_client_publish(client, rx_overcurrent[peer->voi_code], string_value, 0, 0, 0);
            } else if(peer->alert_payload.alert_field.overtemperature)
                {
                    //ESP_LOGE(TAG, "ALERT -- OVERTEMPERATURE -- cru");
                    CTU_state_change(CTU_REMOTE_FAULT_STATE, (void *)peer);
                    if (SD_CARD)
                        write_sd_card(rx_overtemperature[peer->voi_code], peer->dyn_payload.temp1.f, &peer->alert_payload.alert_time);
                    if (MQTT)
                        esp_mqtt_client_publish(client, rx_overtemperature[peer->voi_code], string_value, 0, 0, 0);
                }  else if (peer->alert_payload.alert_field.charge_complete)
                    {
                        //ESP_LOGE(TAG, "ALERT -- CHARGE COMPLETE  -- cru");
                        fully_charged[peer->voi_code] = true;

                        ble_central_kill_CRU(peer->conn_handle, NULL);
                        if (SD_CARD)
                            write_sd_card(rx_charge_complete[peer->voi_code], 1.00, &peer->alert_payload.alert_time);
                        if (MQTT)
                            esp_mqtt_client_publish(client, rx_charge_complete[peer->voi_code], string_value, 0, 0, 0);
                    }   
    }
}

/* Successivelly parse attribute data buffer to find appropriate values */
static void ble_central_unpack_static_param(const struct ble_gatt_attr *attr, uint16_t conn_handle)
{
    struct peer *peer = peer_find(conn_handle);

    if (peer!=NULL)
    {
        peer->stat_payload.ble_addr0 = attr->om->om_data[0];
        peer->stat_payload.ble_addr1 = attr->om->om_data[1];
        peer->stat_payload.ble_addr2 = attr->om->om_data[2];
        peer->stat_payload.ble_addr3 = attr->om->om_data[3];
        peer->stat_payload.ble_addr4 = attr->om->om_data[4];
        peer->stat_payload.ble_addr5 = attr->om->om_data[5];
    }

    //ASSIGN CORRECT POSITION OF A-CTU USING THEIR MAC ADDRESSES
    if(!peer->CRU)
    {
        if ((peer->stat_payload.ble_addr5 == Actu_addr1[5]) && (peer->stat_payload.ble_addr4 == Actu_addr1[4]) && (peer->stat_payload.ble_addr3 == Actu_addr1[3]) 
            && (peer->stat_payload.ble_addr2 == Actu_addr1[2]) && (peer->stat_payload.ble_addr1 == Actu_addr1[1]) && (peer->stat_payload.ble_addr0 == Actu_addr1[0]))
        {
            peer->position = 1;
        } else if ((peer->stat_payload.ble_addr5 == Actu_addr2[5]) && (peer->stat_payload.ble_addr4 == Actu_addr2[4]) && (peer->stat_payload.ble_addr3 == Actu_addr2[3]) 
                   && (peer->stat_payload.ble_addr2 == Actu_addr2[2]) && (peer->stat_payload.ble_addr1 == Actu_addr2[1]) && (peer->stat_payload.ble_addr0 == Actu_addr2[0]))
        {
            peer->position = 2;
        } else if ((peer->stat_payload.ble_addr5 == Actu_addr3[5]) && (peer->stat_payload.ble_addr4 == Actu_addr3[4]) && (peer->stat_payload.ble_addr3 == Actu_addr3[3]) 
                    && (peer->stat_payload.ble_addr2 == Actu_addr3[2]) && (peer->stat_payload.ble_addr1 == Actu_addr3[1]) && (peer->stat_payload.ble_addr0 == Actu_addr3[0]))
        {
            peer->position = 3;
        } else if ((peer->stat_payload.ble_addr5 == Actu_addr4[5]) && (peer->stat_payload.ble_addr4 == Actu_addr4[4]) && (peer->stat_payload.ble_addr3 == Actu_addr4[3]) 
                    && (peer->stat_payload.ble_addr2 == Actu_addr4[2]) && (peer->stat_payload.ble_addr1 == Actu_addr4[1]) && (peer->stat_payload.ble_addr0 == Actu_addr4[0]))
        {
            peer->position = 4;
        } /*else if ((peer->stat_payload.ble_addr5 == 0x30) && (peer->stat_payload.ble_addr4 == 0x8c) && (peer->stat_payload.ble_addr3 == 0x25) 
                    && (peer->stat_payload.ble_addr2 == 0xfb) && (peer->stat_payload.ble_addr1 == 0x0b) && (peer->stat_payload.ble_addr0 == 0xac))
        {
            peer->position = 1;
        } */else 
        { 
            peer->position = 0;
            ESP_LOGE(TAG, "ERROR - AUX CTU NOT IDENTIFIED ");
            ble_central_kill_AUX_CTU(peer->conn_handle);
        }

        char string_value[2] = "0";
        if (MQTT)
            esp_mqtt_client_publish(client, tx_status[peer->position-1], string_value, 0, 0, 0);
        ESP_LOGI(TAG, "AUX-CTU POSITION = %d", peer->position);
    } else 
    {
        if ((peer->stat_payload.ble_addr5 == cru_6F35[5]) && (peer->stat_payload.ble_addr4 == cru_6F35[4]) && (peer->stat_payload.ble_addr3 == cru_6F35[3]) 
                   && (peer->stat_payload.ble_addr2 == cru_6F35[2]) && (peer->stat_payload.ble_addr1 == cru_6F35[1]) && (peer->stat_payload.ble_addr0 == cru_6F35[0]))
        {
            ESP_LOGI(TAG, "6F35");
            strcpy(peer->voi_code_string, "6F35");
            peer->voi_code = VOI_6F35;
        } else if ((peer->stat_payload.ble_addr5 == cru_CE8J[5]) && (peer->stat_payload.ble_addr4 == cru_CE8J[4]) && (peer->stat_payload.ble_addr3 == cru_CE8J[3]) 
            && (peer->stat_payload.ble_addr2 == cru_CE8J[2]) && (peer->stat_payload.ble_addr1 == cru_CE8J[1]) && (peer->stat_payload.ble_addr0 == cru_CE8J[0]))
        {
            ESP_LOGI(TAG, "CE8J");
            strcpy(peer->voi_code_string, "CE8J");
            peer->voi_code = VOI_CE8J;
        } else if ((peer->stat_payload.ble_addr5 == cru_3PAU[5]) && (peer->stat_payload.ble_addr4 == cru_3PAU[4]) && (peer->stat_payload.ble_addr3 == cru_3PAU[3]) 
            && (peer->stat_payload.ble_addr2 == cru_3PAU[2]) && (peer->stat_payload.ble_addr1 == cru_3PAU[1]) && (peer->stat_payload.ble_addr0 == cru_3PAU[0]))
        {
            ESP_LOGI(TAG, "3PAU");
            strcpy(peer->voi_code_string, "3PAU");
            peer->voi_code = VOI_3PAU;
        } else if ((peer->stat_payload.ble_addr5 == cru_D8X5[5]) && (peer->stat_payload.ble_addr4 == cru_D8X5[4]) && (peer->stat_payload.ble_addr3 == cru_D8X5[3]) 
            && (peer->stat_payload.ble_addr2 == cru_D8X5[2]) && (peer->stat_payload.ble_addr1 == cru_D8X5[1]) && (peer->stat_payload.ble_addr0 == cru_D8X5[0]))
        { 
            ESP_LOGI(TAG, "D8X5");
            strcpy(peer->voi_code_string, "D8X5");
            peer->voi_code = VOI_D8X5;
        } else 
        { 
            ESP_LOGE(TAG, "ERROR - SCOOTER NOT IDENTIFIED");
            ble_central_kill_CRU(peer->conn_handle, NULL);
        }
        char string[50];
        sprintf(string, "Scooter %s connected", peer->voi_code_string);
        esp_mqtt_client_publish( client, debug, string, 0, 0, 0);
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
        time_sec = abs(difftime(now, loc_success));

        // save timestamp
        time(&now);
        localtime_r(&now, &peer->dyn_payload.dyn_time);

        char string_value[50];
        if ((peer->CRU) && (peer->position))
        {
            if (peer->dyn_payload.vrect.f > VOLTAGE_FULL_THRESH_ON)
                peer->dyn_payload.rx_power = peer->dyn_payload.vrect.f * peer->dyn_payload.irect.f;              
            else
                peer->dyn_payload.rx_power = 0;
            if (SD_CARD)
            {
                write_sd_card(rx_voltage[peer->voi_code], peer->dyn_payload.vrect.f, &peer->dyn_payload.dyn_time);
                write_sd_card(rx_current[peer->voi_code], peer->dyn_payload.irect.f, &peer->dyn_payload.dyn_time);
                write_sd_card(rx_temp[peer->voi_code], peer->dyn_payload.temp1.f, &peer->dyn_payload.dyn_time);
                write_sd_card(rx_power[peer->voi_code], peer->dyn_payload.rx_power, &peer->dyn_payload.dyn_time);
            }
            if (MQTT)
            {
                sprintf(string_value, "%.02f", peer->dyn_payload.vrect.f);
                esp_mqtt_client_publish(client, rx_voltage[peer->voi_code], string_value, 0, 0, 0);
                sprintf(string_value, "%.02f", peer->dyn_payload.irect.f);
                esp_mqtt_client_publish(client, rx_current[peer->voi_code], string_value, 0, 0, 0);
                sprintf(string_value, "%.02f", peer->dyn_payload.temp1.f);
                esp_mqtt_client_publish(client, rx_temp[peer->voi_code], string_value, 0, 0, 0);
                sprintf(string_value, "%.02f", peer->dyn_payload.rx_power);
                esp_mqtt_client_publish(client, rx_power[peer->voi_code], string_value, 0, 0, 0);

            }
        } if ( (!peer->CRU) && ( (!current_localization_process()) || full_power_pads[peer->position-1] ) )
        {
            if (peer->dyn_payload.vrect.f > 60)
                peer->dyn_payload.tx_power = peer->dyn_payload.vrect.f * peer->dyn_payload.irect.f;
            else
                peer->dyn_payload.tx_power = 0;
            if (SD_CARD)
            {
                write_sd_card(tx_voltage[peer->position-1], peer->dyn_payload.vrect.f, &peer->dyn_payload.dyn_time);
                write_sd_card(tx_current[peer->position-1], peer->dyn_payload.irect.f, &peer->dyn_payload.dyn_time);
                write_sd_card(tx_temp1[peer->position-1], peer->dyn_payload.temp1.f, &peer->dyn_payload.dyn_time);
                write_sd_card(tx_temp2[peer->position-1], peer->dyn_payload.temp2.f, &peer->dyn_payload.dyn_time);
                write_sd_card(tx_power[peer->position-1], peer->dyn_payload.tx_power, &peer->dyn_payload.dyn_time);
            }
            if (MQTT)
            {
                sprintf(string_value, "%.02f", peer->dyn_payload.vrect.f);
                esp_mqtt_client_publish(client, tx_voltage[peer->position-1], string_value, 0, 0, 0);
                sprintf(string_value, "%.02f", peer->dyn_payload.irect.f);
                esp_mqtt_client_publish(client, tx_current[peer->position-1], string_value, 0, 0, 0);
                sprintf(string_value, "%.02f", peer->dyn_payload.temp1.f);
                esp_mqtt_client_publish(client, tx_temp1[peer->position-1], string_value, 0, 0, 0);
                sprintf(string_value, "%.02f", peer->dyn_payload.temp2.f);
                esp_mqtt_client_publish(client, tx_temp2[peer->position-1], string_value, 0, 0, 0); 
                sprintf(string_value, "%.02f", peer->dyn_payload.tx_power);
                esp_mqtt_client_publish(client, tx_power[peer->position-1], string_value, 0, 0, 0);
            }
        }
    }
}

uint8_t ble_central_update_control_enables(uint8_t enable, uint8_t full_power, uint8_t led, struct peer *peer)
{
    int rc = 0;
    uint8_t value[PRU_CONTROL_CHAR_SIZE];
    const struct peer_chr *control_chr;

    //time(&now);
    //localtime_r(&now, &info);
    //ESP_LOGI(TAG, "Time is %s", asctime(&info));

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
    peer->contr_payload.led = led;
    peer->contr_payload.RFU = 0;

    value[0] = enable;
    //value[1] = full_power;
    value[1] = 1;
    value[2] = led;
    value[3] = 0;
    value[4] = 0;

    rc = ble_gattc_write_no_rsp_flat(peer->conn_handle, control_chr->chr.val_handle,
                              (void *)value, sizeof value);

    if (!rc)
    {    
        time(&now);
        localtime_r(&now, &info);

        //ENABLE OR DISABLE CHARGING 
        if(led)
        {
            ESP_LOGW(TAG, "SENDING LED COMMAND on pad=%d", peer->position);
            //todo: add MQTT
        }

        if (enable == 1)
        {
            if(full_power)
            {
                ESP_LOGW(TAG, "FULL MODE");
                full_power_pads[peer->position-1] = 1;
                
                const char string_value[2] = "1"; 
                if (SD_CARD)
                    write_sd_card(tx_status[peer->position-1], 1.00, &info);
                if (MQTT)
                    esp_mqtt_client_publish(client, tx_status[peer->position-1], string_value, 0, 0, 0);

            } else {
                ESP_LOGW(TAG, "HALF MODE");
                low_power_pads[peer->position-1] = 1;
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
                low_power_pads[peer->position-1] = 0;
            }
            ESP_LOGW(TAG, "Disable charge on pad=%d!", peer->position);
        }
    }
    return rc;
}