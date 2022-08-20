//The main resides on the APP_CPU (Core 1). 
//The host refers to another context, one that handles the NimBLE stack and its various states.
// The host resides on the PRO_CPU (Core 0).

#include "esp_err.h"

#include "ble_wpt_cru.h"

#include "include/cru_hw.h"

/*******************************************************************/
/**                           DEFINES                             **/
/*******************************************************************/

#define DEVICE_NAME                     "PRU"                          /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                32                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 20 ms). */

#define DYNAMIC_PARAM_TIMER_INTERVAL    pdMS_TO_TICKS(500)                    /**< Timer synced to Dynamic parameter characteristic (50 ms). */
#define NOTIFICATION_INTERVAL           pdMS_TO_TICKS(250)				       /**< Timer synced to Alert (Notification) parameter characteristic (250 ms). */

#define WPT_SVC_UUID16               0xFFFE

static const char* TAG = "MAIN";

int counter = 0;



//nimBLE
static uint8_t own_addr_type;
static int bleprph_gap_event(struct ble_gap_event *event, void *arg);




//read dynamic parameters from I2C sensors
static void dynamic_param_timeout_handler(void *arg)
{
    if (xSemaphoreTake(i2c_sem, portMAX_DELAY) == pdTRUE)
    {
        switch(counter)
        {
            //voltage
            case 0:
                counter++;
                dyn_payload.vrect.f = i2c_read_voltage_sensor();
                //dyn_payload.vrect.f = 10.12;
                break;
            
            //current
            case 1:
                counter++;
                dyn_payload.irect.f = i2c_read_current_sensor();
                //dyn_payload.irect.f = 10.13;
                break;

            //temperature 1
            case 2:
                counter = 0;
                dyn_payload.temp.f = i2c_read_temperature_sensor();
                //dyn_payload.temp.f = 10.14;
                break;
        }
    }
}

static void alert_timeout_handler(void *arg)
{      
  		// Validate temperature levels
		if (dyn_payload.temp.f > OVER_TEMPERATURE)
		{	alert_payload.alert_field.overtemperature = 1;	}
		else
		{	alert_payload.alert_field.overtemperature = 0;	}

        // Validate voltage levels
		if (dyn_payload.vrect.f > OVER_VOLTAGE)
		{	alert_payload.alert_field.overvoltage = 1;	}
		else
		{	alert_payload.alert_field.overvoltage = 0;	}

        // Validate current levels
		if (dyn_payload.irect.f > OVER_CURRENT)
		{	alert_payload.alert_field.overcurrent = 1;	}
		else
		{	alert_payload.alert_field.overcurrent = 0;	}


        // Values are then assigned to global payload instance of dynamic characteristic
        dyn_payload.alert = alert_payload.alert_field.internal;

}

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void bleprph_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    //PRU must advertises WPT service
    fields.uuids16 = (ble_uuid16_t[]) {
        BLE_UUID16_INIT(WPT_SVC_UUID16)
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    //set name
    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    //set appearance to be recognized as AUX CTU
    fields.appearance = 2;
    fields.appearance_is_present = 1;

    //set time interval
    fields.adv_itvl = APP_ADV_INTERVAL;
    fields.adv_itvl_is_present = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, bleprph_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleprph uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unused by
 *                                  bleprph.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        ESP_LOGI(TAG, "connection %s; status=%d ",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
        }

        if (event->connect.status != 0) {
            /* Connection failed; resume advertising. */
            bleprph_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:

        ESP_LOGI(TAG, "disconnect; reason=%d ", event->disconnect.reason);
        /* Stop timers */
        if (xTimerIsTimerActive(dynamic_t_handle) == pdTRUE)
        {
            xTimerStop(dynamic_t_handle, 10);
        }
        if (xTimerIsTimerActive(alert_t_handle) == pdTRUE)
        {
            xTimerStop(alert_t_handle, 10);
        }
        /* Connection terminated; resume advertising. */
        bleprph_advertise();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        ESP_LOGI(TAG, "connection updated; status=%d ",
                    event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        assert(rc == 0);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "advertise complete; reason=%d",
                    event->adv_complete.reason);
        bleprph_advertise();
        return 0;
    }

    return 0;
}

/** 
 * @brief Task function used by the BLE host (NimBLE)
 * @details This function will handle most BLE calls from any other
 *          task running on both cores.
 * 
 * @param param void argument which can be used if needed (not 
 *              currently used)
*/
void host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started\n");

    /* This function will return only when nimble_port_stop() is executed. */
    nimble_port_run();
    ESP_LOGI(TAG, "BLE Host Task Ended");

    nimble_port_freertos_deinit();
}

/** 
 * @brief Host reset callback function
 * @details This function will handle all scheduled BLE resets from 
 *          any task and any core.
 * 
 * @param reason The BLE HS reason for the reset
*/
static void host_ctrl_on_reset(int reason)
{
    ESP_LOGW(TAG, "Resetting state; reason=%d\n", reason);
}

/** 
 * @brief Host sync callback function
 * @details This function will handle a synchronisation event once the 
 *          NimBLE BLE stack ports on FreeRTOS.
*/
static void host_ctrl_on_sync(void)
{
    int rc;

    ESP_LOGI(TAG, "Device synced");

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "error determining address type; rc=%d\n", rc );
        return;
    }

    /* Printing ADDR */
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    ESP_LOGI(TAG, "Device Address: \n ");
    ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x \n", addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);

    /* Begin advertising. */
    bleprph_advertise();
}


/** 
 * @brief Initialization function for software timers
*/
void init_sw_timers(void)
{
    //
    // Create timers
    dynamic_t_handle = xTimerCreate("dynamic params", DYNAMIC_PARAM_TIMER_INTERVAL, pdTRUE, NULL, dynamic_param_timeout_handler);
    alert_t_handle = xTimerCreate("alert", NOTIFICATION_INTERVAL, pdTRUE, NULL, alert_timeout_handler);

    //xTimerStart(dynamic_t_handle, 0);


    if ((dynamic_t_handle == NULL) || (alert_t_handle == NULL))
    {
        ESP_LOGW(TAG, "Timers were not created successfully");
        return;
    }

}

/** 
 * @brief Initialization function for hardware
 * @details This init function serves for initialization as well as
 *          configuration of I2C Master interface
 *          
*/
void init_hw(void)
{
    esp_err_t err_code;

    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    err_code =  i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    ESP_ERROR_CHECK(err_code);
}

/** 
 * @brief Initialization function for all PTU modules
 * @details This is an all encompassing function designed to initialize what
 *          the PTU needs to function regardless of its current mode of operation.
*/
void init_setup(void)
{    
    /* Initialize software timers */
    init_sw_timers();
    
    /* Initialize GPIOs and other hardware peripherals (I2C) */
    init_hw();

    /* Initialize I2C semaphore */
    i2c_sem = xSemaphoreCreateMutex();
}

/** 
 * @brief Main function
 * @details This function handles the beginning of the program on reboot. Once it starts,
 *          both the BLE host core (CPU0) and the main core (CPU1) start with their respective
 *          tasks and configurations.
*/
void app_main(void)
{
    int rc;

    /* Initialize NVS partition */
    esp_err_t esp_err_code = nvs_flash_init();
    if  (esp_err_code == ESP_ERR_NVS_NO_FREE_PAGES || esp_err_code == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        esp_err_code = nvs_flash_init();
    }
    ESP_ERROR_CHECK(esp_err_code);

    /* Bind HCI and controller to NimBLE stack */
    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());
    nimble_port_init();

    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = host_ctrl_on_reset;
    ble_hs_cfg.sync_cb = host_ctrl_on_sync;

    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    assert(rc == 0);

    /* Host management loop (Host context) */
    nimble_port_freertos_init(host_task);

    /* Initialize all elements of CRU (timers and I2C)*/
    init_setup();   

}
