//The main resides on the APP_CPU (Core 1). 
//The host refers to another context, one that handles the NimBLE stack and its various states.
// The host resides on the PRO_CPU (Core 0).

#include "esp_err.h"

#include "ble_wpt_cru.h"

#include "include/cru_hw.h"

/*******************************************************************/
/**                           DEFINES                             **/
/*******************************************************************/

#define DEVICE_NAME                     "CRU"                          /**< Name of device. Will be included in the advertising data. */

#define WPT_SVC_UUID16               0xFFFE

static const char* TAG = "MAIN";

static float volt, curr, t;
static int counter = 0, Temp_counter = 0, Volt_counter = 0, Curr_counter = 0, ChargeComp_counter = 0;

int i2c_master_port;
i2c_config_t conf;

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
                volt = i2c_read_voltage_sensor();
                if (volt !=-1)
                    dyn_payload.vrect.f = volt;
                break;
            
            //current
            case 1:
                counter++;
                curr = i2c_read_current_sensor();
                if (curr != -1)
                    dyn_payload.irect.f = curr;         
                break;

            //temperature 1
            case 2:
                counter = 0;
                t = i2c_read_temperature_sensor();
                if (t != -1)
                    dyn_payload.temp1.f = t;
                break;
            default:
                xSemaphoreGive(i2c_sem);
                break;
        }
    }
}

static void alert_timeout_handler(void *arg)
{      
  		// Validate temperature levels
		if (dyn_payload.temp1.f > OVER_TEMPERATURE)
        {
            Temp_counter++;
            ESP_LOGI(TAG, "OVER TEMPERATURE");
            if (Temp_counter > 99) 
            {
                ESP_LOGE(TAG, "OVER TEMPERATURE");
                alert_payload.alert_field.overtemperature = 1;
            } 
    	}

        // Validate voltage levels
		if (dyn_payload.vrect.f > OVER_VOLTAGE)
		{
            Volt_counter++;
            ESP_LOGI(TAG, "OVER VOLTAGE");
            if (Volt_counter > 5) 
            {
                ESP_LOGE(TAG, "OVER VOLTAGE");
                alert_payload.alert_field.overvoltage = 1;
            }	
        }

        // Validate current levels
		if (dyn_payload.irect.f > OVER_CURRENT)
		{
            Curr_counter++;
            ESP_LOGI(TAG, "OVER CURRENT");
            if (Curr_counter > 99)  
            {
                ESP_LOGE(TAG, "OVER CURRENT");
                alert_payload.alert_field.overcurrent = 1;	
            }
        }

        //validate whether the battery is charged
        if ((dyn_payload.vrect.f > VOLTAGE_FULL_THRESH) && (dyn_payload.irect.f < CURRENT_THRESH))
        {   
            ChargeComp_counter++;
            ESP_LOGI(TAG, "CHARGE COMPLETE");
            if (ChargeComp_counter > 199)
            {
                ESP_LOGE(TAG, "CHARGE COMPLETE");
                alert_payload.alert_field.charge_complete = 1;
            }
        }


        //simulate alert situation
        //alert_payload.alert_field.charge_complete = 1;

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
    int rc;

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_DIR;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.high_duty_cycle = 1;
    adv_params.itvl_min = 20;
    adv_params.itvl_max = 40;

    //declare MASTER ADDRESS
    ble_addr_t master;
    master.type = 0;
    
    master.val[0]= 0x12;
    master.val[1]= 0x15;
    master.val[2]= 0x9c;
    master.val[3]= 0x84;
    master.val[4]= 0x21;
    master.val[5]= 0x78;
/*
    master.val[0]= 0xb2;
    master.val[1]= 0xcb;
    master.val[2]= 0x25;
    master.val[3]= 0xfb;
    master.val[4]= 0x0b;
    master.val[5]= 0xac;
*/
    rc = ble_gap_adv_start(own_addr_type, &master, BLE_HS_FOREVER,
                           &adv_params, bleprph_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
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
    i2c_master_port = I2C_MASTER_NUM;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
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
            //SET MAX TX POWER
            esp_ble_tx_power_set(event->connect.conn_handle, ESP_PWR_LVL_P9); 
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

        // reset alerts counters
        Temp_counter = Volt_counter = Curr_counter = ChargeComp_counter = 0 ;

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
        //ESP_LOGI(TAG, "advertise complete; reason=%d",
        //            event->adv_complete.reason);
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
    alert_t_handle = xTimerCreate("alert", ALERT_PARAM_TIMER_INTERVAL, pdTRUE, NULL, alert_timeout_handler);

    if ((dynamic_t_handle == NULL) || (alert_t_handle == NULL))
    {
        ESP_LOGW(TAG, "Timers were not created successfully");
        return;
    }
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

    //SET MAX TX POWER
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);

    /* Initialize all elements of CRU (timers and I2C)*/
    init_setup();   

/*
    //INIT STATIC PAYLOAD 
    uint8_t mac[6] = {0};
    esp_efuse_mac_get_default(mac);
    ESP_LOGI(TAG, "MAC Address: \n ");
    ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x \n", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
*/
    /* just for testing I2C */
    //xTimerStart(dynamic_t_handle, 0);
}
