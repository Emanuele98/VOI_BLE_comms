//The main resides on the APP_CPU (Core 1). 
//The host refers to another context, one that handles the NimBLE stack and its various states.
// The host resides on the PRO_CPU (Core 0).

#include "esp_err.h"
#include "ble_wpt_aux_ctu.h"

#include "include/aux_ctu_hw.h"
#include "include/led_strip.h"


struct timeval tv_start;



/*******************************************************************/
/**                           DEFINES                             **/
/*******************************************************************/

#define DEVICE_NAME                     "AUX_CTU"                              /**< Name of device. Will be included in the advertising data. */

#define DYNAMIC_PARAM_TIMER_INTERVAL    pdMS_TO_TICKS(20)                      /**< Timer synced to Dynamic parameter characteristic (10 ms). */
#define ALERT_PARAM_TIMER_INTERVAL      pdMS_TO_TICKS(60)				       /**< Timer synced to Alert parameter characteristic (40 ms). */

/* WPT SERVICE BEING ADVERTISED */
#define WPT_SVC_UUID16                  0xFFFE

static const char* TAG = "MAIN";

static float volt, curr, t1, t2;
static int counter = 0, Temp_counter = 0, Volt_counter = 0, Curr_counter = 0;

// queue to handle gpio event from isr
xQueueHandle gpio_evt_queue = NULL;


//nimBLE
static uint8_t own_addr_type;
static int bleprph_gap_event(struct ble_gap_event *event, void *arg);


void switch_safely_off(void)
{
    //disable interfaces
    disable_full_power_output();
    disable_low_power_output();
    //wait
    vTaskDelay(OR_TIME_GAP);
    //enable OR gate
    enable_OR_output();
}

static void gpio_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if(full_power) {
                ESP_LOGE(TAG, "FOD!");
                FOD_counter++;
                if (FOD_counter > 199)
                {
                    switch_safely_off();
                    alert_payload.alert_field.FOD = 1;
                    // RED LEDS
                    strip_enable = false;
                    strip_misalignment = false;
                    strip_charging = false;
                    set_strip(200, 0, 0);
                    vTaskDelay(1000);
                }
            }
        }
    }
}

//read dynamic parameters from I2C sensors
void dynamic_param_timeout_handler(void *arg)
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
                counter++;
                t1 = i2c_read_temperature_sensor(0);
                if (t1 != -1)
                    dyn_payload.temp1.f = t1;
                break;

            //temperature 2
            case 3:
                counter = 0;
                t2 = i2c_read_temperature_sensor(1);
                if (t2 != -1)
                    dyn_payload.temp2.f = t2;
                break;
            default:
                xSemaphoreGive(i2c_sem);
                break;
        }
    }
}

// close power interfaces here to add safety redundacy
// check if the value is greater 10 times (100 ms)

void alert_timeout_handler(void *arg)
{      
  		// Validate temperature levels
		if ((dyn_payload.temp1.f > OVER_TEMPERATURE) || (dyn_payload.temp2.f > OVER_TEMPERATURE))
		{
            Temp_counter++;
            if (Temp_counter > 10)
            {
                alert_payload.alert_field.overtemperature = 1;
                switch_safely_off();
            } 
    	}

        // Validate voltage levels
		if (dyn_payload.vrect.f > OVER_VOLTAGE)
		{
            Volt_counter++;
            if (Volt_counter > 5)
            {
                alert_payload.alert_field.overvoltage = 1;
                switch_safely_off();
            }	
        }

        // Validate current levels
		if (dyn_payload.irect.f > OVER_CURRENT)
		{
            Curr_counter++;
            if (Curr_counter > 10)
            {
                alert_payload.alert_field.overcurrent = 1;	
                switch_safely_off();
            }
        }

        /* just for TESTING */
        //alert_payload.alert_field.overcurrent = 1;

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

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        
        strip_enable = true;
        
        ESP_LOGI(TAG, "connection %s; status=%d; handle= %d",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status, event->connect.conn_handle);
        if (event->connect.status == 0) {
            ble_gap_conn_find(event->connect.conn_handle, &desc);
            //SET MAX TX POWER
            esp_ble_tx_power_set(event->connect.conn_handle, ESP_PWR_LVL_P9); 
        } else {
            /* Connection failed; resume advertising. */
            bleprph_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:

        ESP_LOGI(TAG, "disconnect; reason=%d ", event->disconnect.reason);

        switch_safely_off();

        if ((!alert) && (!charge_complete))
        {
            strip_enable = false;
            strip_misalignment = false;
            strip_charging = false;
            set_strip(0, 100, 100);
        }

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
        Temp_counter = Volt_counter = Curr_counter = 0;

        /* Connection terminated; resume advertising. */
        bleprph_advertise();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        ESP_LOGI(TAG, "connection updated; status=%d ",
                    event->conn_update.status);
        ble_gap_conn_find(event->conn_update.conn_handle, &desc);
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

    //uint8_t mac[6] = {0};
    //esp_efuse_mac_get_default(mac);
    //ESP_LOGI(TAG, "MAC Address: \n ");
    //ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x \n", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

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
    /* Software timer for led strip default state */
    connected_leds_handle = xTimerCreate("connected_leds", CONNECTED_LEDS_TIMER_PERIOD, pdTRUE, NULL, connected_leds);
    misaligned_leds_handle = xTimerCreate("misaligned_leds", MISALIGNED_LEDS_TIMER_PERIOD, pdTRUE, NULL, misaligned_leds);
    charging_leds_handle = xTimerCreate("charging leds", CHARGING_LEDS_TIMER_PERIOD, pdTRUE, NULL, charging_state);

    if ((dynamic_t_handle == NULL) || (alert_t_handle == NULL) || (connected_leds_handle == NULL) || (misaligned_leds_handle == NULL))
    {
        ESP_LOGW(TAG, "Timers were not created successfully");
        return;
    }

    xTimerStart(connected_leds_handle, 0);
    xTimerStart(misaligned_leds_handle, 0);
    xTimerStart(charging_leds_handle, 0);

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

    install_strip(STRIP_PIN);

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

    //OUTPUT PINS
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //INPUT PIN
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_HIGH_LEVEL;
    //bit mask of the pins
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(FOD_FPGA, gpio_isr_handler, (void*) FOD_FPGA);

    switch_safely_off();
}

/** 
 * @brief Initialization function for all PTU modules
 * @details This is an all encompassing function designed to initialize what
 *          the PTU needs to function regardless of its current mode of operation.
*/
void init_setup(void)
{    
    /* Initialize GPIOs and other hardware peripherals (I2C) */
    init_hw();

    /* Initialize software timers */
    init_sw_timers();

    /* Initialize I2C semaphore */
    i2c_sem = xSemaphoreCreateMutex();
}

void on_reset(void)
{
    ESP_LOGI(TAG, "on_reset");
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
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Bind HCI and controller to NimBLE stack */
    esp_nimble_hci_and_controller_init();
    nimble_port_init();

    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = host_ctrl_on_reset;
    ble_hs_cfg.sync_cb = host_ctrl_on_sync;

    rc = gatt_svr_init();
    if (rc!=0){
    esp_restart();
    }

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc!=0){
    esp_restart();
    }

    /* Host management loop (Host context) */
    nimble_port_freertos_init(host_task);

    /* Initialize all elements of AUX CTU (timers and I2C)*/
    init_setup();

    gettimeofday(&tv_start, NULL);

    //ESP_LOGI(TAG, "%d", esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV));

    //SET MAX TX POWER
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);


    /*uncomment to test the I2C without the need of being connected to the central unit */
    //xTimerStart(dynamic_t_handle, 0);

/*
    //INIT STATIC PAYLOAD 
    uint8_t mac[6] = {0};
    esp_efuse_mac_get_default(mac);
    ESP_LOGI(TAG, "MAC Address: \n ");
    ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x \n", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
*/

/*
    while(1){
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(esp_register_shutdown_handler(&on_reset));
        esp_restart();
    }
*/
}
