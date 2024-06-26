//The main resides on the APP_CPU (Core 1). 
//The host refers to another context, one that handles the NimBLE stack and its various states.
// The host resides on the PRO_CPU (Core 0).

#include "esp_err.h"
#include "ble_wpt_cru.h"
#include "cru_hw.h"
#include "lis3dh.h"

/* accelerometer */

#define INT_EVENT    // inertial event interrupts used (wake-up, free fall or 6D/4D orientation)
#define INT_USED
#define INT1_PIN      34

// user task stack depth for ESP32
#define TASK_STACK_DEPTH 2048

static lis3dh_sensor_t* sensor;

/**
 * Common function used to get sensor data.
 */
void read_data ()
{
    lis3dh_float_data_t  data;

    if (lis3dh_new_data (sensor) &&
        lis3dh_get_float_data (sensor, &data))
        // max. full scale is +-16 g and best resolution is 1 mg, i.e. 5 digits
        printf("%.3f LIS3DH (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
               (double)sdk_system_get_time()*1e-3, 
                data.ax, data.ay, data.az);
}

/**
 * In this case, any of the possible interrupts on interrupt signal *INT1* is
 * used to fetch the data.
 *
 * When interrupts are used, the user has to define interrupt handlers that
 * either fetches the data directly or triggers a task which is waiting to
 * fetch the data. In this example, the interrupt handler sends an event to
 * a waiting task to trigger the data gathering.
 */

static xQueueHandle gpio_evt_queue = NULL;

// User task that fetches the sensor values.

void user_task_interrupt (void *pvParameters)
{
    uint8_t gpio_num;

    while (1)
    {
        //* MOTION WAKE UP INTERRUPT USED *//
        
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY))
        {
            lis3dh_int_event_source_t event_src = {};

            // get the source of the interrupt and reset *INTx* signals
            lis3dh_get_int_event_source (sensor, &event_src, lis3dh_int_event1_gen);
 
            // in case of event interrupt
            if (event_src.active)
            {
                //if (event_src.x_low)  printf("x is lower than threshold\n");
                //if (event_src.y_low)  printf("y is lower than threshold\n");
                //if (event_src.z_low)  printf("z is lower than threshold\n");
                //if (event_src.x_high) printf("x is higher than threshold\n");
                //if (event_src.y_high) printf("y is higher than threshold\n");
                //if (event_src.z_high) printf("z is higher than threshold\n");

                if (event_src.x_high || event_src.y_high || event_src.z_high)
                {
                    printf("Motion detected\n");
                    dyn_payload.RFU = 1;
                }
            }
        }
        
        //* READ DATA FROM SENSOR *//
        /*
        // read sensor data
        read_data ();
        
        // passive waiting until 1 second is over
        vTaskDelay(1000/portTICK_PERIOD_MS);
        */
    }
}

// Interrupt handler which resumes user_task_interrupt on interrupt

void IRAM int_signal_handler (uint8_t gpio)
{
    // send an event with GPIO to the interrupt user task
    xQueueSendFromISR(gpio_evt_queue, &gpio, NULL);
}




/*******************************************************************/
/**                           DEFINES                             **/
/*******************************************************************/

#define DEVICE_NAME                     "CRU"                          /**< Name of device. Will be included in the advertising data. */

#define WPT_SVC_UUID16               0xFFFE

static const char* TAG = "MAIN";

static int counter = 0, Temp_counter = 0, Volt_counter = 0, Curr_counter = 0, ChargeComp_counter = 0;

int i2c_master_port;
i2c_config_t conf;

//nimBLE
static uint8_t own_addr_type;
static int bleprph_gap_event(struct ble_gap_event *event, void *arg);


//read dynamic parameters from ADC sensors
static void get_adc(void *arg)
{
    static float volt = 0, curr = 0;
    static int adc_counter = 0;

    while(1)
    {
        adc_counter++;
        
        //voltage
        volt += adc_read_voltage_sensor();
        //current
        curr += adc_read_current_sensor();         
        
        //Multisampling
        if (adc_counter == NO_OF_SAMPLES)
        {
            //average
            dyn_payload.vrect.f = volt / NO_OF_SAMPLES;
            dyn_payload.irect.f = curr / NO_OF_SAMPLES;

            volt = 0;
            curr = 0;
            adc_counter = 0;

            //ESP_LOGW(TAG, "Voltage: %.2f, Current: %.2f", dyn_payload.vrect.f, dyn_payload.irect.f );
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        /* Feed watchdog */
        esp_task_wdt_reset();
    }

    //if for any reason the loop terminates
    vTaskDelete(NULL); 
}

//read dynamic parameters from I2C sensors
static void get_temp(void *arg)
{
    while (1)
    {
        float t1 = 0, t2 = 0;
        if (xSemaphoreTake(i2c_sem, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            switch(counter)
            {
                //temperature 1
                case 0:
                    counter++;
                    t1 = i2c_read_temperature_sensor(0);
                    if (t1 != -1)
                        dyn_payload.temp1.f = t1;
                    break;

                //temperature 2
                case 1:
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

    //if for any reason the loop terminates
    vTaskDelete(NULL); 
}

static void alert_timeout_handler(void *arg)
{      
    // Validate temperature levels
    if ((dyn_payload.temp1.f > static_payload.overtemperature) || (dyn_payload.temp2.f > static_payload.overtemperature))
    {
        Temp_counter++;
        //ESP_LOGI(TAG, "OVER TEMPERATURE");
        if (Temp_counter > 99) 
        {
            ESP_LOGE(TAG, "OVER TEMPERATURE");
            alert_payload.alert_field.overtemperature = 1;
        } 
    }

    // Validate voltage levels
    if (dyn_payload.vrect.f > static_payload.overvoltage)
    {
        Volt_counter++;
        if (Volt_counter > 5) 
        {
            ESP_LOGE(TAG, "OVER VOLTAGE");
            alert_payload.alert_field.overvoltage = 1;
        }	
    }

    // Validate current levels
    if (dyn_payload.irect.f > static_payload.overcurrent.f)
    {
        Curr_counter++;
        //ESP_LOGI(TAG, "OVER CURRENT");
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
        //ESP_LOGI(TAG, "CHARGE COMPLETE");
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
    adv_params.itvl_min = 0x20;
    adv_params.itvl_max = 0x40;

    //declare MASTER ADDRESS
    ble_addr_t master;
    master.type = 0;
    
/*
    master.val[0]= 0xde;
    master.val[1]= 0x52;
    master.val[2]= 0xa9;
    master.val[3]= 0xb2;
    master.val[4]= 0x43;
    master.val[5]= 0x54;
*/
    /* test */
    master.val[0]= 0x3e;
    master.val[1]= 0xac;
    master.val[2]= 0xc0;
    master.val[3]= 0xe2;
    master.val[4]= 0xde;
    master.val[5]= 0xc4;

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
    /* INIT OUTPUT PIN */
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins
    io_conf.pin_bit_mask = (1ULL<<25);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    //SET IT LOW
    gpio_set_level(25, 0);

    /* Init adc */
    init_adc();

    /* init I2C*/
    esp_err_t err_code;

    i2c_master_port = I2C_MASTER_NUM;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_400K;
    err_code = i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    ESP_ERROR_CHECK(err_code);

    /* init acceleremoter task */
    // init the sensor with slave address LIS3DH_I2C_ADDRESS_1 connected to I2C_BUS.
    sensor = lis3dh_init_sensor (I2C_MASTER_NUM, LIS3DH_I2C_ADDRESS_1, 0);

    if (sensor) 
    {
        /** --- INTERRUPT CONFIGURATION PART ---- */
        
        // Interrupt configuration has to be done before the sensor is set
        // into measurement mode to avoid losing interrupts

        // create an event queue to send interrupt events from interrupt
        // handler to the interrupt task
        gpio_evt_queue = xQueueCreate(10, sizeof(uint8_t));

        // configure interupt pins for *INT1* and *INT2* signals and set the interrupt handler
        gpio_enable(INT1_PIN, GPIO_INPUT);
        gpio_set_interrupt(INT1_PIN, GPIO_INTTYPE_EDGE_POS, int_signal_handler);
        
        /** -- SENSOR CONFIGURATION PART --- */

        // set polarity of INT signals if necessary
        // lis3dh_config_int_signals (sensor, lis3dh_high_active);
        
        // enable data interrupts on INT1 
        lis3dh_int_event_config_t event_config;
    
        event_config.mode = lis3dh_wake_up;
        event_config.threshold = 10;
        event_config.x_low_enabled  = false;
        event_config.x_high_enabled = true;
        event_config.y_low_enabled  = false;
        event_config.y_high_enabled = true;
        event_config.z_low_enabled  = false;
        event_config.z_high_enabled = true;
        event_config.duration = 0; 
        event_config.latch = true;
        
        lis3dh_set_int_event_config (sensor, &event_config, lis3dh_int_event1_gen);
        lis3dh_enable_int (sensor, lis3dh_int_event1, lis3dh_int1_signal, true);


        // configure HPF and reset the reference by dummy read
        lis3dh_config_hpf (sensor, lis3dh_hpf_normal, 0, true, true, true, true);
        lis3dh_get_hpf_ref (sensor);
        
        // enable ADC inputs and temperature sensor for ADC input 3
        lis3dh_enable_adc (sensor, true, true);
        
        // LAST STEP: Finally set scale and mode to start measurements
        lis3dh_set_scale(sensor, lis3dh_scale_2_g);
        lis3dh_set_mode (sensor, lis3dh_odr_10, lis3dh_high_res, true, true, true);

        /** -- TASK CREATION PART --- */

        // must be done last to avoid concurrency situations with the sensor
        // configuration part

        // create a task that is triggered only in case of interrupts to fetch the data
        xTaskCreate(user_task_interrupt, "user_task_interrupt", TASK_STACK_DEPTH, NULL, 2, NULL);
        
    }
    else
        printf("Could not initialize LIS3DH sensor\n");
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
        } else
        {
            /* Connection failed; resume advertising. */
            bleprph_advertise();
        }

        // reset alerts counters
        Temp_counter = Volt_counter = Curr_counter = ChargeComp_counter = 0 ;

        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnect; reason=%d ", event->disconnect.reason);

        /* Connection terminated; resume advertising. */
        bleprph_advertise();

        break;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        ESP_LOGI(TAG, "connection updated; status=%d ",
                    event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        assert(rc == 0);
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "advertise complete; reason=%d",
                    event->adv_complete.reason);
        bleprph_advertise();
        break;
    
    default:
        //ESP_LOGI(TAG, "OTHERS");
        break;
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
    ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x \n", addr_val[0], addr_val[1], addr_val[2], addr_val[3], addr_val[4], addr_val[5]);

    /* Begin advertising. */
    bleprph_advertise();
}


/** 
 * @brief Initialization function for software timers
*/
void init_sw_timers(void)
{
    uint8_t err;
    TimerHandle_t alert_t_handle;
    
    // Create timers
    alert_t_handle = xTimerCreate("alert", ALERT_PARAM_TIMER_INTERVAL, pdTRUE, NULL, alert_timeout_handler);

    if (alert_t_handle == NULL)
    {
        ESP_LOGW(TAG, "Timers were not created successfully");
        return;
    }

    //Start alert timer
    xTimerStart(alert_t_handle, 10);

    // create tasks to get measurements as fast as possible
    
    err = xTaskCreatePinnedToCore(get_temp, "get_temp", 2048, NULL, 5, NULL, 1);
    if ( err != pdPASS )
    {
        ESP_LOGE(TAG, "Task get_temp was not created successfully");
        return;
    }
    
    err = xTaskCreatePinnedToCore(get_adc, "get_adc", 5000, NULL, 5, NULL, 1);
    if( err != pdPASS )
    {
        ESP_LOGE(TAG, "Task get_ was not created successfully");
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

}
