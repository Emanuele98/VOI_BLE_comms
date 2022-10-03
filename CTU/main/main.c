/* NimBLE */
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "syscfg/syscfg.h"
/* WiFI */
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/event_groups.h"
#include "esp_sntp.h"

#include "nvs_flash.h"
#include "esp_err.h"

#include "ble_central.h"

#define WIFI_SSID      CONFIG_WIFI_SSID
#define WIFI_PASS      CONFIG_WIFI_PASSWORD
#define WIFI_MAXIMUM_RETRY 5

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static int s_retry_num = 0;


struct timeval tv_start;
led_strip_t* strip;

time_t now;
struct tm *info;
char buffer[64];

static const char* TAG = "MAIN";

/**
 * @brief Handler for WiFi and IP events
 * 
 */
static void wifi_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Initialize wifi module
 * 
 */
static void wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();

    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
       number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
       happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s",
                 WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s",
                 WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_handler);
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_handler);
    vEventGroupDelete(s_wifi_event_group);
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
    CTU_state_change(CTU_CONFIG_STATE, NULL);

    ESP_LOGW(TAG, "Resetting state; reason=%d\n", reason);
}

/** 
 * @brief Host sync callback function
 * @details This function will handle a synchronisation event once the 
 *          NimBLE BLE stack ports on FreeRTOS.
*/
static void host_ctrl_on_sync(void)
{
    ESP_LOGI(TAG, "Device synced");

    /* Make sure we have proper identity address set (public preferred) */
    ble_hs_util_ensure_addr(0);

    /* Initialize configuration state */
    CTU_state_change(CTU_CONFIG_STATE, NULL);
}

/** 
 * @brief Initialization function for software timers
 * @details This init function will setup all software timers needed for 
 *          checking any local error as well as registering a new device.
*/
void init_sw_timers(void)
{
    /* Software timer for sequential switching of the charging pads */
    localization_switch_pads_t_handle = xTimerCreate("localization", PERIODIC_SWITCH_TIMER_PERIOD, pdTRUE, NULL, CTU_periodic_pad_switch);

    /* Software timer for periodic scanning once 1 CRU is connected */
    periodic_scan_t_handle = xTimerCreate("scan", PERIODIC_SCAN_TIMER_PERIOD, pdTRUE, NULL, CTU_periodic_scan_timeout);
}

/** 
 * @brief Initialization function for all CTU modules
 * @details This is an all encompassing function designed to initialize what
 *          the CTU needs to function regardless of its current mode of operation.
*/
void init_setup(void)
{
    
    /* Initialize software timers */
    init_sw_timers();

    /* Initialize state switch semaphore */
    m_set_state_sem = xSemaphoreCreateMutex();
    
    /* Initialize LED strip on PIN 15*/
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(15, RMT_CHANNEL_0);
    // set counter clock to 40MHz
    config.clk_div = 2;
    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
    //install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    strip->clear(strip, 10);

    //todo: delete the strip when the CTU resets is done
    //strip->del(strip);

    //Initialize WiFi module
    //wifi_init();
}

/** 
 * @brief Main function
 * @details This function handles the beginning of the program on reboot. Once it starts,
 *          both the BLE host core (CPU0) and the main core (CPU1) start with their respective
 *          tasks and configurations.
*/
void app_main(void)
{
    /* Initialize NVS partition */
    esp_err_t esp_err_code = nvs_flash_init();
    if  (esp_err_code == ESP_ERR_NVS_NO_FREE_PAGES || esp_err_code == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Bind HCI and controller to NimBLE stack */
    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());
    nimble_port_init();

    /* Configure the host. */
    ble_hs_cfg.reset_cb = host_ctrl_on_reset;
    ble_hs_cfg.sync_cb = host_ctrl_on_sync;

    /* Initialize data structures to track connected peers. */
    peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);

    /* Set the default device name. */
    ble_svc_gap_device_name_set("Airfuel CTU");

    /* Host management loop (Host context) */
    nimble_port_freertos_init(host_task);

    /* Initialize all elements of CTU */
    //todo: wait until this is complete
    init_setup();

    gettimeofday(&tv_start, NULL);


    //sntp_restart();
    //sntp_set_time_sync_notification_cb()

/*
    sntp_sync_time(&tv_start);
    now = tv_start.tv_sec;
    info = localtime(&now);
    printf("%s",asctime (info));
    strftime (buffer, sizeof buffer, "Today is %A, %B %d.\n", info);
    printf("%s",buffer);
    strftime (buffer, sizeof buffer, "The time is %I:%M %p.\n", info);
    printf("%s",buffer);
    vTaskDelay(5000);
*/
    //ESP_LOGE(TAG, "RAM 0 left %d", esp_get_minimum_free_heap_size());

    /* Runtime function for main context */
    CTU_states_run();
}
