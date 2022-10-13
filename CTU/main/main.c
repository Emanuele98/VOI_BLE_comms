/* NimBLE */
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "syscfg/syscfg.h"

#include "nvs_flash.h"
#include "esp_err.h"

#include "include/ble_central.h"
#include "include/led_strip.h"
#include "include/sd_card.h"
#include "include/wifi.h"

static const char* TAG = "MAIN";

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
    vTaskDelay(2000);
    CTU_state_change(CTU_CONFIG_STATE, (void *)NULL);

    ESP_LOGW(TAG, "Resetting state; reason=%d\n", reason);
    //todo: reset happens here (guru meditation error)
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
    CTU_state_change(CTU_CONFIG_STATE, (void *)NULL);
}

/** 
 * @brief Initialization function for software timers
 * 
*/
void init_sw_timers(void)
{
     /* Software timer for periodic scanning once 1 CRU is connected */
    periodic_scan_t_handle = xTimerCreate("scan", PERIODIC_SCAN_TIMER_PERIOD, pdTRUE, NULL, CTU_periodic_scan_timeout);

    //todo: remove 
    /* Software timer for led strip default state */
    periodic_leds_handle = xTimerCreate("leds", PERIODIC_LEDS_TIMER_PERIOD, pdTRUE, NULL, CTU_periodic_leds_blink);

    //xTimerStart(periodic_leds_handle, 10);
}

/** 
 * @brief Initialization function for all CTU modules
 * @details This is an all encompassing function designed to initialize what
 *          the CTU needs to function regardless of its current mode of operation.
*/
void init_setup(void)
{    
    /* Install leds */
    install_strip(STRIP_1_PIN, 0);
    install_strip(STRIP_2_PIN, 1);
    install_strip(STRIP_3_PIN, 2);
    install_strip(STRIP_4_PIN, 3);

    /* Install SD card */
    if (SD_CARD)
        install_sd_card();

    /* Initialize WiFi connectivity*/
    if (WIFI)
        connectivity_setup();

    /* Initialize software timers */
    init_sw_timers();    

    /* Initialize state switch semaphore */
    //m_set_state_sem = xSemaphoreCreateMutex();
}

/** 
 * @brief Main function
 * @details This function handles the beginning of the program on reboot. Once it starts,
 *          both the BLE host core (CPU0) and the main core (CPU1) start with their respective
 *          tasks and configurations.
*/

//todo: use ESP_ERROR_CHECK_WITHOUT_ABORT() everywhere

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    
    /* Initialize NVS partition */
    esp_err_t esp_err_code = nvs_flash_init();
    if  (esp_err_code == ESP_ERR_NVS_NO_FREE_PAGES || esp_err_code == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Initialize all elements of CTU */
    init_setup();

    /* Bind HCI and controller to NimBLE stack */
    esp_nimble_hci_and_controller_init();
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

    //SET MAX TX POWER
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9); 

    //TODO:
    //check esp_bt_sleep_enable()
 
    /* Runtime function for main context */
    CTU_states_run();
}
