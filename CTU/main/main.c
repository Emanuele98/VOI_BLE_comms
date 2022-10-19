/* NimBLE */
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "syscfg/syscfg.h"

#include "nvs_flash.h"
#include "esp_err.h"

#include "include/ble_central.h"
#include "include/sd_card.h"
#include "include/wifi.h"
#include "include/DHT22.h"

static const char* TAG = "MAIN";

/**
 * @brief Init the values on NVS
 * 
 */
void init_NVS(void)
{
    //TODO: SET ONLY IF ITS DOES NOT EXIST YET! -- OTHERWISE GET
    //NVS reading
    esp_err_t err = nvs_open("reconnection", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else 
        {
            time(&now);
            timePad[0] = timePad[1] = timePad[2] = timePad[3] = timeScooter[0] = timeScooter[1] = timeScooter[2] = timeScooter[3] = now;

            esp_err_t err = nvs_get_i64(my_handle, "pad1", &timePad[0]);
            if (err == ESP_ERR_NVS_NOT_FOUND)
                    nvs_set_i64(my_handle, "pad1", timePad[0]);

            err = nvs_get_i64(my_handle, "pad2", &timePad[1]);
            if (err == ESP_ERR_NVS_NOT_FOUND)
                nvs_set_i64(my_handle, "pad2", timePad[1]);

            err = nvs_get_i64(my_handle, "pad3", &timePad[2]);
            if (err == ESP_ERR_NVS_NOT_FOUND)
                nvs_set_i64(my_handle, "pad3", timePad[2]);

            err = nvs_get_i64(my_handle, "pad4", &timePad[3]);
            if (err == ESP_ERR_NVS_NOT_FOUND)
                nvs_set_i64(my_handle, "pad4", timePad[3]);
            
            err = nvs_get_i64(my_handle, "3PAU", &timeScooter[VOI_3PAU]);
            if (err == ESP_ERR_NVS_NOT_FOUND)
                nvs_set_i64(my_handle, "3PAU", timeScooter[VOI_3PAU]);

            err = nvs_get_i64(my_handle, "6F35", &timeScooter[VOI_6F35]);
            if (err == ESP_ERR_NVS_NOT_FOUND)
                nvs_set_i64(my_handle, "6F35", timeScooter[VOI_6F35]);

            err = nvs_get_i64(my_handle, "CE8J", &timeScooter[VOI_CE8J]);
            if (err == ESP_ERR_NVS_NOT_FOUND)
                nvs_set_i64(my_handle, "CE8J", timeScooter[VOI_CE8J]);

            err = nvs_get_i64(my_handle, "D8X5", &timeScooter[VOI_D8X5]);
            if (err == ESP_ERR_NVS_NOT_FOUND)
                nvs_set_i64(my_handle, "D8X5", timeScooter[VOI_D8X5]);

            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "NVS INIT FAILED!\n" : "NVS INIT DONE\n");
            
            // Close
            nvs_close(my_handle);
        }
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

    /* Software timer for periodic temperature and humidity measurements */
    ambient_temp_handle = xTimerCreate("Temp", PERIODIC_AMBIENT_TEMP_TIMER, pdTRUE, NULL, CTU_ambient_temperature);

    // start periodic temperature and humidity measurements
    xTimerStart(ambient_temp_handle, 100);
}

/** 
 * @brief Initialization function for all CTU modules
 * @details This is an all encompassing function designed to initialize what
 *          the CTU needs to function regardless of its current mode of operation.
*/
void init_setup(void)
{   
    /* Install SD card */
    if (SD_CARD)
        install_sd_card();

    /* Initialize WiFi connectivity*/
    if (WIFI)
        connectivity_setup();

    /* Initialize software timers */
    init_sw_timers();    

    /* Init values on NVS */ 
    init_NVS();
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
        // NVS partition was truncated and needs to be erased
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
