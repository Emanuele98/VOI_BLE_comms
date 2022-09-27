#include "nvs_flash.h"

/* NimBLE */
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "syscfg/syscfg.h"

#include "esp_err.h"

#include "ble_central.h"

struct timeval tv_start;

static const char* TAG = "MAIN";

void ble_store_config_init(void);

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

void write_nvs_partition(void)
{
    esp_err_t err;
    
    /* Create new NVS handle */
    nvs_handle_t nvs_handle;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        /* Write all elements needed in NVS partition */
        printf("Updating  in NVS ... ");

        err = nvs_set_u8(nvs_handle, "optional fields", CONFIG_CTU_STATIC_OP_FIELDS);
        printf((err != ESP_OK) ? "Failed writing optional fields!\n" : "Done\n");
        err = nvs_set_u8(nvs_handle, "CTU power", CONFIG_CTU_STATIC_POWER);
        printf((err != ESP_OK) ? "Failed writing CTU power!\n" : "Done\n");
        err = nvs_set_u8(nvs_handle, "max impedance", CONFIG_CTU_STATIC_MAX_IMPEDANCE);
        printf((err != ESP_OK) ? "Failed writing max impedance!\n" : "Done\n");
        err = nvs_set_u8(nvs_handle, "max load", CONFIG_CTU_STATIC_MAX_LOAD);
        printf((err != ESP_OK) ? "Failed writing max load!\n" : "Done\n");
        err = nvs_set_u16(nvs_handle, "RFU 1", CONFIG_CTU_STATIC_RFU_1);
        printf((err != ESP_OK) ? "Failed writing RFU 1!\n" : "Done\n");
        err = nvs_set_u8(nvs_handle, "CTU class", CONFIG_CTU_STATIC_CTU_CLASS);
        printf((err != ESP_OK) ? "Failed writing CTU class!\n" : "Done\n");
        err = nvs_set_u8(nvs_handle, "HW revision", CONFIG_CTU_STATIC_HW_REVISION);
        printf((err != ESP_OK) ? "Failed writing HW revision!\n" : "Done\n");
        err = nvs_set_u8(nvs_handle, "FW revision", CONFIG_CTU_STATIC_FW_REVISION);
        printf((err != ESP_OK) ? "Failed writing FW revision!\n" : "Done\n");
        err = nvs_set_u8(nvs_handle, "Prot revision", CONFIG_CTU_STATIC_PROT_REVISION);
        printf((err != ESP_OK) ? "Failed writing Prot revision!\n" : "Done\n");
        err = nvs_set_u8(nvs_handle, "Max devices", CONFIG_CTU_STATIC_MAX_DEVICES);
        printf((err != ESP_OK) ? "Failed writing Max devices!\n" : "Done\n");
        err = nvs_set_u16(nvs_handle, "Company ID", CONFIG_CTU_STATIC_COMPANY_ID);
        printf((err != ESP_OK) ? "Failed writing Max devices!\n" : "Done\n");
        err = nvs_set_u32(nvs_handle, "RFU 2", CONFIG_CTU_STATIC_RFU_2);
        printf((err != ESP_OK) ? "Failed writing RFU 2!\n" : "Done\n");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        printf("Committing updates in NVS ... ");
        err = nvs_commit(nvs_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(nvs_handle);
    }
    esp_restart();
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
    /* Initialize NVS partition */
    esp_err_t esp_err_code = nvs_flash_init();
    if  (esp_err_code == ESP_ERR_NVS_NO_FREE_PAGES || esp_err_code == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        esp_err_code = nvs_flash_init();
    }
    ESP_ERROR_CHECK(esp_err_code);

    /* Read NVS partition for constant data  */
    if (ble_central_get_CTU_static() != ESP_OK)
    {
       /* Write NVS partition if no configurations can be found */
       write_nvs_partition();
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

    /* XXX Need to have template for store */
    ble_store_config_init();

    /* Host management loop (Host context) */
    nimble_port_freertos_init(host_task);

    /* Initialize all elements of CTU */
    init_setup();

    gettimeofday(&tv_start, NULL);

    //ESP_LOGI(TAG, "%d", MAC);


    //ESP_LOGE(TAG, "RAM 0 left %d", esp_get_minimum_free_heap_size());

    /* Runtime function for main context */
    CTU_states_run();
}
