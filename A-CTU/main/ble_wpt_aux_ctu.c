#include "include/ble_wpt_aux_ctu.h"

/**********************************************************************************/
/**                       Static function declarations                           **/
/**********************************************************************************/

/* Debug tag */
static const char* TAG = "BLE_PHERIPERAL";

// Declare ALL the WPT service and its characteristics UUIDs 
static const ble_uuid128_t WPT_SERVICE_UUID = 
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0xFE, 0xFF, 0x55, 0x64);

static const ble_uuid128_t WPT_PEER_STAT_UUID =
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x73, 0xE6, 0x55, 0x64);

static const ble_uuid128_t WPT_DYN_UUID = 
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x74, 0xE6, 0x55, 0x64);

static const ble_uuid128_t WPT_CONTROL_UUID =
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x70, 0xE6, 0x55, 0x64);

static const ble_uuid128_t WPT_ALERT_UUID =
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x72, 0xE6, 0x55, 0x64);


//*CHR CALLBACK FUNCTIONS
static int gatt_svr_chr_read_peer_static(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);
    
static int gatt_svr_chr_read_dynamic(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);

static int gatt_svr_chr_write_control(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);

static int gatt_svr_chr_notify_alert_dsc(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);


static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &WPT_SERVICE_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[])
        {   {
                // Characteristic: a-ctu static payload.
                .uuid = &WPT_PEER_STAT_UUID.u,
                .access_cb = gatt_svr_chr_read_peer_static,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                // Characteristic: a-ctu dynamic payload.
                .uuid = &WPT_DYN_UUID.u,
                .access_cb = gatt_svr_chr_read_dynamic,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                // Characteristic: a-ctu control payload.
                .uuid = &WPT_CONTROL_UUID.u,
                .access_cb = gatt_svr_chr_write_control,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                // Characteristic: a-ctu alert payload.
                .uuid = &WPT_ALERT_UUID.u,
                .access_cb = gatt_svr_chr_notify_alert_dsc,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                0, // No more characteristics in this service.
            }
        },
    },

    {
        0, // No more services. 
    },
};

static int
gatt_svr_chr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
                   void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
    ESP_LOGW(TAG, "err! length=%d", om_len);

        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

static int gatt_svr_chr_read_peer_static(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    ESP_LOGE(TAG, "READ STATIC CALLBACK");

    int err_code ;
    
    //INIT STATIC PAYLOAD 
    uint8_t mac[6] = {0};
    esp_efuse_mac_get_default(mac);
    ESP_LOGI(TAG, "MAC Address: \n ");
    ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x \n", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

    static_payload.mac_0 = mac[0];
    static_payload.mac_1 = mac[1];
    static_payload.mac_2 = mac[2];
    static_payload.mac_3 = mac[3];
    static_payload.mac_4 = mac[4];
    static_payload.mac_5 = mac[5];

    uint8_t data[PRU_STATIC_CHAR_SIZE] = { static_payload.mac_0,
                                           static_payload.mac_1,
                                           static_payload.mac_2,
                                           static_payload.mac_3,
                                           static_payload.mac_4,
                                           static_payload.mac_5 }; // 6 bytes of data
                                                
    err_code = os_mbuf_append(ctxt->om, &data,
                        sizeof data);

    //Start app timers
    xTimerStart(dynamic_t_handle, 0);
    xTimerStart(alert_t_handle, 0);

    //set alert values to initial state 0
    alert_payload.alert_field.overtemperature = 0; 
    alert_payload.alert_field.overvoltage = 0;
    alert_payload.alert_field.overcurrent = 0;
    alert_payload.alert_field.FOD = 0;
    alert_payload.alert_field.charge_complete = 0;

    return err_code;
}

static int gatt_svr_chr_read_dynamic(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    //ESP_LOGE(TAG, "READ DYNAMIC CALLBACK");

    int err_code;

	uint8_t data[PRU_DYNAMIC_CHAR_SIZE] = { dyn_payload.vrect.b[0],
                                            dyn_payload.vrect.b[1],
                                            dyn_payload.vrect.b[2],
											dyn_payload.vrect.b[3],
											dyn_payload.irect.b[0],
											dyn_payload.irect.b[1],
											dyn_payload.irect.b[2],
											dyn_payload.irect.b[3],		
											dyn_payload.temp1.b[0],
											dyn_payload.temp1.b[1],
											dyn_payload.temp1.b[2],
											dyn_payload.temp1.b[3],
                                            dyn_payload.temp2.b[0],
											dyn_payload.temp2.b[1],
											dyn_payload.temp2.b[2],
											dyn_payload.temp2.b[3],
											dyn_payload.alert,
											dyn_payload.RFU}; // 18 bytes of data

    //ESP_LOGW(TAG, "- DYN CHR - rx voltage = %.02f", dyn_payload.vrect.f);
    //ESP_LOGW(TAG, "- DYN CHR - rx voltage = %.02f", dyn_payload.irect.f);
    //ESP_LOGW(TAG, "- DYN CHR - rx temperature = %.02f", dyn_payload.temp1.f);
    //ESP_LOGW(TAG, "- DYN CHR - rx temperature = %.02f", dyn_payload.temp2.f);

    //ESP_LOGE(TAG, "power: %d",esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_CONN_HDL0));

    err_code = os_mbuf_append(ctxt->om, &data,
                        sizeof data);
    return err_code;
}

static int gatt_svr_chr_write_control(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{

    ESP_LOGE(TAG, "WRITE CONTROL CALLBACK");

    int err_code = 0 ;

    //uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om); // 5

    uint8_t data[PRU_CONTROL_CHAR_SIZE];

    err_code = gatt_svr_chr_write(ctxt->om,
                            sizeof data,
                            sizeof data,
                            &data, NULL);
    
    control_payload.enable = data[0];
	control_payload.full_power = data[1];
    control_payload.critical = data[2];
	control_payload.RFU = ((uint16_t)data[3]<<8)|data[4];

    if(control_payload.critical)
    {
        disable_low_power_output();
        vTaskDelay(OR_TIME_GAP / portTICK_PERIOD_MS);
        enable_full_power_output();
        } else
            {
            if(control_payload.enable)
            {
                //disable OR gate
                disable_OR_output();
                //wait 
                vTaskDelay(OR_TIME_GAP / portTICK_PERIOD_MS);
                //enable power
                if(control_payload.full_power) {
                enable_full_power_output();
                } else {
                enable_low_power_output();
                }
            } else {
                    //disable power
                    if(control_payload.full_power) {
                    disable_full_power_output();
                    } else {
                    disable_low_power_output();
                    }
                    //wait
                    vTaskDelay(OR_TIME_GAP / portTICK_PERIOD_MS);
                    //enable OR gate
                    enable_OR_output();
                }
            }
    return err_code;
}

static int gatt_svr_chr_notify_alert_dsc(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    ESP_LOGE(TAG, "WRITE ALERT CALLBACK");
    int err_code = 0 ;

    uint8_t data[ALERT_CHAR_SIZE];

    data[0] = alert_payload.alert_field.internal;

    // read alert chr
    err_code = os_mbuf_append(ctxt->om, &data,
                        sizeof data);
    if (err_code != ESP_OK)
    {
        return err_code;
    }

    //NOTIFY alert chr
    struct os_mbuf *om;
    om = ble_hs_mbuf_from_flat(data, sizeof(data));

    err_code = ble_gattc_notify_custom(conn_handle, attr_handle, om);


    return err_code;
}



int gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }
    

    return 0;
}
