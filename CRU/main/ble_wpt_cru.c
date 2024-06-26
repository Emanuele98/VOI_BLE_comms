#include "include/ble_wpt_cru.h"

/**********************************************************************************/
/**                       Static function declarations                           **/
/**********************************************************************************/

/* Debug tag */
static const char* TAG = "BLE_PHERIPERAL";

//*declare service
static const ble_uuid128_t wpt_service_uuid = BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0xFE, 0xFF, 0x55, 0x64);

//*declare characteristics
static const ble_uuid128_t WPT_PEER_STAT_UUID =
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x73, 0xE6, 0x55, 0x64);

static const ble_uuid128_t WPT_DYN_UUID = 
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x74, 0xE6, 0x55, 0x64);

static const ble_uuid128_t WPT_ALERT_UUID =
 BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x72, 0xE6, 0x55, 0x64);


//*CHR CALLBACK FUNCTIONS
static int gatt_svr_chr_read_peer_static(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);
    
static int gatt_svr_chr_read_dynamic(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);

static int gatt_svr_chr_notify_alert_dsc(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);

//static uint16_t alert_handle;


static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &wpt_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        {   {
                // Characteristic: cru static payload.
                .uuid = &WPT_PEER_STAT_UUID.u,
                .access_cb = gatt_svr_chr_read_peer_static,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                // Characteristic: cru dynamic payload.
                .uuid = &WPT_DYN_UUID.u,
                .access_cb = gatt_svr_chr_read_dynamic,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                // Characteristic: cru alert payload.
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
    ESP_LOGE(TAG, "READ PRU STATIC CALLBACK");

    int err_code = 0 ;
    uint8_t data[PRU_STATIC_CHAR_SIZE];

    //*READ ADDRESSES
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
    /*
        //INIT STATIC PAYLOAD 
        uint8_t mac[6] = {0};
        esp_efuse_mac_get_default(mac);
        ESP_LOGI(TAG, "MAC Address: \n ");
        ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x \n", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    */
        /* Printing ADDR */
        uint8_t ble_addr[6] = {0};
        ble_hs_id_copy_addr(0, ble_addr, NULL);

        static_payload.ble_addr0 = ble_addr[0];
        static_payload.ble_addr1 = ble_addr[1];
        static_payload.ble_addr2 = ble_addr[2];
        static_payload.ble_addr3 = ble_addr[3];
        static_payload.ble_addr4 = ble_addr[4];
        static_payload.ble_addr5 = ble_addr[5];

        data[0] = static_payload.ble_addr0;
        data[1] = static_payload.ble_addr1;
        data[2] = static_payload.ble_addr2;
        data[3] = static_payload.ble_addr3;
        data[4] = static_payload.ble_addr4;
        data[5] = static_payload.ble_addr5;
                                                    
        err_code = os_mbuf_append(ctxt->om, &data,
                            sizeof data);
    }
    //* WRITE ALERTS LIMITS
    else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        err_code = gatt_svr_chr_write(ctxt->om,
                    sizeof data,
                    sizeof data,
                    &data, NULL);
        
        static_payload.overcurrent.b[0] = data[0];
        static_payload.overcurrent.b[1] = data[1];
        static_payload.overcurrent.b[2] = data[2];
        static_payload.overcurrent.b[3] = data[3];
        static_payload.overvoltage = data[4];
        static_payload.overtemperature = data[5];
        ESP_LOGW(TAG, "static_payload.overcurrent: %.2f", static_payload.overcurrent.f);
        ESP_LOGW(TAG, "static_payload.overvoltage: %d", static_payload.overvoltage);
        ESP_LOGW(TAG, "static_payload.overtemperature: %d", static_payload.overtemperature);
    }

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
    //ESP_LOGE(TAG, "READ PRU DYNAMIC CALLBACK");

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
/*
    ESP_LOGW(TAG, "- DYN CHR - rx voltage = %.02f", dyn_payload.vrect.f);
    ESP_LOGW(TAG, "- DYN CHR - rx current = %.02f", dyn_payload.irect.f);
    ESP_LOGW(TAG, "- DYN CHR - rx temperature = %.02f", dyn_payload.temp1.f);
*/
    // set 0 to accelerometer after you sent it
    
    dyn_payload.RFU = 0;

    err_code = os_mbuf_append(ctxt->om, &data,
                        sizeof data);
    return err_code;
}


static int gatt_svr_chr_notify_alert_dsc(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    //ESP_LOGE(TAG, "WRITE ALERT CALLBACK");
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
