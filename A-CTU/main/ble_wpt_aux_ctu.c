#include "include/ble_wpt_aux_ctu.h"

/**********************************************************************************/
/**                       Static function declarations                           **/
/**********************************************************************************/

/* Debug tag */
static const char* TAG = "BLE_PHERIPERAL";

// Declare ALL the WPT service and its characteristics UUIDs 
static const ble_uuid128_t WPT_SERVICE_UUID = 
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0xFE, 0xFF, 0x55, 0x64);

static const ble_uuid128_t WPT_PRU_STAT_UUID =
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x73, 0xE6, 0x55, 0x64);

static const ble_uuid128_t WPT_PTU_STAT_UUID =
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x71, 0xE6, 0x55, 0x64);

static const ble_uuid128_t WPT_PRU_DYN_UUID = 
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x74, 0xE6, 0x55, 0x64);

static const ble_uuid128_t WPT_CONTROL_UUID =
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x70, 0xE6, 0x55, 0x64);

static const ble_uuid128_t WPT_ALERT_UUID =
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x72, 0xE6, 0x55, 0x64);


//*CHR CALLBACK FUNCTIONS
static int gatt_svr_chr_read_pru_static(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);

static int gatt_svr_chr_write_ptu_static(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);
    
static int gatt_svr_chr_read_pru_dynamic(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);

static int gatt_svr_chr_write_pru_control(uint16_t conn_handle, uint16_t attr_handle,
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
                // Characteristic: pru static payload.
                .uuid = &WPT_PRU_STAT_UUID.u,
                .access_cb = gatt_svr_chr_read_pru_static,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                // Characteristic: ptu static payload.
                .uuid = &WPT_PTU_STAT_UUID.u,
                .access_cb = gatt_svr_chr_write_ptu_static,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                // Characteristic: pru dynamic payload.
                .uuid = &WPT_PRU_DYN_UUID.u,
                .access_cb = gatt_svr_chr_read_pru_dynamic,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                // Characteristic: pru control payload.
                .uuid = &WPT_CONTROL_UUID.u,
                .access_cb = gatt_svr_chr_write_pru_control,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                // Characteristic: pru alert payload.
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

static int gatt_svr_chr_write_ptu_static(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    ESP_LOGE(TAG, "WRITE PTU STATIC CALLBACK");

    int err_code ;

    uint8_t data[PTU_STATIC_CHAR_SIZE];
    //todo: fill parameteres

    err_code = gatt_svr_chr_write(ctxt->om,
                            sizeof data,
                            sizeof data,
                            &data, NULL);
    
    ptu_static_payload.optional_fields = data[0];
    ptu_static_payload.ptu_power = data[1];
	ptu_static_payload.max_impedance = data[2];
	ptu_static_payload.max_load = data[3];
	ptu_static_payload.RFU1 = ((uint16_t)data[4]<<8)|data[5];
	ptu_static_payload.ptu_class = data[6];
	ptu_static_payload.hard_rev = data[7];
	ptu_static_payload.firm_rev = data[8];
	ptu_static_payload.protocol_rev = data[9];
	ptu_static_payload.max_num_devices = data[10];
	ptu_static_payload.company_id = (((uint16_t)data[11])<<8)|data[12];
	ptu_static_payload.RFU2 = (((uint32_t)data[13])<<24)|
							(((uint32_t)data[13])<<16)|
							(((uint32_t)data[13])<<8)|
							data[13];  // 17bites of data

    //Start app timers
    xTimerStart(dynamic_t_handle, 0);
    xTimerStart(alert_t_handle, 0);

    return err_code;

}


static int gatt_svr_chr_read_pru_static(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    ESP_LOGE(TAG, "READ STATIC CALLBACK");

    int err_code ;

    //INIT STATIC PAYLOAD 
    static_payload.optional_fields = OPTIONAL_FIELDS_STAT;
	static_payload.protocol_rev = PROTOCOL_REVISION;
	static_payload.pru_cat = PRU_CATEGORY;
	static_payload.pru_info = PRU_INFORMATION;
	static_payload.hard_rev = PRU_HARD_REVISION;
	static_payload.firm_rev = PRU_FIRM_REVISION;
	static_payload.prect_max = PRECT_MAXIMUM;
	static_payload.company_id = COMPANY_ID;

    unsigned char *rfu_p = (unsigned char*)&static_payload.RFU2;

    uint8_t data[PRU_STATIC_CHAR_SIZE] = {static_payload.optional_fields,
    											static_payload.protocol_rev,
                                                static_payload.RFU1,
											    static_payload.pru_cat,
											    static_payload.pru_info,
											    static_payload.hard_rev,
											    static_payload.firm_rev,
											    static_payload.prect_max,
											    static_payload.vrect_min_stat >> 8,
											    static_payload.vrect_min_stat & 0x00FF,
											    static_payload.vrect_high_stat >> 8,
											    static_payload.vrect_high_stat & 0x00FF,
											    static_payload.vrect_set >> 8,
											    static_payload.vrect_set & 0x00FF,
											    static_payload.company_id >> 8,
											    static_payload.company_id & 0x00FF,
											    rfu_p[0],rfu_p[1],rfu_p[2],rfu_p[3]}; // 20 bytes of data
                                                
    err_code = os_mbuf_append(ctxt->om, &data,
                        sizeof data);
    return err_code;
}

static int gatt_svr_chr_read_pru_dynamic(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    ESP_LOGE(TAG, "READ DYNAMIC CALLBACK");

    int err_code;

	uint8_t data[PRU_DYNAMIC_CHAR_SIZE] = { dyn_payload.vrect.b[0],
                                            dyn_payload.vrect.b[1],
                                            dyn_payload.vrect.b[2],
											dyn_payload.vrect.b[3],
											dyn_payload.irect.b[0],
											dyn_payload.irect.b[1],
											dyn_payload.irect.b[2],
											dyn_payload.irect.b[3],		
											dyn_payload.temp.b[0],
											dyn_payload.temp.b[1],
											dyn_payload.temp.b[2],
											dyn_payload.temp.b[3],
											dyn_payload.alert,
											dyn_payload.RFU}; // 14 bytes of data

    ESP_LOGW(TAG, "- DYN CHR - rx voltage = %.02f", dyn_payload.vrect.f);
    ESP_LOGW(TAG, "- DYN CHR - rx voltage = %.02f", dyn_payload.irect.f);
    ESP_LOGW(TAG, "- DYN CHR - rx temperature = %.02f", dyn_payload.temp.f);

    err_code = os_mbuf_append(ctxt->om, &data,
                        sizeof data);
    return err_code;
}

static int gatt_svr_chr_write_pru_control(uint16_t conn_handle, uint16_t attr_handle,
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
	control_payload.RFU = ((uint16_t)data[2]<<8)|data[3];

    //change power output accordingly
    if(control_payload.enable)
    {
        if(control_payload.full_power) {
        enable_full_power_output();
        } else {
        enable_low_power_output();
        }
    } else {
        if(control_payload.full_power) {
        disable_full_power_output();
        } else {
        disable_low_power_output();
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

    uint8_t data[alert_CHAR_SIZE];

    //simulate alert situation
    //alert_payload.alert_field.overtemperature = 1;

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
