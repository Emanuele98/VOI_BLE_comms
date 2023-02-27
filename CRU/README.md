# Peripheral (A-CTU)
Bluetooth LE architecture for Bumblebee Auxiliary Communication Transmitting Unit (A-CTU)

## **Table of contents**
- [**Installation**](#installation)

- [**Application files**](#application-files) 
    - [**main.c**](#mainc)
    - [**ble_wpt_aux_ctu.c**](#ble_wpt_aux_ctuc)
    - [**aux_ctu_hw.c**](#aux_ctu_hwc)

- [**idf.py tool**](#idfpy-tool)
    - [**Menuconfig**](#menuconfig)
    - [**Logging**](#logging)
    - [**sdkconfig file**](#sdkconfig-file)

## **Installation**
Making a CTU work with an ESP32 chip requires a few steps. They are provided by the "Get Started" section of the ESP-IDF documentation at https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html. 
It is important to take into account that the CTU has been tested with the IDF v4.2 which can be found at https://github.com/espressif/esp-idf.

------------------------

## **Application files**
Most of what the application does can be seen inside the _main.c_ file.

### **main.c**

- This file contains most of the program with all the different initialization functions. It also contains the basis for the application timers and the overall features of the program.
- It starts by initializing all the different modules needed by the A-CTU application.
- There are 2 timers which update dynamic and alert values, respectively, every `DYNAMIC_PARAM_TIMER_INTERVAL` and `ALERT_PARAM_TIMER_INTERVAL`
     
### **ble_wpt_aux_ctu.c**
   
- This file takes care of BLE events and their handlers, characteristic definitions and payload unpacking/updating;
- An unpacking function handles a particular set of payloads coming from a CTU. It involves the use of a buffer and a GATTS type function named `ble_hs_mbuf_from_flat`;
- As for the updating functions, they are composed of a fixed length buffer and a call to a interface function `ble_hs_mbuf_to_flat`;
- Every characteristic read/write/notify is handled by its respective callback function, declared at the beginning.

```
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &WPT_SERVICE_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[])
        {   
            {
                // Characteristic: ptu static payload.
                .uuid = &WPT_PTU_STAT_UUID.u,
                .access_cb = gatt_svr_chr_write_ptu_static,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                // Characteristic: pru dynamic payload.
                .uuid = &WPT_DYN_UUID.u,
                .access_cb = gatt_svr_chr_read_peer_dynamic,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                // Characteristic: pru control payload.
                .uuid = &WPT_CONTROL_UUID.u,
                .access_cb = gatt_svr_chr_write_peer_control,
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
```

### **aux_ctu_hw.c**
- The hardware module takes care of I2C measurements;
    
------------------------
## **`idf.py` tool**

The `idf.py` tool provided by Espressif allows any ESP32 programmer to build, flash, analyse and configure any ESP chip. The installation procedure also includes this feature and therefore allows a user to take advantage of all the previously described tools.

Command lists can be obtained with the following command:

>`idf.py`

Recommended procedure
    - `idf.py build` to compile and link the project, and produce binary files that are programmable on a ESP chip;
    - `erase_flash` to erase the previous firmware;
    - `flash` to install the new firmware inside the ESP chip;
    - `monitor` to reboot and see the logs;
    - these commands can be unified into idf.py:
    
>`build erase_flash flash monitor`

--> If the ESP board is the only peripheral of the laptop, it should not be necessary to declare which port to use. Otherwise, yes.

It is important however to follow Espressif's guidelines to make sure its framework works properly on any machine.

### **Menuconfig**

To change any embedded configurations, it is required to do so with the ESP-IDF utility that is `menuconfig`. By using the `idf.py` tool combined as such:

>`idf.py` menuconfig

a graphical interface will show up in your current terminal window (or ESP-IDF command prompt if on Windows). It is then possible to change all configurations present in the esp-idf repository and in the Central CTU project.

### **Logging**

- Logging configurations can be found inside the `menuconfig` interface (inside components).
-  It can be changed from `verbose` which is the most resource demanding logging mode, to `error` which is the least demanding. 
- It can also remove logging completely for releases.

### **sdkconfig file**

- The sdkconfig file and its *.old and *.defaults counterparts are all representing configurations defined in `menuconfig` prior to compilation. 
- It is important not to change values directly inside those files, but to simply go to `menuconfig` instead.
