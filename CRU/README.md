# Peripheral (CRU)
Bluetooth LE architecture for Bumblebee Communication Transmission Unit (CRU)

## **Table of contents**
- [**Peripheral**](#Peripheral-(PRU))
  - [**Table of contents**](#table-of-contents)
  - [**Installation**](#installation)
  - [**Application files**](#application-files) 
- [**idf.py tool**](#idfpy-tool)
- [**Menuconfig**](#menuconfig)
    - [**Modes of operation**](#modes-of-operation)
    - [**Logging**](#logging)
    - [**sdkconfig file**](#sdkconfig-file)
    
## **Installation**

Making a CRU work with an ESP32 chip requires a few steps. They are provided by the "Get Started" section of the ESP-IDF documentation. It is important to take into account that the PTU has been tested with the IDF v4.1.1 which can be found at https://github.com/espressif/esp-idf. To directly specify an IDF version with Git, use the folloing command:

>git clone -b v4.1.1 --recursive https://github.com/espressif/esp-idf.git

Another important step to consider is reviewing the configurations by using `menuconfig`, a tool that allows configurations to be easily changed in a simple CLI.


## **Application files**
------------------------
Most of what the application does can be seen inside the _main.c_ file.

  * main.c

       - This file contains most of the program with all the different initialization functions. It also contains the basis for the application timers and the overall features of the program.
        - It starts by initializing all the different modules needed by the PRU application.
        - There are 2 timers which update dynamic and alert values, respectively, every 10ms and 250 ms
------------------------
    
In the _ble_ folder, there are 2 files that handle Bluetooth Low Energy functions. Specifically, it takes care of all the characteristics callbacks related to the Wireless Power Transfer service and it also takes care of the characteristics payloads.

   * ble_wpt_pru.h
   
      - This header file defines some variables for the preprocessor as well as the data structures needed for the application. It contains all the different structures for BLE characteristics, in accordance with the AFA standard.
      - The way UUIDs work in the software is by masking specific characteristic UUIDs with the base 128 bits vendor UUID. Therefore, when the application refers to a BLE characteristic, the resulting UUID is the combination of both the mask and the base.
      -The most important components of this header file are all the structures that define the various characteristic payloads
     
   * ble_wpt_pru.c
   
      - This file takes care of BLE events and their handlers, characteristic definitions and payload unpacking/updating.
      - An unpacking function handles a particular set of payloads coming from a PTU. It involves the use of a buffer and a GATTS type function named `ble_hs_mbuf_from_flat`.
      - As for the updating functions, they are composed of a fixed length buffer and a call to a interface function `ble_hs_mbuf_to_flat`.
      - Every characteristic read/write/notify is handled by its respective callback function, declared at the beginning:

```
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &wpt_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        {   {
                // Characteristic: pru static payload.
                .uuid = &WPT_PEER_STAT_UUID.u,
                .access_cb = gatt_svr_chr_read_peer_static,
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
                .uuid = &WPT_PEER_DYN_UUID.u,
                .access_cb = gatt_svr_chr_read_peer_dynamic,
                .flags = BLE_GATT_CHR_F_READ,
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

### **Hardware utility (ptu_hw.c)**

- The hardware module takes care of I2C measurements.

## **Non-volatile storage (NVS)**

Currently, the static values needed for the PTU are all stored in a separate partition from the application partition. This partition is named NVS and has to be programmed (flashed) before a PTU application can run properly.

## **`idf.py` tool**

The `idf.py` tool provided by Espressif allows any ESP32 programmer to build, flash, analyse and configure any ESP chip. The installation procedure also includes this feature and therefore allows a user to take advantage of all the previously described tools.

Command lists can be obtained with the following command:

>`idf.py`

By combining it with an argument such as `build`, the `idf.py` tool will be able to compile and link the project, and produce binary files that are programmable on a ESP chip.

It is important however to follow Espressif's guidelines to make sure its framework works properly on any machine.

## **Menuconfig**

To change any embedded configurations, it is required to do so with the ESP-IDF utility that is `menuconfig`. By using the `idf.py` tool combined as such:

>`idf.py` menuconfig

a graphical interface will show up in your current terminal window (or ESP-IDF command prompt if on Windows). It is then possible to change all configurations present in the esp-idf repository and in the Central PTU project.

### **Logging**

Logging configurations can be found inside the `menuconfig` interface. It can be changed from `verbose` which is the most resource demanding logging mode, to `error` which is the least demanding. It can also remove logging completely for releases.

### **sdkconfig file**

The sdkconfig file and its *.old and *.defaults counterparts are all representing configurations defined in `menuconfig` prior to compilation. It is important not to change values directly inside those files, but to simply go to `menuconfig` instead.
