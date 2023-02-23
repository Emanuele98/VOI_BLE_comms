# **Central (CTU)**
Bluetooth LE architecture for Bumblebee power transmission unit (CTU)

## **Table of contents**
- [**Installation**](#installation)
- [**Code Components**](#code-components)
    - [**Main (main.c)**](#main-mainc)
    - [**CTU states (CTU_states.c)**](#CTU-states-CTU_statesc)
    - [**BLE central client (ble_central.c)**](#ble-central-client-ble_centralc)
    - [**WiFi connection (wifi.c)**](#wifi-connection-wific)
    - [**SD Card (sd_card.c)**](#sd-card-sd_cardc)
    - [**Peer (peer.c)**](#peer-peerc)
    - [**DHT22 sensor (DHT22.c)**](#DHT22-sensor-DHT22.c)
- [**idf.py tool**](#idfpy-tool)
    - [**Menuconfig**](#menuconfig)
    - [**Logging**](#logging)
    - [**sdkconfig file**](#sdkconfig-file)

--------------------------------------------------

## **Installation**

Making a CTU work with an ESP32 chip requires a few steps. They are provided by the "Get Started" section of the ESP-IDF documentation at https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html. 
It is important to take into account that the CTU has been tested with the IDF v4.2 which can be found at https://github.com/espressif/esp-idf.

--------------------------------------------------

## **Code Components**

Throughout this document, mentions of main and host are not uncommon. The main refers to the RTOS context that initiates the program following the bootloader application. It also takes care of the state machine that runs on the ESP32. The main resides on the APP_CPU (Core 1). The host refers to another context, one that handles the NimBLE stack and its various states. The host resides on the PRO_CPU (Core 0).

1 Service:
    - wpt_svc_uuid (defined by Bluetooth SIG).
    
4 Characteristics:
    - `wpt_char_control_uuid`; 
    - `wpt_char_alert_uuid`;  
    - `wpt_char_stat_uuid`; 
    - `wpt_char_dyn_uuid`.

### **Main (main.c)**

This file contains the very important function `app_main()` which is the starting point of any ESP32 application. There are also many configuration functions and some low-level callbacks related to the BLE host. The FreeRTOS task assigned to the application's start function is named *main* and runs on the Pro CPU (CPU0).

- `app_main()` starts by trying to initialize the NVS partition and validating that this initialization worked properly;
- Then, the *main* task procedes to get all NVS data present in the NVS partition (CTU Static Data);
- Next, the `main` task initializes the ESP Bluetooth controller (link layer) and VHCI transport layer between NimBLE Host and ESP Bluetooth controller;
- Afterwards, some of the BLE `host`'s callbacks are set to various callback functions, both of them need to be customized to a specific use case;
    - This implies that whenever the `host` has a reset procedure scheduled, it will call `host_ctrl_on_reset` and proceed accordingly;
    - It is crucial to wait for `host_ctrl_on_sync` to be called before initiating any BLE procedure.

The next part of `app_main()` involves creating and initializing the peer structure. A peer is equivalent to a peripeheral, either CRU or A-CTU. Use cases that involve multiple peripherals connected simultaneously require most of what a peer structure is able to offer such as:
- A linked list pointer to the next peer in line;
- BLE related variables:
    - Connection handle;
    - Discovery process tracker;
    - Specific payloads.
- A FreeRTOS task handle.

Then, all hardware and software modules required by the application are initialized:
    - SD CARD;
    - WIFI (+ connection to the MQTT broker);
    - 2 software timers:
        - scanning process (every second - can be changed in `PERIODIC_SCAN_TIMER_PERIOD` );
        - ambient temperature measurements (every 10 seconds - can be changed in `PERIODIC_AMBIENT_TEMP_TIMER` );
    - NVS data are then retrieved in some local variables (here, the disconnection times is saved in order to wait for a reconnection, e.g. after an alert, even among       reboots).

The last step is then to run the states module to really manage the CTU dynamically.

### **CTU states (CTU_states.c)**

The CTU state machine is handled almost entirely inside this module. It progresses and retracts to different states depending on its previous state and the AFA standard. Any transition that is undertaken is handled in the function `CTU_state_change(...)`. The different possible states are the following:

- **Configuration State**

    - The Configuration State tries to connect to all the A-CTU. The unit moves to the Low Power state as soon as all 4 of them are successfully connected. If this is       not the case, it will move to the Low Power state anyway after 20 seconds (`CONF_STATE_TIMEOUT`) of absence of any A-CTU advertisements;
    - The BLE connection process is described in details in the next CTU module [*BLE central client (ble_central.c)*](#ble-central-client-ble_centralc).

- **Low power state**

    - Here, the CTU is finally searching for CRUs;
    - When a CRU gets connected, the `localization_process` happens:
       - Each pad is switched on sequentially in a low power mode (to avoid charging of faulty CRUs);
       - the value of the `Vrect` is compared with a given treshold (`VOLTAGE_LOW_THRESH`);
       - if it is above the treshold, then the right pad has been found and the Power Transfer State can finally begin; 
       - The variables which determine the speed of this process are `LOC_CTU_TIMER_PERIOD` and `LOC_CRU_TIMER_PERIOD`.

- **Power transfer state**
  
    - This state means at least one scooter is being charged or fully charged;
    - The master keeps reading the sensor values of the A-CTU and the CRU, send them to the MQTT broker and writes them to the SD card:
        - The speed of this process is crucial as it takes a lot of CPU resources;
        - `CTU_TIMER_PERIOD` and `CRU_TIMER_PERIOD`;
    - `Localization process` for other CRUs happens in the meanwhile;
    - When the CTU reaches the `BLE_MAX_CONNECTIONS` peers connected (currently set to 8), no further scan attempts will be undertaken.


- **Local fault state**
    
    - Happens when sensor values received from A-CTUs are over the defined limit (overvoltage | overcurrent | overtemperature) or a metal object was placed in between       the gap (FOD);
    - The alert is sent from the A-CTU;
    - The A-CTU is switched OFF, and then disconnected for some time depending on the type of alert:
        - `TX_RECONNECTION_OVERVOLTAGE`;
        - `TX_RECONNECTION_OVERTEMPERATURE`;
        - `TX_RECONNECTION_OVERCURRENT`;
        - `TX_RECONNECTION_FOD`;
    - Finally, the master then goes back to the:
        - Configuration State if no other A-CTUs are still connected;
        - Power Transfer State if at least one scooter is currently being charged or already fully charged;
        - Low Power State otherwise.

- **Latching fault state**

    - Happens when sensor values received from CRU are over the defined limit (overvoltage | overcurrent | overtemperature);
    - At first, the application stop the power output under the relative scooter;
    - The scooter won't be able to enstablish the BLE connection again for some time depending on the type of alert:
        - `RX_RECONNECTION_OVERCURRENT`;
        - `RX_RECONNECTION_OVERTEMPERATURE`;
        - `RX_RECONNECTION_OVERVOLTAGE`; 
    - Finally, the master then goes back to the:
        - Low Power state if no other scooters are currently being charged or have already reached the fully charged state;
        - Power Transfer State otherwise.
      

### **BLE central client (ble_central.c)**

- The BLE central module is almost solely handled by the `host` context. The only part of this module that runs on another task is the handle function for individual peripherals;
- Any type of function/procedure/callback that is needed for BLE communication is present in this CTU module.

- **Discovery attempt**

    All discovery attempts will follow the same pattern. The steps leading to a succesful discovery attempt are the following:
    - _CTU_periodic_scan_timeout_

        - The discovery parameters used to configure the scanning procedure on the ESP32 are stored in `disc_params`.
        - Once the relative software timer is started in the Configuration State, any device within BLE range will be eligible for device discovery.
        - ble_central_gap_event() is the callback function used for GAP and GATTC events all throughout the application's runtime.

    - _ble_central_gap_event_
        
        - This callback executes if the previous discovery procedure has succesfuly completed. The resulting event associated with the callback function will then be             `_BLE_GAP_EVENT_DISC_`.

    - _ble_central_connect_if_interesting_

        - it will then run into `ble_central_connect_if_interesting` to determine if it is pertinent to connect to the newly found device; 
        - Each reported BLE device in proximity is analyzed through the `ble_central_should_connect` function:
            - RSSI value must be below `MINIMUM_ADV_RSSI` (-90dB);
            - The MAC address is checked to connect only to registered devices.
       
- **Connection/Registration attempt**

    - Connection attempts happen whenever a CRU is found by the discovery procedure;
    - The scanning is stopped before the connection attempt;
    - Conn_params defines the connection parameters;
    - The GAP procedure, as per NimBLE specifications, also attributes an event handler for both GAP and GATT events. The callback function chosen for this purpose is `ble_central_gap_event` which has been used previously by `ble_gap_disc`. Henceforth, this function will only run on events, not as a callback;
    - On a successful connection attempt, the new connection with all its parameters will fill a peer structure and be added to a memory pool;
    - To properly identify what the CRU has to offer, the `host` task will fill out all Airfuel defined characteristics inside the peer structure's various memory pools with the `peer_disc_all` function.

   Once this runs completely, the `host` context procedes to run different functions in order:
    - ble_central_on_disc_complete
      - Only when all services/characteristics/descriptors have been discovered;
    - ble_central_on_static_chr_read
      - When the CRU static characteristic has been read;
      - It unpacks the CRU static characteristic and compiles its content within its peer structure;
    - ble_central_on_write_cccd
      - Handles cccd subscription (to enable notifications on CTU);
    - ble_central_on_subscribe
      - read the first dynamic chr
    - ble_central_on_control_enable
      - only if the peer is an A-CTU, it checks the presence of the Control chr
    - ble_central_wpt_start
      - Creates a FreeRTOS task and assigns it to the new peer;   

    After all those steps, the peer is connected and registered for the WPT service:
        --> the master keeps reading its Dynamic chr and check if any Alert are detected (a field of dyn chr is filled with alert status);
        --> if an alert is detected, the master read the Alert chr to undestand its nature and react accordingly.
        
- **Localization process**

    If a connected CRU does not have a defined position (_peer->position = 0_) the localization process happens:
    - Basically:
        - All these scooters are set as to-be-checked (_set_scooters_tobechecked_);
        - Switch on a pad and wait a reasonable amount of time to see the change in the rx side (`MIN_SWITCH_TIME`);
        - Check these scooters reading their Dynamic characteristic to know `Vrect` and reset their to-be-checked variable. 
        - Is the rx voltage above the treshold? (`VOLTAGE_LOW_THRESH`)
            - If yes, the position of CRU is known and we move on;
            - If not and all scooters have been checked, we try the same process with the next pad (_pass_the_baton_);
        - These processes are located in _ble_central_on_AUX_CTU_dyn_read_ and _ble_central_on_localization_process_.
        - To double check the localization was correct, the scooter must receive `VOLTAGE_FULL_THRESH_ON` in the first 10 seconds of the charging.
    - Restrictions:
        - If position not found after `MAX_LOC_CHECKS` attempts, the scooter must wait `MIN_TIME_AFTER_LOC` to prioritize others to be found.

- **Fully charged**
     - Whenever a scooter reaches the fully charged state, the charging must be stopped. However, the scooter must stay connected as the charging pad is not available        for others.
     - `fully_charged[]` is used to keep track of the fully charged scooters, while their relative charge_complete field contained in the peer structure tells wheter           the scooter needs to be disconnected in the killing process. Indeed, ble_central_kill_CRU is called after receiving the notifications as its treated as an             alert.
     - Since the CRU is currently lacking an accelerometer sensor, in order to know if the scooter is still placed on the pad the RSSI value is repeatedly checked:
        - when it is found below `MINIMUM_FULLY_CHARGED_RSSI` for 5 consecutive checks, it means the scooter left and the pad is available for others.

- **Undesidered alerts managament**
If, for some reason, the CTU should not care about an alert from a peripheral, it needs not only to avoid taking actions but also to read its Static Chr to reset the alert variable inside the peripheral unit. (this has been done for undesidered FOD and OV on pad 3).


### **WiFi (wifi.c)**

The WiFi unit is pretty dumb. After connecting to `WIFI_SSID` with `WIFI_PASS` (which can be configured in Menuconfig), it connects to the broker. Whenever a disconnection happen, a forced reconnections is done.
Please bear in mind that the ESP32 modules only has 2.4 GHz capabilities.

### **SD Card (sd_card.c)**
This module exploits the https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sdspi_host.html. If the installation of the peripheral fails, it won't try.

### **Peer (peer.c)**

Internal structure for CTU peer module. The peer is the main container for any peripherals context. All things relating to peripherals are incorporated into this module. It also implements all the functions necessary to create, fill and destroy peers dynamically.

### **DHT22 sensor (DHT22.c)**

Temperature and Humidity sensor. Provides value and checks every `PERIODIC_AMBIENT_TEMP_TIMER` whether it's too cold to enable any charging through the variable `TOO_COLD`. Also, NVS values about the reconnection times are refreshed here as very CPU expesive.

--------------------------------------------------

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
