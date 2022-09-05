# **Central (CTU)**
Bluetooth LE architecture for Bumblebee power transmission unit (CTU)

## **Table of contents**
- [**Installation**](#installation)
- [**Code Components**](#code-components)
    - [**Main (main.c)**](#main-mainc)
    - [**CTU states (CTU_states.c)**](#CTU-states-CTU_statesc)
    - [**BLE central client (ble_central.c)**](#ble-central-client-ble_centralc)
- [**Non-volatile storage (NVS)**](#non-volatile-storage-nvs)
- [**idf.py tool**](#idfpy-tool)
- [**Menuconfig**](#menuconfig)
    - [**Modes of operation**](#modes-of-operation)
    - [**Logging**](#logging)
    - [**sdkconfig file**](#sdkconfig-file)


## **Installation**

Making a CTU work with an ESP32 chip requires a few steps. They are provided by the "Get Started" section of the ESP-IDF documentation. It is important to take into account that the CTU has been tested with the IDF v4.1.1 which can be found at https://github.com/espressif/esp-idf. To directly specify an IDF version with Git, use the folloing command:

>git clone -b v4.1.1 --recursive https://github.com/espressif/esp-idf.git

Another important step to consider is reviewing the configurations by using `menuconfig`, a tool that allows configurations to be easily changed in a simple CLI.

## **Code Components**

Throughout this document, mentions of main and host are not uncommon. The main refers to the RTOS context that initiates the program following the bootloader application. It also takes care of the state machine that runs on the ESP32. The main resides on the APP_CPU (Core 1). The host refers to another context, one that handles the NimBLE stack and its various states. The host resides on the PRO_CPU (Core 0).

### **Main (main.c)**

This file contains the very important function `app_main()` which is the starting point of any ESP32 application. There are also many configuration functions and some low-level callbacks related to the BLE host. The FreeRTOS task assigned to the application's start function is named *main* and runs on the Pro CPU (CPU0).

- `app_main()` starts by trying to initialize the NVS partition and validating that this initialization worked properly.
- Then, the *main* task procedes to get all NVS data present in the NVS partition (CTU Static Data).
- Next, the `main` task initializes the ESP Bluetooth controller (link layer) and VHCI transport layer between NimBLE Host and ESP Bluetooth controller.
- Afterwards, some of the BLE `host`'s callbacks are set to various callback functions, both of them need to be customized to a specific use case.
    - This implies that whenever the `host` has a reset procedure scheduled, it will call `host_ctrl_on_reset` and proceed accordingly. 

The next part of `app_main()` involves creating and initializing the peer structure. A peer is equivalent to a CRU since all CRUs are servers. Use cases that involve multiple CRUs connected simultaneously require most of what a peer structure is able to offer such as:
- A linked list pointer to the next peer in line
- BLE related variables
    - Connection handle
    - Discovery process tracker
    - Specific payloads
- A FreeRTOS task handle
- A FreeRTOS semaphore handle to manage asynchronous BLE read requests

Then, all hardware and software modules required by the application are initialized.

The last step is then to run the states module to really manage the CTU dynamically.

### **CTU states (CTU_states.c)**

The CTU state machine is handled almost entirely inside this module. It progresses and retracts to different states depending on its previous state and the AFA standard. Any transition that is undertaken is handled in the function `CTU_state_change(...)`. The different possible states are the following:

- **Configuration State**

    - The Configuration State only starts the periodic local check timer (to determine if any Local Faults are present).
    //to be added - connection to at least 1 aux ctu

- **Low power state**
  
    - The power save state is the main state from which the CTU will run. In this state, `main` is in a busy wait kind of loop.
    - In this state, both the `main` and `host` are involved. The `main` handles very little in the current application, but depending on the application, it can house some logic.
    - The `host`, on the other hand handles both the connection procedure and the registration sequence.
    - By receiving a valid advertisement with WPT Service UUID and also within a `rssi` that is smaller than `MINIMUM_ADV_RSSI`, the CTU procedes with multiple read/write procedures to allow an exchange of static parameters. This process will be described in more details in the next CTU module [*BLE central client (ble_central.c)*](#ble-central-client-ble_centralc).
   - The CTU transitions to the power transfer state if there are less than `MYNEWT_VAL(BLE_MAX_CONNECTIONS)` peers connected, in which case there will be attempts to scan periodically for other CRUs. Once there are `BLE_MAX_CONNECTIONS` CRUs connected, no further scan attempts will be undertaken.
   - Also, here the `localization_process` happens. Each pad is switched on sequentially in a low power mode (to avoid charging of faulty CRUs), so the value of the `Vrect` is compared with a given treshold. If it is above the treshold, then the right pad has been found and the Power Transfer State can finally begin. 

- **Power transfer state**
  
    - This state also involves both `main` and `host`, but this time equally. The `main` task handles all power transfer substates and all decisions regarding the values received from neighboring CRUs. The `host` task instead handles all communication procedures between any CRU and the CTU.
    - Entering the "parent" function `CTU_power_transfer_state`, `main` starts by setting the total number of latching faults to 0. This allows on to reset to an error-less state.


- **Local fault state**
    
    - Happens when sensor values received from A-CTUs are over the defined limit (overvoltage | overcurrent | overtemperature);
    - Any time this state is reached through the `CTU_periodic_local_check` timeout function, the BLE stack resets and the configuration state is reached;
    - The main application's duty is then to remain in a continuous loop until the `host_ctrl_on_reset` callback is run. Only then will the state flag change to the configuration state. Also, any and all peers that were previously present on the stack are now gone. They will have to advertise yet again, and connect to the CTU once again.

- **Latching fault state**

    - Happens when sensor values received from CRU are over the defined limit (overvoltage | overcurrent | overtemperature);
    - If `main` enters this state, the CTU has 3 "strikes" to correct any possible cause for this fault.
    - At first, the application stop the power output under the relative peer.
    - The CRUs will be disconnected and a delay of 5 seconds will be started. This allows a CRU to perhaps fix the latching fault on his own.
    - At the count of 3 consecutive latching faults, local manteinance is requested (email to @bumblebee)


### **BLE central client (ble_central.c)**

- The BLE central module is almost solely handled by the `host` context. The only part of this module that runs on another task is the handle function for individual CRUs.
- Any type of function/procedure/callback that is needed for BLE communication is present in this CTU module.

- **Discovery attempt**

    All discovery attempts will follow the same pattern. The steps leading to a succesful discovery attempt are the following:
    - _ble_central_scan_start_

        - The discovery parameters used to configure the scanning procedure on the ESP32 are stored in `disc_params`.
        - Once a scan procedure starts through the function `ble_gap_disc`, any device within BLE range will be eligible for device discovery.
        - ble_central_gap_event() is the callback function used for GAP and GATTC events all throughout the application's runtime.

    - _ble_central_gap_event_
        
        - This callback executes if the previous discovery procedure has succesfuly completed. The resulting event associated with the callback function will then be `_BLE_GAP_EVENT_DISC_`.

    - _ble_central_connect_if_interesting_

        - it will then run into `ble_central_connect_if_interesting` to determine if it is pertinent to connect to the newly found device. 
        - For each BLE device in proximity, the `host` task will analyze, through the `ble_central_should_connect` function, their service UUID to determine if it is a CRU. It will also determine if said CRU is in a relatively close range (rssi > -80dBm)
        - On an unsuccesful discovery procedure, it returns and cancels any active discovery procedure.

- **Connection/Registration attempt**

    - Connection attempts happen whenever a CRU is found by the discovery procedure.
    - The GAP procedure, as per NimBLE specifications, also attributes an event handler for both GAP and GATT events. The callback function chosen for this purpose is `ble_central_gap_event` which has been used previously by `ble_gap_disc`. Henceforth, this function will only run on events, not as a callback.
    - On a successful connection attempt, the new connection with all its parameters will fill a peer structure and be added to a memory pool.
    - To properly identify what the CRU has to offer, the `host` task will fill out all Airfuel defined characteristics inside the peer structure's various memory pools with the `peer_disc_all` function.

   Once this runs completely, the `host` context procedes to run different functions in order:
    - ble_central_on_disc_complete
      - Only when all services/characteristics/descriptors have been discovered;
    - ble_central_on_static_chr_read
      - When the CRU static characteristic has been read;
      - It unpacks the CRU static characteristic and compiles its content within its peer structure;
      - Then, it writes without response to the CTU static characteristic;
    - ble_central_on_write_cccd
      - Handles both the writing process for the CRU alert characteristic and the cccd subscription (to enable notifications on CTU);
    - ble_central_on_subscribe
      - read the first dynamic chr
    - ble_central_on_control_enable
      - only if the peer is an A-CTU, it checks the presence of the Control chr
    - ble_central_wpt_start
      - Creates a task and a semaphore and assigns them to the new peer;   

    After all those steps, the peer is connected and registered for the WPT service.
        - Keep reading the Dynamic chr (at least every 20ms) and check if any Alert is detected (a field of dyn chr is filled with alert status).
        
- **Localization process**

    If the peer is a CRY, the localization process starts (ble_central_on_localization function);
    - Basically:
        - Switch on a pad and wait a reasonable amount of time to see the change in the rx side;
        - Read the Dynamic characteristic to know `Vrect`. Above the treshold?
        - If yes, the position of CRU is known and we move on;
        - If not, we try the same process with the next pad;
    - Restrictions:
        - Only one CRU can undergo the localization process per time to avoid misunderstandings; 
        - If position not found after a predefined amount of time, it disconnects to allow other CRU to try the process.
   

- **Signal reception**

    On any signal reception, the event handler that was described before will handle any events related to those signals. For example, a notification will trigger a notification event in `ble_central_gap_event` and parse all bytes from the characteristic that triggered this specific notification (for the CTU, notifications are in fact alerts from the CRU).

- **Peer task handling**

    Here the description covers only CRU; however, the A-CTU peer task handling follows almost the same procedure with similar functions (e.g. ble_central_AUX_CTU_task_handle).

    - Any CRUs peer structure will be assigned a task handle and a binary semaphore handle;
    - All CRUs will run simultaneously on the `ble_central_CRU_task_handle` function and periodically read the dynamic characteristic when available;
    - The availabiliy of this characteristic depends on the semaphores state bit;
    - If the semaphore is taken and has yet to be given back, the peer task will simply wait for a short period of time and iterate once more. Only when the semaphore is given will this task trigger a read procedure on the dynamic characteristic;
    - The semaphore is taken (in `ble_central_CRU_task_handle`);
    - The semaphore is given back (in `ble_central_on_CRU_dyn_read`).

## **Non-volatile storage (NVS)**

Currently, the static values needed for the CTU are all stored in a separate partition from the application partition. This partition is named NVS and has to be programmed (flashed) before a CTU application can run properly.

## **`idf.py` tool**

The `idf.py` tool provided by Espressif allows any ESP32 programmer to build, flash, analyse and configure any ESP chip. The installation procedure also includes this feature and therefore allows a user to take advantage of all the previously described tools.

Command lists can be obtained with the following command:

>`idf.py`

By combining it with an argument such as `build`, the `idf.py` tool will be able to compile and link the project, and produce binary files that are programmable on a ESP chip.

It is important however to follow Espressif's guidelines to make sure its framework works properly on any machine.

## **Menuconfig**

To change any embedded configurations, it is required to do so with the ESP-IDF utility that is `menuconfig`. By using the `idf.py` tool combined as such:

>`idf.py` menuconfig

a graphical interface will show up in your current terminal window (or ESP-IDF command prompt if on Windows). It is then possible to change all configurations present in the esp-idf repository and in the Central CTU project.

### **Logging**

Logging configurations can be found inside the `menuconfig` interface. It can be changed from `verbose` which is the most resource demanding logging mode, to `error` which is the least demanding. It can also remove logging completely for releases.

### **sdkconfig file**

The sdkconfig file and its *.old and *.defaults counterparts are all representing configurations defined in `menuconfig` prior to compilation. It is important not to change values directly inside those files, but to simply go to `menuconfig` instead.