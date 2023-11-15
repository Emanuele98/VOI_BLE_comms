# Bumblebee-Airfuel-Comms

All the units are implemented in ESP-32 boards.
The following protocol is based on, but not entirely follows, the AirFuel Alliance Resonant Wireless Power Transfer (WPT) System
4 Baseline System Specification (BSS).

------------------------------------------------

### Important notes

- The NimBLE stack has been chosen as more lightweigth protocol, comprising smaller heap and flash requirements;
- Communication Transmitting Unit (CTU) is the brain of the CCU. It acts as a client, requesting access to the data.
- Communication Receiving Units (CRUs) and Auxiliary CTU (A-CTUs) act as servers, holding the data;
- This implements a bluetooth protocol between one host (CTU) and many peripherals (CRUs and A-CTUs). 

### Monitoring

- The data are sent via MQTT to a NodeRED dashboard;
- Data are then saved in AWS cloud storage;
- Remote switching done via a smart plug connected to the local WiFi.

------------------------------------------------- 

### BLE Service and Characteristics

- 1 service:
  - Wireless Power Transfer (WPT)
- 4 characteristics
  - CRU and A-CTU Static (READ)
  - CRU Dynamic (READ)
  - A-CTU Control (READ | WRITE)
  - CRU and A-CTU Alert (READ | WRITE | NOTIFY)
-------------------------------------------------

### Basic state procedure
1. The CTU first needs to be configured, meaning being successfully connected to at least one A-CTU;
2. The CRU repeatedly sends advertisements until it receives a CTU Connection Request;
3. The CTU moves from 'Configuration state' to 'Low Power State' to enstablish a connection with first CRU;
4. When the CTU connects to either a CRU or A-CTU, the following read/write procedures take place:
      - The CTU reads the value of Static chr;
      - The CTU writes the value of Static chr (sets the alert threshold);
      - The CTU reads the value of Dynamic chr, which contains I2C sensor measurements (V, A, °C):
      - The CTU create a task (FreeRTOS) which will take care of the connected peripheral.
5. If the connection is enstablished with a CRU, the localization process starts (more details in the CTU README.md):
      - The CTU switches on (in a low power mode) sequentially each transmitting pad (the command is sent over BLE to each A-CTU);
      - Wait a reasonable time-gap for getting a reliable rx voltage measurement;
      - The CTU reads the value of CRU Dynamic to know CRU measurements;
      - If Vrx above the fixed treshold, the position is found and the charging can start;
      - After a failed double check on each pad, this CRU must wait a time-gap before retrying. This will go as long as the scooter is connected.
6. Charging is then initiated and the CTU moves to 'Power Transfer State':
      - The CRU must pass a fixed Voltage threshold in the first 10s to keep the charging active;
      - The CRU must receive a minimum voltage to trigger the charger and make the leds show a flowing green light;
      - If it does not receive this minimum amount, the 'misaligned state' is shown (bliniking orange) in order to make the user understand he must move the scooter. 
7. The CTU then keeps periodically read the CRU Dynamic parameter;
8. Keeps running until either the scooter is fully charged or it sends an alert (overvoltage, overcurrent, overtemperature);
10. The CTU eventually also go to 'Local fault' state if an alert is transmitted from any A-CTU (overvoltage, overcurrent, overtemperature, FOD).
      - every alert sets its respective timegap before which the unit will get connected again.


-------------------------------------------------
### Main differences with Airfuel specifications

- PTU is called CTU (Communication Transmitting Unit);
- PRU is called CRU (Communication Receiving Unit);
- A-CTU added to the system on each power transmitting pad (Auxiliary CTU);
- A-CTUs send I2C measurements to the CTU over BLE (as other wired protocols suffer system interferences);
- CRU and A-CTU share same structure in the code ('struct peer');
- Beacons removed as for now these devices are always ON;
- Registration timeout removed;
- Dynamic control of output power has been removed as no such capability is implemented at the SoA;
- Localization protocol has been added, allowing to identify where the PRU is placed;
- Fields of Dynamic chr changed to allow float measurements to be sent over BLE;
- Control chr kept only for A-CTU; removed from CRU as considered useless;
- Fully charged state is kept until the scooter moves away (done by checking the RSSI).

 `find more details about how CTU, A-CTU and CRU and individually work in the READMEs contained in their relative folders.`
