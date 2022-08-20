# Bumblebee-Airfuel-Comms

All the units are implemented in ESP-32 boards.
The following protocol is based on, but not entirely follows, the AirFuel Alliance Resonant Wireless Power Transfer (WPT) System
4 Baseline System Specification (BSS).

------------------------------------------------

### Important notes

- The NimBLE stack has been chosen as more lightweigth protocol, comprising smaller heap and flash requirements;
- Communication Transmitting Unit (CTU) acts as a client, requesting access to the data (here most of the system logic happens);
- Communication Receiving Units (CRUs) and Auxiliary CTU (A-CTUs) act as servers, holding the data;
- This implements a bluetooth protocol between one host (CTU) and many peripherals (CRUs and A-CTUs). 

------------------------------------------------- 

### Service and Characteristics

- 1 service:
  - Wireless Power Transfer (WPT)
- 5 characteristics
  - CRU Static (READ)
  - CTU Static (READ | WRITE)
  - CRU Dynamic (READ)
  - CRU Control (READ | WRITE)
  - CRU Alert (READ | WRITE | NOTIFY)
-------------------------------------------------

### Basic state procedure
1. The CTU first needs to be configured, meaning being successfully connected to at least one A-CTU;
2. The CRU repeatedly sends advertisements until it receives a CTU Connection Request;
3. The CTU moves from 'Power Save State' to 'Low Power State' to enstablish a connection with first CRU;
4. When the CTU connects to either a CRU or A-CTU, the following read/write procedures take place:
      - The CTU reads the value of Static chr;
      - The CTU writes the value of Static chr;
      - The CTU reads the value of Dynamic chr, which contains I2C sensor measurements (V, A, °C).
5. If the connection is enstablished with a CRU, the localization process starts (more details in the CTU README.md):
      - The CTU switches on (in a low power mode) sequentially each transmitting pad (the command is sent over BLE to each A-CTU);
      - Reasonable time-gap for the voltage to be detected on the rx side;
      - The CTU reads the value of CRU Dynamic to know CRU measurements;
      - If Vrx above a treshold, right pad found!;
      - If timer expires, it disconnects to allow other CRU to undergo the localization process.
6. Charging is then initiated and the CTU moves to 'Power Transfer State';
7. The CTU then keeps reading the CRU Dynamic parameter at least every 250ms;
8. Keeps running until the CRU detects a system error or completes charging, so it sends a CRU Alert notifications to the CTU;
9. The CTU takes care of the Alert, going to 'Latching fault state' (system error) or back to 'Power save state' (charge complete).
10. The CTU eventually also go to 'Local fault state' if an alert is transmitted from any A-CTU.


-------------------------------------------------
### Main differences with Airfuel specifications

- PTU is called CTU (Communication Transmitting Unit)
- PRU is called CRU (Communication Receiving Unit)
- A-CTU added to the system on each power transmitting pad (Auxiliary CTU)
- A-CTUs send I2C measurements to the CTU over BLE (as other wired protocols suffer system interferences)
- CRU and A-CTU share same structure in the code ('struct peer')
- Beacons removed as these devices will be always on;
- Registration timeout removed;
- Dynamic control of output power has been removed as no such capability is implemented at the SoA;
- Localization protocol has been added, allowing to identify where the PRU is placed and therefore which pad needs to be charged;
- Fields of Dynamic chr changed to allow float measurements to be sent over BLE;
- Control chr kept only for A-CTU; removed from CRU as considered useless.

 `find more details about how CTU, A-CTU and CRU and individually work in the READMEs contained in their relative folders.`
 
 (still to be updated)
