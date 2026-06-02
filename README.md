# Car Telemetry IoT System
Reads OBD-II parameters from a vehicle via ESP32 and streams data to GCP over MQTT.
![C](https://img.shields.io/badge/C-embedded-blue)
![Platform](https://img.shields.io/badge/Espressif-orange)


### System Architecture
<img width="1774" height="1037" alt="architecture" src="https://github.com/user-attachments/assets/52d3cd31-2179-4727-94a4-535ac480d773" />


## 🔧 Device Components

| Element | Model | Function |
|---------|-------|---------|
| Microcontroler | ESP32-S3 | Processing | Dual-core, WiFi, 520KB RAM |
| Transceiver CAN | TJA1051T/3 | Signal conversion | ISO 11898-2 |
| GPS module | ATGM336H | Localization + UTC |
| Voltage Regulator | Pololu S8V9F3 | 12V → 3.3V | 
| Protection | TVS + Fuse 500mA + Capactiors | Device Protection | Automotive standard |


### Wiring diagram
<img width="995" height="697" alt="schemat_ideowy" src="https://github.com/user-attachments/assets/cac07ad2-c1e5-49fa-b3d5-375df546dcf0" />


## 💻 Software Stack

### Firmware (C/C++, ESP-IDF 5.2)
- **FreeRTOS**
- **Protocols:**
  - CAN (ISO 11898-1) via TWAI
  - WiFi via ESP-IDF stack
  - MQTT via Paho library
  - UART 
 - **Core Modules:**
  - `obd_manager.c` — OBD-II PID acquisition and parsing
  - `gps_manager.c` — GPS data acquisition
  - `driving_score.c` — Driving behavior analysis and scoring
  - `data_manager.c` — FSM and data orchestration
  - `flash_storage.c` — Offline buffering
  - `mqtt_manager.c` — Cloud communication and message publishing
### Backend (Python 3.8+)
```
paho-mqtt           # MQTT client
pandas              # Data manipulation
pyarrow             # Parquet serialization
google-cloud-*      # GCP APIs
```
