# Car Telemetry IoT System
Reads OBD-II parameters from a vehicle via ESP32 and streams data to GCP over MQTT.
![C/C++](https://img.shields.io/badge/C%2FC%2B%2B-Embedded-blue?style=flat-square&logo=cplusplus)
![Platform](https://img.shields.io/badge/Espressif-ESP--IDF_v5.2-orange?style=flat-square&logo=espressif)
![Python](https://img.shields.io/badge/Python-3.8%2B-blue?style=flat-square&logo=python)
![GCP](https://img.shields.io/badge/GCP-Cloud_Infrastructure-green?style=flat-square&logo=google-cloud)
![License](https://img.shields.io/badge/License-MIT-yellow?style=flat-square)
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
  **Core Modules:**
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
## 🛠️ Firmware Installation
### Prerequisites
- Visual Studio Code
- ESP-IDF Extension (V5.2+)

### Clone repo
```bash
git clone <repository_url>
cd vehicle-telemetry/firmware
```

## ⚙️ Configuration
Launch the interactive configuration menu to tweak the SDK properties:
```bash
idf.py menuconfig
```
Navigate through the configuration menus and apply the following parameters:
- **Partition Table** -> `Custom partition table CSV`
- **Bluetooth:**
 - Toggle: `Enable`
 - Set Host to: `NimBLE - BLE only` to enable provisioning

##  Build and Flash
```bash
# Build the project binaries
idf.py build

# Flash the binary and open the serial data monitor
# (Replace /dev/ttyUSB0 with your specific COM port if on Windows/macOS)
idf.py -p /dev/ttyUSB0 flash monitor
```
If there are problems with build try:
```bash
idf.py fullclean
idf.py build
or
idf.py --no-cache build
```

## ☁️ Cloud Infrastructure Deployment
### 🇬🇨🇵 Google Cloud Platform Setup
Provision the following cloud-native resources within your active GCP account layout:
- Cloud Storage bucket
- Bigquery dataset and table
- Compute Engine VM

Install MQTT broker on the virtual machine, deploy the Python ETL script, and run it as a system daemon. Update credentials in the script accordingly.
