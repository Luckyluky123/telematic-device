#ifndef COMMONS_H
#define COMMONS_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief Type of telemetry data contained in a packet.
 */
typedef enum {
    DATA_TYPE_OBD, 
    DATA_TYPE_GPS,
    DATA_TYPE_DIAGNOSTIC
} DataType_t;

//OBD telemetry frame

typedef struct {
    uint16_t speed;
    uint16_t rpm;
    uint8_t fuel_level;
    float fuel_rate;
    uint32_t run_time;
    int16_t coolant_temp;
    uint8_t dtc_count;
    uint8_t mil_status;
    float driving_score;
} OBDFrame_t;


//GPS data structure
typedef struct {
    int32_t lat;
    int32_t lon;
} GPSData_t;



typedef struct {
    DataType_t type;
    uint32_t timestamp;
    union {
        OBDFrame_t obd;
        GPSData_t gps;
    } data;
} TelemetryPacket_t;

/**
 * @brief Global system queue for telemetry data.
 */
extern QueueHandle_t telemetryQueue;

/**
 * @brief Global system event group for state management.
 */
extern EventGroupHandle_t systemEventGroup;


/* =========================
 * System event flags
 * ========================= */

#define WIFI_BIT (1 << 0)
#define CAN_BIT (1 << 1)
#define MQTT_CONNECTED_BIT (1 << 2)
#define FLASH_BIT (1 << 3)
#define DM_HEARTBEAT_BIT (1 << 4)
#define SLEEP_PREPARE_BIT (1 << 5)
#define WIFI_RECONNECT_BIT (1 << 6)
#define DIAG_MODE_BIT (1<<7) 
#define OBD_BUSY_BIT (1<<8) 

#endif 