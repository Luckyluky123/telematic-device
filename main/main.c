#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "commons.h"
#include "obd_manager.h"
#include "wifi_manager.h"
#include "gps_manager.h"
#include "data_manager.h"
#include "mqtt_manager.h"
#include "system_monitor.h"
#include "flash_storage.h"
#include "time_manager.h"
#include "driving_score.h"
static const char *TAG = "MAIN";

/* =====================================================
 * Global RTOS objects
 * ===================================================== */
EventGroupHandle_t systemEventGroup;
QueueHandle_t telemetryQueue = NULL;



/*
 * System architecture overview:
 *
 * Priority:
 * 1. CAN (vehicle activity)
 * 2. WiFi (connectivity)
 *
 * Behavior:
 *
 * - CAN OFF:
 *   - If telemetry exists in queue → store to flash
 *   - Enter low power mode to reduce energy consumption
 *   - Wait for CAN activity
 *
 * - WiFi available:
 *   - Flush buffered data to cloud (MQTT)
 *   - Then optionally enter power saving mode
 *
 * - CAN ON:
 *   - Collect data from CAN + GPS
 *   - Send via MQTT if possible
 *   - If WiFi is unavailable → buffer data in flash
 *   - Prevent queue overflow using local storage
 */

/* =====================================================
 * Application entry point
 * ===================================================== */


void app_main(void) {

    /* Create RTOS synchronization objects */
    systemEventGroup = xEventGroupCreate();
    if (systemEventGroup == NULL) {
    ESP_LOGE(TAG, "Failed to create EventGroup");
    return;
    }
    telemetryQueue = xQueueCreate(20,sizeof(TelemetryPacket_t));
    if (telemetryQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create telemetry queue");
        return;
    }

    //inicjalizacja modulow 
    wifi_manager_init();
    vTaskDelay(pdMS_TO_TICKS(100));
    mqtt_manager_init();
    vTaskDelay(pdMS_TO_TICKS(100));
    time_manager_init();
    driving_score_init();
    vTaskDelay(pdMS_TO_TICKS(3000));
    init_gps_uart();
    vTaskDelay(pdMS_TO_TICKS(200));
    init_twai_obd();
    
    
   
    /* STARTING TASKS */
    xTaskCreatePinnedToCore(
        obd_task,
        "vTaskOBD",
        4096,
        NULL,
        5,
        NULL,
        1
    );
     xTaskCreatePinnedToCore(
        gps_task,
        "vTaskGPS",
        4096,
        NULL,
        4,
        NULL,
        1
    );
    xTaskCreatePinnedToCore(
        data_manager_task,
        "DataMgr",
        4096,
        NULL,
        3,
        NULL,
        0
    );
    xTaskCreatePinnedToCore(
        system_monitor_task,
        "SysMntr",
        4096,
        NULL,
        1,
        NULL,
        0
    ); 
}
    
    