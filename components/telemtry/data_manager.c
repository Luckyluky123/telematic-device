/**
 * @file data_manager.c
 * @brief Telemetry data management state machine.
 *
 * Responsibilities:
 * - Real-time telemetry transmission via MQTT
 * - Local flash buffering when connectivity is unavailable
 * - Upload of buffered telemetry data
 * - Transition to deep sleep mode during inactivity
 */

#include "data_manager.h"
#include "commons.h"

#include "mqtt_manager.h"
#include "system_monitor.h"
#include "flash_storage.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "freertos/timers.h"

static const char *TAG = "DATA_MGR";

/* Timeouts */
#define UPLOAD_TIMEOUT_US   (5 * 60 * 1000000LL) // 5 minutes
#define IDLE_WAIT_US        (1 * 60 * 1000000LL) // 1 minute

/**
 * @brief Data manager state machine.
 */
typedef enum {
    STATE_IDLE_WAIT,    // Waiting for vehicle activity
    STATE_DRIVING_LIVE,  // Sending telemetry directly to MQTT
    STATE_DRIVING_BUFFER,  // Storing telemetry in flash memory
    STATE_UPLOAD_PENDING,  // Uploading buffered telemetry
    STATE_SLEEP_NOW  // Entering deep sleep mode
} SystemState_t;


const char* state_to_str(SystemState_t state) {
    switch (state) {
        case STATE_IDLE_WAIT:     return "IDLE_WAIT";
        case STATE_DRIVING_LIVE:  return "DRIVING_LIVE";
        case STATE_DRIVING_BUFFER: return "DRIVING_BUFFER";
        case STATE_UPLOAD_PENDING: return "UPLOAD_PENDING";
        case STATE_SLEEP_NOW:      return "SLEEP_NOW";
        default:                   return "UNKNOWN";
    }
}

int64_t last_can_activity_time = 0;

void data_manager_task(void *pvParameters) {

    /* Initialize state machine */
    SystemState_t current_state = STATE_IDLE_WAIT;
    SystemState_t prev_state    = -1;

    int64_t state_enter_time = esp_timer_get_time();

    bool has_new_data_packet = false;
    TelemetryPacket_t received_packet;

    init_flash_storage();
    ESP_LOGI(TAG, "Start");

    while(1) {
        int64_t now = esp_timer_get_time();

        EventBits_t bits = xEventGroupGetBits(systemEventGroup);
        bool net_ready = (bits & WIFI_BIT) && (bits & MQTT_CONNECTED_BIT);
        bool is_can  = (bits & CAN_BIT); 
        if (xQueueReceive(telemetryQueue, &received_packet, 0) == pdPASS) {
            has_new_data_packet = true;
        }


        switch(current_state) 
        {        
                case STATE_IDLE_WAIT:
                    if (is_can) {
                    current_state = net_ready ? STATE_DRIVING_LIVE : STATE_DRIVING_BUFFER;
                    }
                    else {
                        /* Upload buffered data if available */
                        if (net_ready && !storage_is_empty()) {  
                            current_state = STATE_UPLOAD_PENDING;
                        } 
                        /* Enter sleep after inactivity timeout */
                        else if ((now - state_enter_time) > IDLE_WAIT_US) {
                        current_state = STATE_SLEEP_NOW;
                        }
                    } 
                    break;
                case STATE_DRIVING_LIVE:
                    if (!is_can) current_state = (!storage_is_empty() && net_ready) ? STATE_UPLOAD_PENDING : STATE_IDLE_WAIT;
                    else if (!net_ready) current_state = STATE_DRIVING_BUFFER; //flash
                    break;

                case STATE_DRIVING_BUFFER:
                    if (!is_can) current_state = STATE_IDLE_WAIT; // Vehicle stopped 
                    else if (net_ready) current_state = STATE_DRIVING_LIVE; // Connectivity restored 
                    break;
                case STATE_UPLOAD_PENDING:
                    
                    if (is_can && net_ready) {
                        current_state = STATE_DRIVING_LIVE;
                    }
                    
                    else if ((now - state_enter_time) > UPLOAD_TIMEOUT_US || !net_ready) {
                        current_state = STATE_IDLE_WAIT;
                    }
                    break;
                case STATE_SLEEP_NOW:
                    break;

                default:
                     current_state = STATE_IDLE_WAIT;
                    break;
        }
        if (current_state != prev_state) {
             ESP_LOGI(TAG, "Stan: %s -> %s", state_to_str(prev_state), state_to_str(current_state));
            state_enter_time = now;
            prev_state = current_state;
        }


        /* ---- STATE ACTIONS ---- */
        switch (current_state) {
            case STATE_DRIVING_LIVE:
                if (has_new_data_packet) {
                mqtt_send_telemetry(&received_packet);
            }
            break;

            case STATE_DRIVING_BUFFER:
                if (has_new_data_packet) {
                    storage_save_packet(&received_packet);
                }
                break;
            case STATE_UPLOAD_PENDING:
                if(has_new_data_packet) {

                     /*
                     * Prioritize current telemetry data
                     * while uploading buffered records.
                     */
                    mqtt_send_telemetry(&received_packet);
                }
                else if (net_ready) {
                    TelemetryPacket_t buffered_packet;
                    if (storage_get_next_packet(&buffered_packet)) {
                        mqtt_send_telemetry(&buffered_packet);
                        vTaskDelay(pdMS_TO_TICKS(10)); //Delay
                    } else {
                          /*
                         * All buffered data uploaded.
                         */
                        storage_reset_read_and_clear();
                        current_state = STATE_IDLE_WAIT;
                    }
                }
                break;
            case STATE_SLEEP_NOW:
                storage_flush_to_flash();
                xEventGroupSetBits(systemEventGroup, SLEEP_PREPARE_BIT);
                vTaskDelay(pdMS_TO_TICKS(500));
                prepare_for_sleep();
                break;
            case STATE_IDLE_WAIT:
            default:
                break;
        }
        has_new_data_packet = false;

        xEventGroupSetBits(systemEventGroup, DM_HEARTBEAT_BIT);
        vTaskDelay(pdMS_TO_TICKS(100));
        

    }
}