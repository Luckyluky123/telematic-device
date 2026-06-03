#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include <time.h>

#include "commons.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
static const char *TAG = "MQTT_MGR";

/* =====================================================
 * MQTT configuration
 * ===================================================== */

#define MQTT_BROKER_URI "mqtt://34.116.210.120"
#define MQTT_USER "esp_klient"
#define MQTT_PASS "joanna123"

extern uint32_t diag_start_time;

//client
static esp_mqtt_client_handle_t client = NULL;
static bool is_started = false;

//event logic handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGI(TAG, "Connecting to broker: %s", MQTT_BROKER_URI);
            break;

        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            if (systemEventGroup != NULL) {
                xEventGroupSetBits(systemEventGroup, MQTT_CONNECTED_BIT);
            }
            esp_mqtt_client_subscribe(client, "esp32/cmd", 1);
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            if (systemEventGroup != NULL) {
                xEventGroupClearBits(systemEventGroup, MQTT_CONNECTED_BIT);
            }
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT: Błąd!");
            if (event->error_handle) {
                ESP_LOGE(TAG, "  Error type: %d", event->error_handle->error_type);
                ESP_LOGE(TAG, "  Connect return code: %d", event->error_handle->connect_return_code);
                if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                    ESP_LOGE(TAG, "  ESP-TLS error: 0x%x", event->error_handle->esp_tls_last_esp_err);
                    ESP_LOGE(TAG, "  TLS stack error: 0x%x", event->error_handle->esp_tls_stack_err);
                    ESP_LOGE(TAG, "  Socket errno: %d", event->error_handle->esp_transport_sock_errno);
                }
            }
            break;

        case MQTT_EVENT_DATA:
            if (strncmp(event->topic, "esp32/cmd", event->topic_len) == 0){
                ESP_LOGI(TAG, "Command received: %.*s",
                     event->data_len,
                     event->data);
                if (strncmp(event->data, "START_DIAG", event->data_len) == 0) {
                    xEventGroupSetBits(systemEventGroup, DIAG_MODE_BIT);
                    diag_start_time = pdTICKS_TO_MS(xTaskGetTickCount());
                } else if (strncmp(event->data, "STOP_DIAG", event->data_len) == 0) {
                    xEventGroupClearBits(systemEventGroup, DIAG_MODE_BIT);
                }
            }
            break;
            
        default:
            break;
    }
}

void mqtt_manager_init(void) {
    if (client != NULL) return; 

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.username = MQTT_USER,
        .credentials.authentication.password = MQTT_PASS,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == NULL) {
        ESP_LOGE(TAG, "Błąd inicjalizacji klienta MQTT");
        return;
    }

    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    ESP_LOGI(TAG, "MQTT Zainicjalizowany (oczekiwanie na start przez WiFi)");
}

/**
 * 2. Start after Wifi gets IP
 */
void mqtt_app_start(void) {
    if (client != NULL && !is_started) {
        esp_err_t err = esp_mqtt_client_start(client);
        if (err == ESP_OK) {
            is_started = true;
        }
    }
}

/**
 * 3. Stop by Wifi manager
 */
void mqtt_app_stop(void) {
    if (client != NULL && is_started) {
        esp_mqtt_client_stop(client);
        is_started = false;
    }
}

/**
 * 4. Data sending
 */
void mqtt_send_telemetry(TelemetryPacket_t *packet) {
    if (client == NULL || !is_started) return;

    EventBits_t uxBits = xEventGroupGetBits(systemEventGroup);
    if (!(uxBits & MQTT_CONNECTED_BIT)) {
        return; 
    }
    time_t t = packet -> timestamp;
    char ts[25];
    struct tm timeinfo;

    
    gmtime_r(&t, &timeinfo);
    
    if(timeinfo.tm_year < 100){
        return;
    }

    strftime(ts,sizeof(ts),"%Y-%m-%d %H:%M:%S", &timeinfo);
    
    char payload[512];
    const char *topic;

    if (packet->type == DATA_TYPE_OBD) {
        topic = "esp32/telemetry";
        float fuel_level_f = (float)packet->data.obd.fuel_level;
        snprintf(payload, sizeof(payload),  
                "{"
                "\"device_id\":\"CAR_01\","
                "\"timestamp\":\"%s\","
                "\"rpm\":%u,"
                "\"speed\":%u,"
                "\"fuel_level\":%.2f,"
                "\"fuel_rate\":%.2f,"
                "\"run_time\":%lu,"
                "\"coolant_temp\":%d,"
                "\"dtc_count\":%u,"
                "\"mil_status\":%u,"
                "\"score\":%.2f"
                "}",
                ts, 
                packet->data.obd.rpm,
                packet->data.obd.speed,
                fuel_level_f,
                packet->data.obd.fuel_rate,
                packet->data.obd.run_time,
                packet->data.obd.coolant_temp,
                packet->data.obd.dtc_count,
                packet->data.obd.mil_status,
                packet->data.obd.driving_score
        );
    }
    else if (packet->type == DATA_TYPE_GPS) {
        topic = "esp32/gps"; 
        double lat_f = (double)packet->data.gps.lat / 10000000.0;
        double lon_f = (double)packet->data.gps.lon / 10000000.0;

        snprintf(payload, sizeof(payload),
            "{"
            "\"device_id\":\"CAR_01\","
            "\"timestamp\":\"%s\","
            "\"lat\":%.7f,"
            "\"lon\":%.7f"
            "}",
            ts,
            lat_f,
            lon_f
        );

    } else {
        return;
    }

    int msg_id = esp_mqtt_client_publish(client, topic, payload, 0, 1, 0); // QoS 1 
    
    if (msg_id != -1) {
        return;
    } else {
        ESP_LOGE(TAG, "ERROR PUBLISHING");
    }
}