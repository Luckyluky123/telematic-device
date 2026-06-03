#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "freertos/timers.h"
#include "time_manager.h"

// Libraries for provisioning
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_ble.h"

#include "mqtt_manager.h"
#include "commons.h" 

// =============================================
// WiFi configuration data
// =============================================
#define MAX_PROB_POLACZENIA 8
#define CZAS_OCZEKIWANIA_MS 5000
#define MAX_CYKLI_ROZLACZEN 10
// =============================================

static const char *TAG = "WIFI_MGR";
static int s_retry_num = 0;
static int s_total_disconnect_cycles = 0;

// Flag for esp_wifi_connect()
static volatile bool s_connecting = false;



static TimerHandle_t wifi_reconnect_timer;

static void wifi_reconnect_timer_callback(TimerHandle_t xTimer)
{
     if (systemEventGroup != NULL) {
        xEventGroupSetBits(systemEventGroup, WIFI_RECONNECT_BIT);
    }
}

static void wifi_control_task(void *pvParameters)
{
    while (1) {
        // Wait for RECONNECT bit (blocking wait)
        EventBits_t bits = xEventGroupWaitBits(systemEventGroup, 
                                              WIFI_RECONNECT_BIT, 
                                              pdTRUE,    // Clear bit after reading
                                              pdFALSE, 
                                              portMAX_DELAY);

        if (bits & WIFI_RECONNECT_BIT) {
            ESP_LOGI(TAG, "Task: Reconnect signal received. Retrying connection...");
            s_retry_num = 0; 
            s_connecting = false;
            esp_wifi_connect();
            s_connecting = true;
        }
    }
}

static void prov_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_PROV_EVENT) {
        switch (event_id) {
            case WIFI_PROV_START:
                ESP_LOGI(TAG, "Provisioning started. Waiting for Bluetooth app connection...");
                break;
            case WIFI_PROV_CRED_RECV: {
                wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
                ESP_LOGI(TAG, "Received data from phone! SSID: %s", (const char *) wifi_sta_cfg->ssid);
                break;
            }
            case WIFI_PROV_CRED_FAIL: {
                wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
                ESP_LOGE(TAG, "WiFi credentials validation error\nReason: %s",
                         (*reason == WIFI_PROV_STA_AUTH_ERROR) ? "Wrong password" : "AP not found");
                break;
            }
            case WIFI_PROV_CRED_SUCCESS:
                ESP_LOGI(TAG, "Provisioning completed successfully!");
                break;
            case WIFI_PROV_END:
                ESP_LOGI(TAG, "Shutting down provisioning manager (freeing RAM)");
                wifi_prov_mgr_deinit();
                break;
            default:
                break;
        }
    }
}

/*
 * WiFi and IP event handler
 */
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    // 1. First connection attempt after startup
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        bool provisioned = false;
        wifi_prov_mgr_is_provisioned(&provisioned);
        if (provisioned) {
            esp_wifi_connect();
        } else {
            ESP_LOGI(TAG, "WiFi started, waiting for phone provisioning data...");
        }
    } 
    // 2. WiFi disconnected (or connection failed)
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = event_data;
        ESP_LOGW(TAG, "Disconnect reason: %d", event->reason);
        // Clear system bit indicating no internet connection
        if (systemEventGroup != NULL) {
            xEventGroupClearBits(systemEventGroup, WIFI_BIT);
        }
        mqtt_app_stop(); 
        time_manager_stop_sntp();

        if (s_retry_num < MAX_PROB_POLACZENIA) {
            s_retry_num++;
            ESP_LOGI(TAG, "Retrying AP connection... (Attempt %d/%d)", s_retry_num, MAX_PROB_POLACZENIA);
            s_connecting = true;
            esp_wifi_connect();
        } else {
            s_total_disconnect_cycles++;
            ESP_LOGW(TAG, "Failed to connect to AP. Waiting 5 seconds before next attempt.");
            if (s_total_disconnect_cycles >= MAX_CYKLI_ROZLACZEN) {
                ESP_LOGE(TAG, "Long-term WiFi loss! Forgetting network and switching to Bluetooth provisioning...");
                wifi_prov_mgr_deinit();
                wifi_prov_mgr_reset_provisioning();
                vTaskDelay(pdMS_TO_TICKS(1000)); 
                esp_restart(); 
                } else {
                    if(wifi_reconnect_timer != NULL) {
                        xTimerStart(wifi_reconnect_timer, 0);
                    } else {
                        ESP_LOGE(TAG, "Timer does not exist! Retrying reconnect directly in 5s...");
                        s_retry_num = 0;
                        s_connecting = true;
                        esp_wifi_connect();
                    }
                } 
        }
    }
    // 3. IP obtained (Success!)
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        
        s_retry_num = 0;
        s_connecting = false;
        s_total_disconnect_cycles = 0;
        
        if(wifi_reconnect_timer != NULL) {
            xTimerStop(wifi_reconnect_timer, 0);
        }
        // Set system flag indicating WiFi is connected
        if (systemEventGroup != NULL) {
            xEventGroupSetBits(systemEventGroup, WIFI_BIT);
            ESP_LOGI(TAG, "WIFI CONNECTED (Bit set)");
            mqtt_app_start(); 
            time_manager_start_sntp();
        }
    }
}

void wifi_manager_init(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    
    wifi_reconnect_timer = xTimerCreate("WifiTimer", 
                                        pdMS_TO_TICKS(CZAS_OCZEKIWANIA_MS), 
                                        pdFALSE, 
                                        NULL, 
                                        wifi_reconnect_timer_callback);


    if (wifi_reconnect_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create timer!");
    }
    xTaskCreate(wifi_control_task, "wifi_ctrl_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Starting WiFi...");
    // Initialize TCP/IP stack and default network interface
    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_PROV_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &prov_event_handler, 
                                                        NULL, 
                                                        NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM
    };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));
    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (!provisioned) {
        ESP_LOGI(TAG, "No saved WiFi found. Starting Bluetooth provisioning...");
        
        // Start Bluetooth pairing for provisioning
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_0, NULL, "OBD2_Car", NULL));
        
    } else {
        ESP_LOGI(TAG, "Saved WiFi found in memory. Connecting...");
        
        
        wifi_prov_mgr_deinit();
        
        
        ESP_ERROR_CHECK(esp_wifi_start());
    }
}