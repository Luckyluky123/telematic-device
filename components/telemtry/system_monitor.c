#include "obd_manager.h"
#include "commons.h"
#include "driving_score.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_timer.h"
#include "obd_manager.h"
#include "time_manager.h"

static const char *TAG = "SYS_MON";

#define RX_PIN GPIO_NUM_5

void prepare_for_sleep(void) {
    esp_sleep_enable_ext0_wakeup(RX_PIN, 0);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RX_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Pull GPIO4 to LOW to wake up");
    vTaskDelay(pdMS_TO_TICKS(100));

    time_manager_stop_sntp();
    time_manager_save_to_rtc();
    driving_score_save();
    obd_shutdown();
    esp_deep_sleep_start();

}
void system_monitor_task(void *pvParameters) {
    ESP_LOGI(TAG, "System Monitor started");
    int64_t last_dm_hb = esp_timer_get_time();
    const int64_t DM_TIMEOUT_US = 20000000;

    while (1)
    {
        EventBits_t bits = xEventGroupGetBits(systemEventGroup);
        int64_t now = esp_timer_get_time();
        /* === HEARTBEAT: DATA MANAGER === */
        if (bits & DM_HEARTBEAT_BIT)
        {
            last_dm_hb = now;
            xEventGroupClearBits(systemEventGroup, DM_HEARTBEAT_BIT);
        }


        if (!(bits & SLEEP_PREPARE_BIT)) {  
            if (now - last_dm_hb > DM_TIMEOUT_US) {
                esp_restart();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

    }

}