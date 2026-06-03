#include "time_manager.h"
 
#include <string.h>
#include <sys/time.h>
 
#include <esp_timer.h>
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
 
static const char *TAG = "TIME_MGR";



static RTC_DATA_ATTR time_t   rtc_saved_time   = 0;  // Unix timestamp zapisany przed snem
static RTC_DATA_ATTR uint32_t rtc_magic        = 0;  // Sentinel — sprawdza czy data są ważne
static RTC_DATA_ATTR uint64_t rtc_counter_before  = 0;   // esp_timer_get_time() / 1000 przed snem [ms]

#define RTC_MAGIC_VALUE 0xC0FFEE42

static TimeSource_t s_source = TIME_SOURCE_NONE;
static bool         s_sntp_running = false;

static void set_system_time(time_t t, TimeSource_t source, const char *label){
    struct timeval tv = {.tv_sec = t, .tv_usec = 0};
    settimeofday(&tv,NULL);
    s_source = source;

    struct tm tm_info;
    localtime_r(&t, &tm_info);
}
static void sntp_sync_callback(struct timeval *tv) {
    s_source = TIME_SOURCE_SNTP;
    struct tm tm_info;
    localtime_r(&tv->tv_sec, &tm_info);
}

void time_manager_init (void) {
    //ustawienie strfy czasowej
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();

    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_DEEPSLEEP && rtc_magic == 
        RTC_MAGIC_VALUE && rtc_saved_time > 0) {
        uint64_t counter_after = (uint64_t)esp_timer_get_time();
        uint64_t sleep_us = counter_after - rtc_counter_before;
        time_t sleep_seconds = (time_t)(sleep_us/ 1000000ULL);
        time_t restored = rtc_saved_time + sleep_seconds;

        set_system_time(restored, TIME_SOURCE_RTC, "RTC");
        }else if (reason == ESP_RST_DEEPSLEEP && rtc_magic != RTC_MAGIC_VALUE) {
            ESP_LOGW(TAG, "Wybudzenie z deep sleep ale brak zapisanego czasu");
            }
}

void time_manager_set_from_gps(int day, int month, int year,
                                int hour, int min, int sec)
        
{
    static bool gps_time_set_once = false;
    struct tm tm_gps = {
        .tm_year  = year + 100,   
        .tm_mon   = month - 1,    
        .tm_mday  = day,
        .tm_hour  = hour,
        .tm_min   = min,
        .tm_sec   = sec,
        .tm_isdst = 0,
    };
 
    const char *saved_tz = getenv("TZ");
    setenv("TZ", "UTC0", 1);
    tzset();
    time_t t = mktime(&tm_gps);
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();

    if (t == (time_t)-1) {
        ESP_LOGE(TAG, "GPS: błąd konwersji czasu!");
        return;
    }

     if (!gps_time_set_once) {
        set_system_time(t, TIME_SOURCE_GPS, "GPS");
        gps_time_set_once = true;
    } else {
        struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        s_source = TIME_SOURCE_GPS;
    }
}

void time_manager_start_sntp(void)
{
    if (s_sntp_running) return;
 
    ESP_LOGI(TAG, "Uruchamiam SNTP (pool.ntp.org)...");
 
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");  // backup
    sntp_set_time_sync_notification_cb(sntp_sync_callback);
 
    esp_sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
 
    esp_sntp_init();
    s_sntp_running = true;
}

void time_manager_stop_sntp(void)
{
    if (!s_sntp_running) return;
    esp_sntp_stop();
    s_sntp_running = false;
    ESP_LOGI(TAG, "SNTP zatrzymany");
}

void time_manager_save_to_rtc(void)
{
    if (s_source == TIME_SOURCE_NONE) {
        rtc_magic = 0;
        return;
    }
 
    time_t now;
    time(&now);
 
    rtc_saved_time     = now;
    rtc_counter_before = (uint64_t)esp_timer_get_time(); 
    rtc_magic          = RTC_MAGIC_VALUE;
 
    struct tm tm_info;
    localtime_r(&now, &tm_info);
}

//helpers

bool time_manager_is_set(void)
{
    return s_source != TIME_SOURCE_NONE;
}
 
TimeSource_t time_manager_get_source(void)
{
    return s_source;
}
 
const char* time_manager_source_str(TimeSource_t src)
{
    switch (src) {
        case TIME_SOURCE_NONE: return "NULL";
        case TIME_SOURCE_RTC:  return "RTC (deep sleep)";
        case TIME_SOURCE_GPS:  return "GPS";
        case TIME_SOURCE_SNTP: return "SNTP (WiFi)";
        default:               return "UNKNOWN";
    }
}