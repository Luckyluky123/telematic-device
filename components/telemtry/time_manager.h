#ifndef TIME_MANAGER_H
#define TIME_MANAGER_H
 
#include <stdbool.h>
#include <time.h>
 
#ifdef __cplusplus
extern "C" {
#endif

// Last time synchronization source

typedef enum {
    TIME_SOURCE_NONE = 0,
    TIME_SOURCE_RTC,
    TIME_SOURCE_GPS,
    TIME_SOURCE_SNTP,
} TimeSource_t;

// Initialize time manager  
void time_manager_init(void);

// Set time from GPS
void time_manager_set_from_gps(int day, int month,int year,
                                int hour, int min, int sec);

// SNTP
void time_manager_start_sntp(void);

void time_manager_stop_sntp(void);

// Save time to RTC
void time_manager_save_to_rtc(void);

bool time_manager_is_set(void);

TimeSource_t time_manager_get_source(void);

const char* time_manager_source_str(TimeSource_t src);

#ifdef __cplusplus
}
#endif
 
#endif