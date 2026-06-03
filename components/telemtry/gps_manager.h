#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize GPS UART driver.
 */
void init_gps_uart(void);

/**
 * @brief FreeRTOS task handling GPS parsing and EKF fusion.
 */
void gps_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif