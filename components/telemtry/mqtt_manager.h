#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include "commons.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize MQTT client (does not connect yet)
 */
void mqtt_manager_init(void);

/**
 * @brief Start MQTT connection (called after WiFi connect)
 */
void mqtt_app_start(void);

/**
 * @brief Stop MQTT connection (called on WiFi loss)
 */
void mqtt_app_stop(void);

/**
 * @brief Publish telemetry packet to broker
 */
void mqtt_send_telemetry(TelemetryPacket_t *packet);

#ifdef __cplusplus
}
#endif

#endif