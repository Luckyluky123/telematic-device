#pragma once

#include "commons.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file driving_score.h
 * @brief Driving behavior scoring module.
 *
 * Calculates driver performance score based on:
 * - acceleration
 * - braking
 * - RPM usage
 * - engine temperature
 */

/**
 * @brief Initialize driving score system.
 *
 * Must be called once at system startup.
 */
void driving_score_init(void);

/**
 * @brief Update driving score using latest telemetry data.
 *
 * @param packet Pointer to telemetry packet (OBD data required).
 */
void driving_score_update(TelemetryPacket_t *packet);

/**
 * @brief Reset score to default state (100 points).
 */
void driving_score_reset(void);

/**
 * @brief Save current score to RTC memory (deep sleep persistence).
 */
void driving_score_save(void);

#ifdef __cplusplus
}
#endif