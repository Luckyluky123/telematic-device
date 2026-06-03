#ifndef OBD_MANAGER_H
#define OBD_MANAGER_H


#include "commons.h"

// Function to initialize the TWAI driver

void init_twai_obd(void);

void parse_obd_data(uint8_t pid, uint8_t A, uint8_t B,
                    TelemetryPacket_t *packet);

// FreeRTOS task handling OBD
void obd_task(void *pvParameters);

void obd_shutdown(void);

float obd_get_speed(void);
#endif