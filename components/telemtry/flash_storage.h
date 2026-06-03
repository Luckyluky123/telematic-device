#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stdint.h>
#include "esp_err.h"
#include "commons.h"
#include <stdbool.h>

#define STORAGE_BASE_PATH "/littlefs"
#define DATA_FILE_PATH    "/littlefs/data.bin"
#define RAM_BUFFER_SIZE   20   // Zapisujemy we Flashu po zebraniu 20 paczek


esp_err_t init_flash_storage(void);
esp_err_t storage_save_packet(TelemetryPacket_t *p);
esp_err_t storage_flush_to_flash(void); // <--- Musi tu być!
bool storage_is_empty(void);
bool storage_get_next_packet(TelemetryPacket_t *p);
void storage_reset_read_and_clear(void);
void print_storage_info(void);
#endif