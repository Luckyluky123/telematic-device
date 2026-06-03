#include "flash_storage.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h> 
static const char *TAG = "FLASH_STORAGE";

/* =====================================================
 * Internal state
 * ===================================================== */
static TelemetryPacket_t ram_buffer[RAM_BUFFER_SIZE];
static int buffer_count = 0;
static long read_offset = 0; 

static bool flash_ready = false;



esp_err_t init_flash_storage(void) {
     if (flash_ready) {
        return ESP_OK;
    }
    esp_vfs_littlefs_conf_t conf = {
        .base_path = STORAGE_BASE_PATH,
        .partition_label = "storage",
        .format_if_mount_failed = true,
        .dont_mount = false
    };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LittleFS mount failed: %s", esp_err_to_name(ret));
        flash_ready = false;
        return ret;
    }
    flash_ready = true;
    return ESP_OK;
}

esp_err_t storage_save_packet(TelemetryPacket_t *packet) {
    if (!flash_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    // add to bufor ram
    memcpy(&ram_buffer[buffer_count], packet, sizeof(TelemetryPacket_t));
    buffer_count++;

    if (buffer_count >= RAM_BUFFER_SIZE) {
        return storage_flush_to_flash();
    }
    return ESP_OK;
}

esp_err_t storage_flush_to_flash(void) {
    if (!flash_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    if (buffer_count == 0) return ESP_OK;

    FILE* f = fopen(DATA_FILE_PATH, "ab"); 
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for append");
        return ESP_FAIL;
    }

    size_t written = fwrite(ram_buffer, sizeof(TelemetryPacket_t), buffer_count, f);
    
    /* Ensure data is physically written to flash */
    fflush(f);
    fsync(fileno(f)); 
    fclose(f);

    if (written == buffer_count) {
        buffer_count = 0;
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Flash write error: %zu/%d written", written, buffer_count);
        return ESP_FAIL;
    }
}

bool storage_is_empty(void) {
    if (!flash_ready) {
        return true;
    }
    struct stat st;
    if (stat(DATA_FILE_PATH, &st) != 0) {
        return true; // no file
    }
    
    return (st.st_size == 0 || read_offset >= st.st_size);
}

bool storage_get_next_packet(TelemetryPacket_t *packet) {
    if (!flash_ready) {
        return false;
    }
    FILE* f = fopen(DATA_FILE_PATH, "rb");
    if (f == NULL) return false;

    // last read operation
    if (fseek(f, read_offset, SEEK_SET) != 0) {
        fclose(f);
        return false;
    }

    // read one structure of union
    if (fread(packet, sizeof(TelemetryPacket_t), 1, f) == 1) {
        read_offset += sizeof(TelemetryPacket_t);
        fclose(f);
        return true;
    }

    fclose(f);
    return false; // end of data
}

void storage_reset_read_and_clear(void) {
    if (!flash_ready) {
        ESP_LOGE(TAG, "Storage not initialized");
        return;
    }
    unlink(DATA_FILE_PATH);
    read_offset = 0;
    buffer_count = 0;
}

void print_storage_info(void) {
    if (!flash_ready) {
        ESP_LOGE(TAG, "FS not ready");
        return;
    }
    size_t total = 0, used = 0;
    if (esp_littlefs_info("storage", &total, &used) == ESP_OK) {
        ESP_LOGI(TAG, "Flash [storage] -> Total: %d KB, Used: %d KB", total/1024, used/1024);
    }
}