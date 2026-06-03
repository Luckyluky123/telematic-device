idf_component_register( SRCS "driving_score.c" "time_manager.c" "flash_storage.c" "system_monitor.c" "mqtt_manager.c" "gps_manager.c" "obd_manager.c" "wifi_manager.c" "data_manager.c" "esp_littlefs.h"
    INCLUDE_DIRS "."
    REQUIRES 
        driver        
        esp_timer 
        freertos 
        log 
        esp_wifi        
        esp_event       
        nvs_flash       
        esp_netif       
        lwip  
        esp_common
        esp_system      
        mqtt   
        esp_hw_support
        wifi_provisioning
        protocomm 
        bt
)
