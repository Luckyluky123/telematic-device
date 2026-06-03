/**
 * @file obd_manager.c
 * 
*/


#include "obd_manager.h"
#include "commons.h"
#include "driving_score.h"

#include <string.h>

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "esp_err.h"

#include <time.h>

//#include "driver/twai.h"

static const char *TAG_OBD = "OBD_MANAGER";

/* --- OBD-II DEFINITIONS --- */

#define OBD_FUNCTIONAL_ID       0x7DF
#define ECU_ENGINE_RESPONSE_ID  0x7E8
#define OBD_SERVICE_SHOW_DATA   0x01
#define OBD_RESPONSE_SHOW_DATA  0x41
#define OBD_FRAME_LENGTH        0x02


/* --- PIN CONFIGURATION --- */

/** @brief Transmit pin (TX). */
#define TWAI_TX_PIN GPIO_NUM_4
/** @brief Receive pin (RX). */
#define TWAI_RX_PIN GPIO_NUM_5


// variables protecting against client error on the page 
uint32_t diag_start_time = 0;
const uint32_t DIAG_TIMEOUT_MS = 300000;  //5 minutes
#define ECU_TIMEOUT_TICKS 100 

// global vehicle speed variable
static volatile float g_vehicle_speed = 0.0f;

// structure defining PID queries
typedef struct {
    uint8_t pid; // PID code
    uint16_t interval; // how often to query in NORMAL mode
    uint16_t diag_interval; // how often to query in DIAG mode
    const char* nazwa; // name for logs
} OBDQuery_t;

/** @brief Structure representing a raw CAN frame for processing.
 */
typedef struct {
    uint32_t id;           /*!< CAN message identifier */
    uint8_t dlc;
    uint8_t data[8];       /*!< Data buffer (max 8 bytes) */
} RawCanFrame_t;

// query table
static const OBDQuery_t obd_table[] = {
    {0x0C,3,5,"RPM"},
    {0X0D,2,5,"SPEED"},
    //{0X10,5,50,"FUEL_RATE"},
    {0X05,10,10,"TEMP"},
    {0X1F,10,50,"RUNTIME"},
    {0X01,20,10,"DTC_COUNT"},
    {0X10,6,30,"MAF"},
    {0X2F,15,100,"FUEL_LEVEL"},
    //{0X42,100,200,"VOLTAGE"},

};
#define OBD_TABLE_SIZE (sizeof(obd_table)/sizeof(obd_table[0]))
// helper function for fuel consumption calculation
static float calculate_diesel_fuel_from_maf(float maf, float rpm, float speed)
{
    float fuel_rate = 0.0f;

    // IDLE
    if(speed < 1.0f && rpm > 600.0f){
        return 0.8f; // L/h
    }

    // ENGINE BRAKING (fuel cut)
    if(speed > 20.0f && rpm > 1200.0f && maf < 2.0f){
        return 0.0f;
    }

    // NORMAL DRIVING
    float fuel_lph = maf * 0.084f; // diesel conversion factor

    if(speed > 1.0f){
        fuel_rate = (fuel_lph / speed) * 100.0f; // L/100km
    } else {
        fuel_rate = fuel_lph; // L/h
    }

    return fuel_rate;
}

// parses OBD data and updates telemetry packet
void parse_obd_data(uint8_t pid, uint8_t A, uint8_t B, TelemetryPacket_t *packet){
    switch(pid) {
        case 0x0C: //RPM
            packet->data.obd.rpm = ((A*256)+B)/4;
            break;
        case 0x0D: //Speed
            packet->data.obd.speed = A;
            g_vehicle_speed = A;
            driving_score_update(packet);
            break;
        case 0x10: //fuel consumption
            float maf = ((A * 256.0f) + B) / 100.0f;
            float rpm = packet->data.obd.rpm;
            float speed = packet->data.obd.speed;
            float fuel_rate = calculate_diesel_fuel_from_maf(maf,rpm,speed);
            packet->data.obd.fuel_rate = fuel_rate;
            break;
        case 0x2F: //fuel level
            packet->data.obd.fuel_level = (100.0f/255.0f)*A;
            break;
        // case 0x42: //voltage
        //     packet->data.obd.voltage = ((A*256.0f)+B) / 1000.0f;
        //     break;
        case 0x05: //coolant temperature 
            packet->data.obd.coolant_temp = A-40;
            break;
        case 0x1F: //engine runtime
            packet->data.obd.run_time = (A*256) + B;
            break;
        case 0x01: //DTC
            packet->data.obd.dtc_count = A&0X7F;
            packet->data.obd.mil_status = (A & 0x80) >> 7;
            break;
        default:
            break;
    }

}

// speed getter
float obd_get_speed(void) {
    return g_vehicle_speed;
}

/* ─── Scoring thresholds ─────────────────────────────────────── */
#define THRESH_HARD_BRAKE   -3.0f  /* m/s²  — braking */
#define THRESH_HARD_ACC      2.5f  /* m/s²  — acceleration */
#define THRESH_HIGH_RPM      3500  /* rpm */
#define THRESH_COLD_ENGINE   70    /* °C */
 
#define PENALTY_HARD_BRAKE   2.0f
#define PENALTY_HARD_ACC     1.5f
#define PENALTY_HIGH_RPM     1.0f
#define PENALTY_COLD_ENGINE  5.0f

/* --- VARIABLES AND HANDLES  --- */

/**
 * @brief TWAI node handle.
 * Used for initialization and lifecycle management of the driver.
 */
static twai_node_handle_t node_hdl = NULL;


/** @brief Queue passing CAN frames from ISR to processing task. */
static QueueHandle_t xRawCanQueue = NULL;


/* --- CALLBACK FUNCTIONS (ISR) --- */
/**
 * @brief Callback triggered when a message is received.
 * 
 * This function is automatically called by the driver when a new message arrives.
 * Flow:
 * 1. Reads frame from buffer ('twai_node_receive_from_isr').
 * 2. Copies ID and data into RawCanFrame_t structure.
 * 3. Sends structure to 'xRawCanQueue'.
 * 
 * @note Blocking functions (e.g. vTaskDelay) must not be used here.
 * 
 * @param handle TWAI node handle triggering the callback.
 * 
 */
static bool twai_rx_cb(twai_node_handle_t handle,const twai_rx_done_event_data_t *edata,void *user_ctx);

/**
 * @brief Callback triggered when a state change occurs ('TWAI_ERROR_BUS_OFF'), indicating a critical error.
 */
static bool twai_state_change_cb(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *user_ctx);




void obd_shutdown(void) {
    if (node_hdl == NULL) return;
    twai_node_disable(node_hdl);
    twai_node_delete(node_hdl);
    node_hdl = NULL;
}

static bool IRAM_ATTR twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx) {
    static uint8_t recv_buff[8];


    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff),
    };
    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)){
        
        RawCanFrame_t frame_to_queue;
        frame_to_queue.id = rx_frame.header.id;
        frame_to_queue.dlc = rx_frame.header.dlc;
        uint8_t copy_len = (rx_frame.header.dlc > 8) ? 8 : rx_frame.header.dlc;
        memcpy(frame_to_queue.data, rx_frame.buffer, copy_len);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(xRawCanQueue,&frame_to_queue,&xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken == pdTRUE;
    }
    return false;
}

// CAN bus error code 0x02 -> NO ACK...
static bool twai_error_cb(twai_node_handle_t handle,const twai_error_event_data_t *edata,void *user_ctx)
{
    if (edata->err_flags.val == 100)

        ESP_EARLY_LOGI(TAG_OBD, "CAN bus error: 0x%lx", edata->err_flags.val);
    return false;
}

// callback describing node state
static bool twai_state_change_cb(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *user_ctx) {
    const char *twai_state_name[] ={
        "error_active","error_warning","error_passive","bus_off"
    };

    ESP_EARLY_LOGI(TAG_OBD, "TWAI state: %s -> %s",
                   twai_state_name[edata->old_sta],
                   twai_state_name[edata->new_sta]);
    return false;
}

// processes received CAN frame queue
static bool process_rx_queue(TelemetryPacket_t *packet,
                             bool *is_ecu_connected,
                             int *error_counter)
{
    bool got_response = false;
    RawCanFrame_t frame;

    while (xQueueReceive(xRawCanQueue, &frame, 0) == pdPASS) {

        if (frame.id != ECU_ENGINE_RESPONSE_ID)
            continue;

        if (frame.data[1] != OBD_RESPONSE_SHOW_DATA)
            continue;

        uint8_t pid = frame.data[2];

        if (!(*is_ecu_connected)) {
            *is_ecu_connected = true;
            xEventGroupSetBits(systemEventGroup, CAN_BIT);
            ESP_LOGI(TAG_OBD, "ECU connected");
        }

        if(frame.dlc < 5)
            continue;
        got_response = true;
        *error_counter = 0;
        parse_obd_data(pid, frame.data[3], frame.data[4], packet);
    }
    return got_response;
}

// sends OBD request frame
// esp_err_t allows detecting transmission errors; function returns non-ESP_OK if anything fails via twai_node_transmit
static esp_err_t send_obd_request(uint8_t service, uint8_t pid) {
    static uint8_t send_buff[8];
    static twai_frame_t tx_msg;

    send_buff[0] = OBD_FRAME_LENGTH;
    send_buff[1] = service;
    send_buff[2] = pid;
    for(int i = 3; i < 8; i++) send_buff[i] = 0x55;

    memset(&tx_msg, 0, sizeof(twai_frame_t));
    tx_msg.header.id  = OBD_FUNCTIONAL_ID;
    tx_msg.header.ide = false;
    tx_msg.header.fdf = false;
    tx_msg.header.dlc = 8;
    tx_msg.buffer     = send_buff;
    tx_msg.buffer_len = 8;

    esp_err_t err = twai_node_transmit(node_hdl, &tx_msg, 0);
    if (err != ESP_OK) return err;

    // wait until ISR finishes copying data to hardware — safe to overwrite after
    return twai_node_transmit_wait_all_done(node_hdl, 100);
}

// TWAI initialization
// send queue buffer size 5, otherwise error
void init_twai_obd(void) {
    xRawCanQueue = xQueueCreate(10,sizeof(RawCanFrame_t));

    if(xRawCanQueue ==NULL) {
        ESP_LOGE(TAG_OBD,"CRITICAL ERROR: failed to create queue");
        esp_restart();
        return;
    }
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = TWAI_TX_PIN,
        .io_cfg.rx = TWAI_RX_PIN,
        .bit_timing.bitrate = 500000,
        .tx_queue_depth = 5,
        .fail_retry_cnt = 0,
        .intr_priority = 1
    };
    // create node instance
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config,&node_hdl));


    // register callbacks 
    twai_event_callbacks_t user_cbs = {
        .on_rx_done = twai_rx_cb,
        .on_error = twai_error_cb,
        .on_state_change = twai_state_change_cb,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_hdl,&user_cbs,NULL));



    // set filter for ECU engine address 0x7E8

    twai_mask_filter_config_t filter = {
        .id = ECU_ENGINE_RESPONSE_ID,
        .mask = 0x7FF,
        .is_ext = false,
    };
    ESP_ERROR_CHECK(twai_node_config_mask_filter(node_hdl,0,&filter));

    // enable node
    ESP_ERROR_CHECK(twai_node_enable(node_hdl));
    ESP_LOGI(TAG_OBD, "TWAI started!");
    driving_score_init();
}

/**
 * @brief BUS-OFF recovery procedure based on Espressif example.
 *
 * Calls twai_node_recover() and then polls twai_node_get_info()
 * until node returns to TWAI_ERROR_ACTIVE (or timeout occurs).
 *
 * @return true if recovery succeeded, false on timeout.
 */
static bool perform_bus_off_recovery( bool *is_ecu_connected, int *error_counter,
                                     uint64_t *global_tick)
                            
{
    ESP_LOGI(TAG_OBD,"node inactive, starting recovery");
    xQueueReset(xRawCanQueue);
    xEventGroupClearBits(systemEventGroup, CAN_BIT);
    *is_ecu_connected = false;
    *error_counter    = 0;
    *global_tick      = 0;

    ESP_ERROR_CHECK(twai_node_recover(node_hdl));
    bool recovered = false;
    for (uint8_t i = 0; i<100;i++){
        vTaskDelay(pdMS_TO_TICKS(100));
        twai_node_status_t node_status;
        twai_node_get_info(node_hdl,&node_status,NULL);
        if(node_status.state == TWAI_ERROR_ACTIVE) {
            ESP_LOGI(TAG_OBD,"node recovered");
            recovered = true;
            break;
        }
    }
    if(!recovered) {
        esp_restart();
    }
    return recovered;

}

// main logic
void obd_task( void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(2000));

    if (node_hdl == NULL) {
        ESP_LOGE(TAG_OBD, "CRITICAL ERROR: node_hdl is NULL!");
        vTaskDelete(NULL);
        return;
    }

    uint64_t global_tick = 0;
    TelemetryPacket_t package_to_send;

    // connection error counter
    int error_counter = 0;

    bool is_ecu_connected = false;
    bool prev_diag_mode = false;
    uint64_t last_response_tick = 0;

    // clear structure
    memset(&package_to_send,0,sizeof(package_to_send));
    package_to_send.type = DATA_TYPE_OBD;
    
    ESP_LOGI(TAG_OBD, "Starting OBD task...");
    while(1)  {
        twai_node_status_t node_status;
        twai_node_get_info(node_hdl,&node_status,NULL);
        if(node_status.state == TWAI_ERROR_BUS_OFF) {
            perform_bus_off_recovery(&is_ecu_connected,&error_counter,&global_tick);
            continue;
        }

        // check diagnostic mode bit
        EventBits_t bits = xEventGroupGetBits(systemEventGroup);
        bool is_diag_mode = (bits & DIAG_MODE_BIT);

        if (is_diag_mode && !prev_diag_mode) {
            diag_start_time = pdTICKS_TO_MS(xTaskGetTickCount());
            ESP_LOGI(TAG_OBD, "Diagnostic mode enabled, timer started");
        }
        if (!is_diag_mode && prev_diag_mode) {
            // diagnostic mode disabled
            diag_start_time = 0;
            ESP_LOGI(TAG_OBD, "Diagnostic mode disabled");
        }

        prev_diag_mode = is_diag_mode;

        if (is_diag_mode) {
            uint32_t current_time = pdTICKS_TO_MS(xTaskGetTickCount());
            uint32_t elapsed_time = current_time - diag_start_time;
            
            if (elapsed_time > DIAG_TIMEOUT_MS) {
                xEventGroupClearBits(systemEventGroup, DIAG_MODE_BIT);
                ESP_LOGW(TAG_OBD, "Diagnostic timeout");
                is_diag_mode = false;
                diag_start_time = 0;
            }
        }


        for (int i = 0; i < OBD_TABLE_SIZE; i++) {
            uint16_t interval = is_diag_mode
                        ? obd_table[i].diag_interval
                        : obd_table[i].interval;
            if (interval == 0) continue;
    
            // do not send if queue is overloaded (high error counter and no ECU)
            if (!is_ecu_connected && error_counter > 20) continue;
    
            if (((global_tick + i) % interval) == 0) {
                if (send_obd_request(OBD_SERVICE_SHOW_DATA, obd_table[i].pid) != ESP_OK) {
                    error_counter++;
                }
            }
        }
        bool got_response = process_rx_queue(&package_to_send, &is_ecu_connected, &error_counter);

        // ECU timeout handling (engine off)
        if (is_ecu_connected) {
            if (got_response) {
                last_response_tick = global_tick;
            } else if ((global_tick - last_response_tick) > ECU_TIMEOUT_TICKS) {
                ESP_LOGW(TAG_OBD, "ECU timeout — no response for 5s, disconnecting");
                xEventGroupClearBits(systemEventGroup, CAN_BIT);
                is_ecu_connected = false;
                error_counter = 0;
                last_response_tick = 0;
            }
        }

        // too many TX errors handling
        if (error_counter > 120 && is_ecu_connected) {
            ESP_LOGW(TAG_OBD, "Too many transmission errors (%d) - disconnecting ECU", error_counter);
            xEventGroupClearBits(systemEventGroup, CAN_BIT);
            is_ecu_connected = false;
            error_counter = 0;
            last_response_tick = 0;
        }


        if ((global_tick % 10 == 0) && is_ecu_connected) {
            package_to_send.timestamp = (uint32_t)time(NULL);
            xQueueSend(telemetryQueue, &package_to_send, 0);

        }

    global_tick++;
    vTaskDelay(pdMS_TO_TICKS(50));
    }
}