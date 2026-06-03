#include "driving_score.h"

#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <esp_system.h>

static const char *TAG_SCORE = "DRIVING_SCORE";

/* =====================================================
 * Thresholds
 * ===================================================== */
#define THRESH_BRAKE_SMALL   -2.8f
#define THRESH_BRAKE_NORMAL  -3.4f
#define THRESH_BRAKE_HIGH    -4.1f

#define THRESH_ACC_SMALL      2.7f   
#define THRESH_ACC_NORMAL     3.5f   
#define THRESH_ACC_HIGH       4.1f  

/* =====================================================
 * Penalties
 * ===================================================== */
#define PENALTY_BRAKE_BASE    1.2f
#define PENALTY_ACC_BASE      1.2f
#define PENALTY_HIGH_RPM      1.5f

#define THRESH_HIGH_RPM       3500
#define THRESH_COLD_ENGINE    70
/* =====================================================
 * Timing
 * ===================================================== */
#define COOLDOWN_US           (30LL * 1000000LL)  
#define REPAIR_INTERVAL_US    (60LL * 1000000LL)   

/* =====================================================
 * Multipliers
 * ===================================================== */
#define MULT_SMALL            0.5f
#define MULT_NORMAL           1.0f
#define MULT_HIGH             1.5f
#define COLD_ENGINE_MULT      2.0f   

#define SCORE_RTC_MAGIC 0XABCD1234UL

static RTC_DATA_ATTR uint32_t rtc_score_magic;
static RTC_DATA_ATTR float rtc_score;
static RTC_DATA_ATTR float rtc_session_min_score;


static float s_session_min_score = 100.0f;


/* =====================================================
 * Internal state
 * ===================================================== */
static struct {
    float   score;
    float   prev_speed_ms;
    int64_t prev_time_us;
    bool    first_sample;

    
    int64_t last_brake_us;
    int64_t last_acc_us;
    int64_t last_rpm_us;

    
    bool hard_brake_active;
    bool hard_acc_active;
    bool high_rpm_active;
    bool cold_engine_active;

     
    int64_t last_penalty_us;
    bool    repair_timer_started;

} s_state;

/* =====================================================
 * Helpers
 * ===================================================== */
static float get_multiplier(float abs_val, float thresh_small, float thresh_normal, float thresh_high)
{
    if (abs_val >= thresh_high)   return MULT_HIGH;
    if (abs_val >= thresh_normal) return MULT_NORMAL;
    if (abs_val >= thresh_small)  return MULT_SMALL;
    return 0.0f;
}

static float get_score_cap(void)
{
    if (s_session_min_score <= 10.0f) return 50.0f;
    if (s_session_min_score <= 30.0f) return 65.0f;
    if (s_session_min_score <= 50.0f) return 80.0f;
    if (s_session_min_score <= 70.0f) return 90.0f;
    return 100.0f;
}
/* ─── API ─────────────────────────────────────────────────── */

void driving_score_init(void)
{
    esp_reset_reason_t reason = esp_reset_reason();

    if (reason == ESP_RST_DEEPSLEEP && rtc_score_magic == SCORE_RTC_MAGIC &&
        rtc_score >= 0.0f && rtc_score <=100.0f){
            driving_score_reset();
            s_state.score = rtc_score;
            s_session_min_score = rtc_session_min_score;
        }
    else {
        driving_score_reset();
    }
}

void driving_score_reset(void)
{
    s_state.score = 100.0f;
    s_session_min_score = 100.0f;
    s_state.prev_speed_ms = 0.0f;
    s_state.prev_time_us = 0;
    s_state.first_sample = true;

    s_state.last_brake_us      = -COOLDOWN_US;
    s_state.last_acc_us        = -COOLDOWN_US;
    s_state.last_rpm_us        = -COOLDOWN_US;

    s_state.hard_brake_active  = false;
    s_state.hard_acc_active    = false;
    s_state.high_rpm_active    = false;
    s_state.cold_engine_active = false;

    s_state.last_penalty_us    = 0;
    s_state.repair_timer_started = false;

    rtc_score_magic = 0;


}
void driving_score_save(void)
{
    rtc_score             = s_state.score;
    rtc_session_min_score = s_session_min_score;
    rtc_score_magic       = SCORE_RTC_MAGIC;
}

void driving_score_update(TelemetryPacket_t *packet)
{

    if (packet == NULL) return;

    OBDFrame_t *obd = &packet->data.obd;
    int64_t    now_us = esp_timer_get_time();
    bool cold_engine  = (obd->coolant_temp < THRESH_COLD_ENGINE);

    float acceleration = 0.0f;
    float speed_ms     = obd->speed / 3.6f;

    
    if (!s_state.first_sample) {
    
        float dt = (s_state.prev_time_us == 0) ? 1.0f : (float)(now_us - s_state.prev_time_us) / 1000000.0f;
        if (dt <= 0.0f || dt > 2.0f) dt = 1.0f;
        acceleration = (speed_ms - s_state.prev_speed_ms) / dt;
        if (speed_ms < 2.0f || dt < 0.2f) {
            acceleration = 0.0f;
        }
    }
    

    s_state.prev_speed_ms = speed_ms;
    s_state.prev_time_us  = now_us;
    s_state.first_sample  = false;

    if (fabs(acceleration) < 0.5f)
        acceleration = 0.0f;

    float penalty = 0.0f;



    /* ══════════════════════════════════════════════
       HARD BRAKE
       ══════════════════════════════════════════════ */
    float abs_brake = -acceleration;
    if (acceleration < THRESH_BRAKE_SMALL) {

        float mult = get_multiplier(abs_brake,
                                    -THRESH_BRAKE_SMALL,
                                    -THRESH_BRAKE_NORMAL,
                                    -THRESH_BRAKE_HIGH);

        bool is_high_penalty = (mult >= MULT_HIGH);
        int64_t elapsed      = now_us - s_state.last_brake_us;
        bool in_cooldown     = (elapsed < COOLDOWN_US);

        
        if (!s_state.hard_brake_active) {
            
            if (!in_cooldown || is_high_penalty) {
                penalty += PENALTY_BRAKE_BASE * mult;
                s_state.last_brake_us = now_us;
            }
            s_state.hard_brake_active = true;
        }
    } else {
        s_state.hard_brake_active = false;
    }

    /* ══════════════════════════════════════════════
       HARD ACCELERATION
       ══════════════════════════════════════════════ */
    if (acceleration > THRESH_ACC_SMALL) {

        float mult = get_multiplier(acceleration,
                                    THRESH_ACC_SMALL,
                                    THRESH_ACC_NORMAL,
                                    THRESH_ACC_HIGH);

        bool is_high_penalty = (mult >= MULT_HIGH);
        int64_t elapsed      = now_us - s_state.last_acc_us;
        bool in_cooldown     = (elapsed < COOLDOWN_US);

        if (!s_state.hard_acc_active) {
            if (!in_cooldown || is_high_penalty) {
                penalty += PENALTY_ACC_BASE * mult;
                s_state.last_acc_us = now_us;
            }
            s_state.hard_acc_active = true;
        }
    } else {
        s_state.hard_acc_active = false;
    }

    /* ══════════════════════════════════════════════
       HIGH RPM
       ══════════════════════════════════════════════ */
    if (obd->rpm > THRESH_HIGH_RPM) {

        int64_t elapsed  = now_us - s_state.last_rpm_us;
        bool in_cooldown = (elapsed < COOLDOWN_US);

        if (!s_state.high_rpm_active) {
            if (!in_cooldown) {
                penalty += PENALTY_HIGH_RPM;
                s_state.last_rpm_us = now_us;
            }
            s_state.high_rpm_active = true;
        }
    } else {
        s_state.high_rpm_active = false;
    }


    if (cold_engine && penalty > 0.0f) {
        penalty *= COLD_ENGINE_MULT;
    }

    /* ══════════════════════════════════════════════
       Score update
       ══════════════════════════════════════════════ */
    if (penalty > 0.0f) {
        s_state.last_penalty_us      = now_us;
        s_state.repair_timer_started = true;
        s_state.score               -= penalty;
    } else {
        
        if (s_state.repair_timer_started &&
            (now_us - s_state.last_penalty_us) >= REPAIR_INTERVAL_US)
        {
            s_state.score           += 0.9f;
            s_state.last_penalty_us  = now_us; 
        }
         
    }

    
    if (s_state.score < 0.0f)  s_state.score = 0.0f;

    
    if (s_state.score < s_session_min_score)
        s_session_min_score = s_state.score;

    
    float cap = get_score_cap();
    if (s_state.score > cap) s_state.score = cap;

    obd->driving_score = s_state.score;
}