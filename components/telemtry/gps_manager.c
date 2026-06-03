#include "gps_manager.h"
#include "commons.h" // Dostęp do kolejki i struktur
#include "obd_manager.h"
#include <string.h>
#include <sys/time.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "time_manager.h"
#include <math.h>

#include <time.h>
static const char *TAG_GPS = "GPS_MANAGER";

#define GPS_UART_NUM UART_NUM_1
#define GPS_TX_PIN 43
#define GPS_RX_PIN 44
#define GPS_BUF_SIZE 1024
#define NMEA_LINE_MAX_LEN 128

/* =====================================================
 * EKF tuning parameters
 * ===================================================== */

/*
 * These parameters define the uncertainty model of the EKF:
 *
 * SIGMA_GPS     - GPS position noise (lower = trust GPS more)
 * SIGMA_V       - velocity uncertainty
 * SIGMA_PSI     - heading measurement noise
 * SIGMA_PSIDOT  - yaw rate uncertainty
 * SIGMA_A       - acceleration process noise
 * SIGMA_DDPSI   - yaw acceleration noise
 *
 * Higher values = more responsive filter but noisier output
 * Lower values  = smoother trajectory but slower response
 */

//PARAMETRY FILTRU
#define SIGMA_GPS 4.0f 
#define SIGMA_V 1.0f 
#define SIGMA_PSI 0.7f 
#define SIGMA_PSIDOT 0.3f 
#define SIGMA_A 2.0f 
#define SIGMA_DDPSI 0.2f 
#define V_MIN 3.5 
#define EPS_PSIDOT 0.001f 

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


typedef struct {
    double lat;
    double lon;
    double course;
    double speed;
    double hdop;
    int h, m, s;
    int day, month, year;
    bool valid;
} GPS_Internal_t;
static GPS_Internal_t gps_data;


//Matrixes utilites


/* Matrix multiplication */
void mat_mul(float *A, float *B, float *C, int n, int m, int p) {
    for (int i=0;i<n;i++) {
        for (int j=0;j<p;j++){
            C[i*p+j] = 0;
            for (int k=0;k<m;k++) {
                C[i*p+j] +=A[i*m+k] * B[k*p+j];
            }
        }
    }
}
/* Matrix transpose */
void mat_transpose(float *A, float *AT,int n, int m) {
    for (int i=0;i<n;i++) {
        for (int j=0;j<m;j++){
            AT[j*n+i] = A[i*m+j];
        }
    }
}
/* Matrix addition */
void mat_add(float *A, float *B, float *C, int n, int m) {
    for (int i=0;i<n;i++){
        for (int j=0;j<m;j++){
            C[i*m+j] = A[i*m+j] + B[i*m+j];
        }
    }
}
/* Matrix subtraction */
void mat_sub(float *A, float *B, float *C, int n, int m) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            C[i*m+j] = A[i*m+j] - B[i*m+j];
        }
    }
}

bool mat_inv3(float A[3][3], float Inv[3][3]) {
    float det =
        A[0][0] * (A[1][1]*A[2][2] - A[1][2]*A[2][1])
       -A[0][1] * (A[1][0]*A[2][2] - A[1][2]*A[2][0])
       +A[0][2] * (A[1][0]*A[2][1] - A[1][1]*A[2][0]);

    if (fabsf(det) < 1e-10f) return false;  // macierz osobliwa

    float id = 1.0f / det;

    Inv[0][0] =  (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * id;
    Inv[0][1] = -(A[0][1]*A[2][2] - A[0][2]*A[2][1]) * id;
    Inv[0][2] =  (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * id;

    Inv[1][0] = -(A[1][0]*A[2][2] - A[1][2]*A[2][0]) * id;
    Inv[1][1] =  (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * id;
    Inv[1][2] = -(A[0][0]*A[1][2] - A[0][2]*A[1][0]) * id;

    Inv[2][0] =  (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * id;
    Inv[2][1] = -(A[0][0]*A[2][1] - A[0][1]*A[2][0]) * id;
    Inv[2][2] =  (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * id;

    return true;
}
bool mat_inv2(float A[2][2],float Inv[2][2]){
    float det = (A[0][0]*A[1][1])-(A[0][1]*A[1][0]);
    if (fabsf(det) < 1e-9f) return false;

    float id = 1.0f/det;
    Inv[0][0] = A[1][1]*id;
    Inv[0][1] = -A[0][1]*id;
    Inv[1][0] = -A[1][0]*id;
    Inv[1][1] = A[0][0]*id;


    return true;

}
    

//FILTR KALMAN EXTENTED
static float x_est[5];
static float P[5][5];
static double ref_lat;
static double ref_lon;
static bool   ekf_initialized = false;
static TickType_t last_tick = 0;
//funkcja pomocnicza do konwertowania 
static double nmea_to_decimal(double nmea)
{
    int deg = (int)(nmea / 100);
    double min = nmea - (deg * 100);
    return deg + (min / 60.0);
}

static void latlon_to_enu(double lat, double lon, double lat0, double lon0, double *x, double *y) {
    *y = (lat - lat0) * 111320.0;
    *x = (lon - lon0) * cos(lat0 * M_PI / 180.0) * 111320.0;
}

static void enu_to_latlon(double x, double y, double lat0, double lon0, double *lat, double *lon) {
    *lat = lat0 + y / 111320.0;
    *lon = lon0 + x / (cos(lat0 * M_PI / 180.0) * 111320.0);
}

/* Initialize EKF state */
void kalman_init(double x_GPS0, double y_GPS0,float speed_OBD0,double course_GPS0) {
    x_est[0] = (float)x_GPS0;
    x_est[1] = (float)y_GPS0;
    x_est[2] = speed_OBD0;
    x_est[3] = (float)(course_GPS0*(M_PI/180));
    x_est[4] = 0.0f;
    
     /* Initial uncertainty */
    memset(P,0,sizeof(P));
    P[0][0] = SIGMA_GPS * SIGMA_GPS;
    P[1][1] = SIGMA_GPS * SIGMA_GPS;
    P[2][2] = SIGMA_V * SIGMA_V;
    P[3][3] = SIGMA_PSI * SIGMA_PSI;
    P[4][4] = SIGMA_PSIDOT * SIGMA_PSIDOT;
    ekf_initialized = true;
}
void kalman_predict(float dt, float v_obd) {
    if (v_obd < 1.4f) {
        x_est[2] = 0.0f;
        x_est[4] = 0.0f;

        P[0][0] += 0.001f;
        P[1][1] += 0.001f;

        return;
    }
    float x = x_est[0];
    float y = x_est[1];
    float v = v_obd;
    float psi = x_est[3];
    float psid = x_est[4];

    float x_new,y_new;


    float psi_new = psi + psid*dt;
    while (psi_new >  (float)M_PI) psi_new -= 2.0f * (float)M_PI;
    while (psi_new < -(float)M_PI) psi_new += 2.0f * (float)M_PI;
    //stan skrecenie
    if (fabsf(psid) >= EPS_PSIDOT) {
        x_new = x + (v_obd/psid)*(sinf(psi_new)-sinf(psi));
        y_new = y + (v_obd/psid)*(-cosf(psi_new)+cosf(psi));

    }else {
        x_new = x + v_obd*cosf(psi)*dt;
        y_new = y + v_obd*sinf(psi)*dt;
    }
    
    // P=Fk*P*Fk^T+Qk
    float F[5][5];
    float Q[5][5];
    memset(F,0,sizeof(F));
    memset(Q,0,sizeof(Q));
    //jakobiany
    F[0][0] = 1.0f;
    F[1][1] = 1.0f;
    F[2][2] = 1.0f;
    F[3][3] = 1.0f;
    F[3][4] = dt;
    F[4][4] = 1.0f;
    if (fabsf(psid) >= EPS_PSIDOT) {
        F[0][2] = (sinf(psi_new)-sinf(psi))/psid;
        F[0][3] = (v/psid) * (cosf(psi_new) - cosf(psi));
        F[0][4] = (v*dt*cosf(psi_new))/psid - v*(sinf(psi_new)-sinf(psi))/(psid*psid);

        F[1][2] = (-cosf(psi_new) + cosf(psi)) / psid;
        F[1][3] = (v/psid) * (sinf(psi_new) - sinf(psi));
        F[1][4] = (v*dt*sinf(psi_new))/psid - v*(-cosf(psi_new)+cosf(psi))/(psid*psid);
    }else{
        F[0][2] = (cosf(psi)*dt);
        F[0][3] = -v * sinf(psi)*dt;
        F[1][2] = sinf(psi)*dt;
        F[1][3] = v * cosf(psi) * dt;
       }
    // matrix Q (STATE)
    float dt2 = dt*dt;
    float dt3 = dt2*dt;
    float dt4 = dt3*dt;
    float sa2 = SIGMA_A     * SIGMA_A; //acceleration
    float sd2 = SIGMA_DDPSI * SIGMA_DDPSI; //turning

    Q[0][0] = sa2 * dt4/4.0f;
    Q[0][2] = sa2 * dt3/2.0f;
    Q[1][1] = sa2 * dt4/4.0f;
    Q[2][0] = sa2 * dt3/2.0f;
    Q[2][2] = sa2 * dt2;
    Q[3][3] = sd2 * dt4/4.0f;
    Q[3][4] = sd2 * dt3/2.0f;
    Q[4][3] = sd2 * dt3/2.0f;
    Q[4][4] = sd2 * dt2;

    // P =F*P*F^T + Q
    float FP[5][5],FT[5][5],FPFT[5][5],P_new[5][5];

    mat_mul(&F[0][0],&P[0][0],&FP[0][0],5,5,5);
    mat_transpose (&F[0][0],&FT[0][0],5,5);
    mat_mul(&FP[0][0], &FT[0][0], &FPFT[0][0], 5, 5, 5);
    mat_add(&FPFT[0][0], &Q[0][0], &P_new[0][0], 5, 5);
    memcpy(P,P_new,sizeof(P));

    x_est[0] = x_new;
    x_est[1] = y_new;
    x_est[2] = v_obd;
    x_est[3] = psi_new;
    x_est[4] = psid;
}
void kalman_update(double lat, double lon, float psi_gps, float hdop, float v_obd) {
    double x_gps, y_gps;
    latlon_to_enu(lat,lon,ref_lat,ref_lon,&x_gps,&y_gps);



    float R[3][3];
    memset(R,0,sizeof(R));
    R[0][0] = SIGMA_GPS*SIGMA_GPS*hdop*hdop;
    R[1][1] = SIGMA_GPS*SIGMA_GPS*hdop*hdop;
    R[2][2] = (SIGMA_GPS/v_obd)*(SIGMA_GPS/v_obd+0.1f);
    if (v_obd>=V_MIN) {
        float z[3];
        z[0] = x_gps;
        z[1] = y_gps;
        z[2] = psi_gps;

        float hx[3];
        hx[0] = x_est[0];
        hx[1] = x_est[1];
        hx[2] = x_est[3];
        //innowacja
        float inn[3];
        inn[0] = z[0] - hx[0];
        inn[1] = z[1] - hx[1];
        inn[2] = z[2] - hx[2];
        while (inn[2] >  (float)M_PI) inn[2] -= 2.0f * (float)M_PI;
        while (inn[2] < -(float)M_PI) inn[2] += 2.0f * (float)M_PI;

        
        float H[3][5] = {
            {1, 0, 0, 0, 0},   
            {0, 1, 0, 0, 0},   
            {0, 0, 0, 1, 0},   
        };
        // S = H*P*H^T + R  (3x3)
        float HT[5][3], HP[3][5], HPHT[3][3], S[3][3], Sinv[3][3];
        mat_mul(&H[0][0], &P[0][0], &HP[0][0],   3, 5, 5);
        mat_transpose (&H[0][0], &HT[0][0], 3, 5);
        mat_mul(&HP[0][0], &HT[0][0], &HPHT[0][0], 3, 5, 3);
        mat_add(&HPHT[0][0], &R[0][0], &S[0][0], 3, 3);

        if (!mat_inv3(S, Sinv)) return;  // macierz osobliwa - pomijamy krok

        // K = P*H^T*S^-1  (5x3)
        float PHT[5][3], K[5][3];
        mat_mul(&P[0][0],   &HT[0][0],   &PHT[0][0], 5, 5, 3);
        mat_mul(&PHT[0][0], &Sinv[0][0], &K[0][0],   5, 3, 3);

        // x = x + K*inn
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 3; j++)
                x_est[i] += K[i][j] * inn[j];

        // P = (I - K*H)*P
        float KH[5][5], I5[5][5], IKH[5][5], P_new[5][5];
        memset(I5, 0, sizeof(I5));
        for (int i = 0; i < 5; i++) I5[i][i] = 1.0f;
        mat_mul(&K[0][0],   &H[0][0],  &KH[0][0],   5, 3, 5);
        mat_sub(&I5[0][0],  &KH[0][0], &IKH[0][0],  5, 5);
        mat_mul(&IKH[0][0], &P[0][0],  &P_new[0][0], 5, 5, 5);
        memcpy(P, P_new, sizeof(P));
        }else {
            float z[2];
            z[0] = x_gps;
            z[1] = y_gps;
        
            float hx[2];
            hx[0] = x_est[0];
            hx[1] = x_est[1];
            float inn[2];
            inn[0] = z[0] - hx[0];
            inn[1] = z[1] - hx[1];

            float H[2][5] = {
                {1, 0, 0, 0, 0},   
                {0, 1, 0, 0, 0},   
            };
            // S = H*P*H^T + R  (2x2)
            float HT[5][2], HP[2][5], HPHT[2][2], S[2][2], Sinv[2][2];
            mat_mul(&H[0][0], &P[0][0], &HP[0][0],   2, 5, 5);
            mat_transpose (&H[0][0], &HT[0][0], 2, 5);
            mat_mul(&HP[0][0], &HT[0][0], &HPHT[0][0], 2, 5, 2);

            float R2[2][2] = { {R[0][0], 0}, {0, R[1][1]} };
            mat_add(&HPHT[0][0], &R2[0][0], &S[0][0], 2, 2);

            if (!mat_inv2(S, Sinv)) return;

            // K = P*H^T*S^-1  (5x2)
            float PHT[5][2], K[5][2];
            mat_mul(&P[0][0],   &HT[0][0],   &PHT[0][0], 5, 5, 2);
            mat_mul(&PHT[0][0], &Sinv[0][0], &K[0][0],   5, 2, 2);

            // x = x + K*inn
            for (int i = 0; i < 5; i++)
                for (int j = 0; j < 2; j++)
                    x_est[i] += K[i][j] * inn[j];

            // P = (I - K*H)*P
            float KH[5][5], I5[5][5], IKH[5][5], P_new[5][5];
            memset(I5, 0, sizeof(I5));
            for (int i = 0; i < 5; i++) I5[i][i] = 1.0f;
            mat_mul(&K[0][0],   &H[0][0],  &KH[0][0],   5, 2, 5);
            mat_sub(&I5[0][0],  &KH[0][0], &IKH[0][0],  5, 5);
            mat_mul(&IKH[0][0], &P[0][0],  &P_new[0][0], 5, 5, 5);
            memcpy(P, P_new, sizeof(P));
        }
    

}


bool parse_rmc(char* line) {

    char *data_start = line + 7;
    double time_nmea, lat_nmea, lon_nmea, speed,course;
    int date_nmea;
    char status, lat_dir, lon_dir;

    int found = sscanf(data_start, "%lf,%c,%lf,%c,%lf,%c,%lf,%lf,%d",
                &time_nmea, &status, &lat_nmea, &lat_dir, &lon_nmea, &lon_dir, &speed, &course, &date_nmea
    );
    if (found >= 9 && status == 'A') {
        double lat = nmea_to_decimal(lat_nmea);
        double lon = nmea_to_decimal(lon_nmea);

        if (lat_dir == 'S') lat *= -1.0;
        if (lon_dir == 'W') lon *= -1.0;

        gps_data.lat = lat;
        gps_data.lon = lon;

        gps_data.speed = speed;
        gps_data.course = course;

        int t = (int) time_nmea;
        gps_data.h = t / 10000;
        gps_data.m = (t / 100) % 100;
        gps_data.s = t % 100;


        gps_data.day   = date_nmea / 10000;
        gps_data.month = (date_nmea / 100) % 100;
        gps_data.year  = date_nmea % 100;

        gps_data.valid = true;
        return true;
    }
    return false;
}

bool parse_gga(char *line) {

    char *data_start = line + 7;

    double time;
    double lat, lon;
    char lat_dir, lon_dir;
    int fix, sats;
    double hdop;

    int found = sscanf(data_start,
        "%lf,%lf,%c,%lf,%c,%d,%d,%lf",
        &time,
        &lat, &lat_dir,
        &lon, &lon_dir,
        &fix,
        &sats,
        &hdop
    );
    if (found >= 8 && fix > 0) {
        gps_data.hdop = hdop;
        return true;
    }

    return false;
}



/* Initialize GPS UART driver */
void init_gps_uart(){

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS, //8 bit 
        .parity = UART_PARITY_DISABLE, 
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //queue buffer
    const int uart_buffer_size = 1024;
    //drivers
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1,uart_buffer_size,0,0,NULL,0));
    //parameters
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    //pins
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1,GPS_TX_PIN,GPS_RX_PIN,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
}

void gps_task(void *pvParameters) {
    uint8_t data[GPS_BUF_SIZE];
    char line_buffer[NMEA_LINE_MAX_LEN];
    int line_idx = 0;
    bool rmc_ready = false, gga_ready = false;

    ESP_LOGI(TAG_GPS, "GPS task started");
    uart_flush_input(GPS_UART_NUM);
    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, data, GPS_BUF_SIZE - 1, pdMS_TO_TICKS(100));

        if (len > 0) {
            for (int i = 0; i < len; i++) {
                uint8_t byte = data[i];
                
                if (byte == '\n') {
                    line_buffer[line_idx] = '\0';
                    

                        if (strncmp(line_buffer, "$GNRMC", 6) == 0 ||
                        strncmp(line_buffer, "$GPRMC", 6) == 0) {
                            if (parse_rmc(line_buffer)) rmc_ready = true;
                        }
                        //GGA
                        else if (strncmp(line_buffer, "$GNGGA", 6) == 0 ||
                                strncmp(line_buffer, "$GPGGA", 6) == 0) {
                            if (parse_gga(line_buffer)) gga_ready = true;
                        }
                        if (rmc_ready && gga_ready && gps_data.valid) {
                            float v_obd = obd_get_speed() / 3.6f;  
                            float psi_gps  = (float)(gps_data.course * M_PI / 180.0);

                            //dt
                            TickType_t now = xTaskGetTickCount();
                            float dt = (last_tick == 0) ? 1.0f : (float)(now - last_tick) / configTICK_RATE_HZ;
                            if (dt <= 0.0f || dt > 2.0f) dt = 1.0f;
                            last_tick = now;
                            if (!ekf_initialized) {
                            double x0, y0;
                            ref_lat = gps_data.lat;
                            ref_lon = gps_data.lon;
                            x0 = 0.0;
                            y0 = 0.0;
                            kalman_init(x0, y0, v_obd, gps_data.course);
                            } else {
                                kalman_predict(dt, v_obd);

                                kalman_update(gps_data.lat, gps_data.lon,
                                            psi_gps, (float)gps_data.hdop, v_obd);
                                double ekf_lat, ekf_lon;
                                enu_to_latlon(x_est[0], x_est[1], ref_lat, ref_lon, &ekf_lat, &ekf_lon);
                            
                                time_manager_set_from_gps(gps_data.day, gps_data.month, gps_data.year, gps_data.h, gps_data.m, gps_data.s);

                                TelemetryPacket_t packet;
                                packet.type = DATA_TYPE_GPS;
                                packet.data.gps.lat = (uint32_t)round(ekf_lat * 10000000.0);
                                packet.data.gps.lon = (uint32_t)round(ekf_lon * 10000000.0);
                                packet.timestamp = (uint32_t)time(NULL);

                                //xQueueSend(telemetryQueue, &packet, portMAX_DELAY);

                                if (xQueueSend(telemetryQueue, &packet, 0) == pdPASS) {
                                }
                                else {
                                continue;
                                }
                            }
                            rmc_ready = false;
                            gga_ready = false;
                        }
                        line_idx = 0;
                        
                    } else {
                    
                            if (byte != '\r' && line_idx < NMEA_LINE_MAX_LEN - 1) {
                                line_buffer[line_idx++] = (char)byte;
                            } else if (line_idx >= NMEA_LINE_MAX_LEN - 1) {
                                line_idx = 0;
                            }
                        }
                    }
                }
            }
}