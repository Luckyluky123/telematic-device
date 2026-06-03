/**
 * @file test_parse_obd_main.c
 * @brief Testy jednostkowe parsowania danych OBD-II
 *
 * Używa PRAWDZIWYCH funkcji z obd_manager.c (komponent telemetria).
 * Żeby to działało:
 *   1. W obd_manager.c usuń "static" przed parse_obd_data
 *   2. W obd_manager.h dodaj deklarację (gotowe w modyfikacje_src/)
 */

#include <string.h>
#include <math.h>

#include "unity.h"
#include "obd_manager.h"   /* ← parse_obd_data */
#include "commons.h"       /* ← PaczkaDanych_t */

/* Pomocnik: pusta paczka OBD z wyzerowanymi polami */
static PaczkaDanych_t make_empty_obd_packet(void) {
    PaczkaDanych_t p;
    memset(&p, 0, sizeof(p));
    p.typ = TYP_OBD;
    return p;
}

/* ============================================================
 * PID 0x0C — RPM
 * Wzór: rpm = ((A*256) + B) / 4
 * ============================================================ */

void test_obd_rpm_zero(void) {
    PaczkaDanych_t p = make_empty_obd_packet();
    parse_obd_data(0x0C, 0, 0, &p);
    TEST_ASSERT_EQUAL_UINT16(0, p.dane.obd.rpm);
}

void test_obd_rpm_800(void) {
    /*  800 rpm: (A*256+B)/4 = 800 → A*256+B = 3200 → A=12, B=128 */
    PaczkaDanych_t p = make_empty_obd_packet();
    parse_obd_data(0x0C, 12, 128, &p);
    TEST_ASSERT_EQUAL_UINT16(800, p.dane.obd.rpm);
}

void test_obd_rpm_max(void) {
    /* A=255, B=255 → (255*256+255)/4 = 16383 */
    PaczkaDanych_t p = make_empty_obd_packet();
    parse_obd_data(0x0C, 255, 255, &p);
    TEST_ASSERT_EQUAL_UINT16(16383, p.dane.obd.rpm);
}


/* ============================================================
 * PID 0x01 — DTC count + MIL (lampka kontrolna)
 * Wzór: dtc_count = A & 0x7F,  mil_status = (A & 0x80) >> 7
 * ============================================================ */

void test_obd_dtc_no_errors_mil_off(void) {
    /* A=0x00 → dtc=0, mil=0 */
    PaczkaDanych_t p = make_empty_obd_packet();
    parse_obd_data(0x01, 0x00, 0, &p);
    TEST_ASSERT_EQUAL_UINT8(0, p.dane.obd.dtc_count);
    TEST_ASSERT_EQUAL_UINT8(0, p.dane.obd.mil_status);
}

void test_obd_dtc_3_errors_mil_on(void) {
    /* A=0x83 → bit7=1 (MIL on), bity 0-6 = 3 błędy */
    PaczkaDanych_t p = make_empty_obd_packet();
    parse_obd_data(0x01, 0x83, 0, &p);
    TEST_ASSERT_EQUAL_UINT8(3, p.dane.obd.dtc_count);
    TEST_ASSERT_EQUAL_UINT8(1, p.dane.obd.mil_status);
}

void test_obd_dtc_max_codes_mil_off(void) {
    /* A=0x7F → bit7=0 (MIL off), 127 błędów */
    PaczkaDanych_t p = make_empty_obd_packet();
    parse_obd_data(0x01, 0x7F, 0, &p);
    TEST_ASSERT_EQUAL_UINT8(127, p.dane.obd.dtc_count);
    TEST_ASSERT_EQUAL_UINT8(0,   p.dane.obd.mil_status);
}

/* ============================================================
 * PID nieznany 
 * ============================================================ */

void test_obd_unknown_pid_no_change(void) {
    PaczkaDanych_t p = make_empty_obd_packet();
    p.dane.obd.speed = 99;
    parse_obd_data(0xFF, 1, 2, &p);   /* 0xFF nie istnieje w switch */
    TEST_ASSERT_EQUAL_UINT16(99, p.dane.obd.speed);
}


/* ============================================================
 * ENTRY POINT
 * ============================================================ */
void app_main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_obd_rpm_zero);
    RUN_TEST(test_obd_rpm_800);
    RUN_TEST(test_obd_rpm_max);

   
    RUN_TEST(test_obd_dtc_no_errors_mil_off);
    RUN_TEST(test_obd_dtc_3_errors_mil_on);
    RUN_TEST(test_obd_dtc_max_codes_mil_off);

    RUN_TEST(test_obd_unknown_pid_no_change);


    UNITY_END();
}