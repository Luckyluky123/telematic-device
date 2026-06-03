\
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "unity.h"
#include "esp_log.h"

#include "commons.h"
#include "flash_storage.h"

static const char *TAG = "TEST_FLASH";

/* =========================
 * Pomocnicze tworzenie paczek
 * ========================= */

static PaczkaDanych_t make_obd_packet(uint16_t rpm, uint16_t speed, int16_t temp)
{
    PaczkaDanych_t p;
    memset(&p, 0, sizeof(p));
    p.typ = TYP_OBD;
    p.dane.obd.rpm = rpm;
    p.dane.obd.speed = speed;
    p.dane.obd.coolant_temp = temp;
    return p;
}

static PaczkaDanych_t make_gps_packet(double lat, double lon)
{
    PaczkaDanych_t p;
    memset(&p, 0, sizeof(p));
    p.typ = TYP_GPS;
    p.dane.gps.lat = (int32_t)(lat * 10000000.0);
    p.dane.gps.lon = (int32_t)(lon * 10000000.0);
    return p;
}

/* =========================

 * ========================= */

static void flash_test_setup(void)
{
    init_flash_storage();
    storage_reset_read_and_clear();
}

/* =========================
 * TEST 1 — init
 * ========================= */

void test_flash_init_ok(void)
{
    esp_err_t ret = init_flash_storage();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

/* =========================
 * TEST 2 — empty state
 * ========================= */

void test_storage_empty_after_reset(void)
{
    flash_test_setup();
    TEST_ASSERT_TRUE(storage_is_empty());
}

/* =========================
 * TEST 3 — save packet
 * ========================= */

void test_save_single_packet_no_flush(void)
{
    flash_test_setup();

    PaczkaDanych_t p = make_obd_packet(1500, 60, 90);
    TEST_ASSERT_EQUAL(ESP_OK, storage_save_packet(&p));
}

/* =========================
 * TEST 4 — auto flush
 * ========================= */

void test_flush_after_buffer_full(void)
{
    flash_test_setup();

    for (int i = 0; i < RAM_BUFFER_SIZE; i++) {
        PaczkaDanych_t p = make_obd_packet(1000 + i, i * 2, 80);
        storage_save_packet(&p);
    }

    TEST_ASSERT_FALSE(storage_is_empty());
}

/* =========================
 * TEST 5 — read packet
 * ========================= */

void test_get_next_packet_after_flush(void)
{
    flash_test_setup();

    for (int i = 0; i < RAM_BUFFER_SIZE; i++) {
        PaczkaDanych_t p = make_obd_packet(2000 + i, i, 90 + i);
        storage_save_packet(&p);
    }

    PaczkaDanych_t out;
    TEST_ASSERT_TRUE(storage_get_next_packet(&out));
}

/* =========================
 * TEST 6 — sequential read
 * ========================= */

void test_sequential_read_all_packets(void)
{
    flash_test_setup();

    for (int i = 0; i < RAM_BUFFER_SIZE; i++) {
        PaczkaDanych_t p = make_obd_packet(3000 + i, i, 70);
        storage_save_packet(&p);
    }

    PaczkaDanych_t out;
    int count = 0;

    while (storage_get_next_packet(&out)) {
        count++;
        if (count > RAM_BUFFER_SIZE + 5) break;
    }

    TEST_ASSERT_EQUAL_INT(RAM_BUFFER_SIZE, count);
}

/* =========================
 * TEST 7 — GPS roundtrip
 * ========================= */

void test_gps_packet_survives_flash_roundtrip(void)
{
    flash_test_setup();

    PaczkaDanych_t gps = make_gps_packet(52.1667, 21.0000);

    for (int i = 0; i < RAM_BUFFER_SIZE - 1; i++) {
        PaczkaDanych_t p = make_obd_packet(100, 0, 20);
        storage_save_packet(&p);
    }

    storage_save_packet(&gps);

    PaczkaDanych_t out;
    bool found = false;

    while (storage_get_next_packet(&out)) {
        if (out.typ == TYP_GPS) {
            found = true;
            break;
        }
    }

    TEST_ASSERT_TRUE(found);
}

/* =========================
 * TEST 8 — reset
 * ========================= */

void test_reset_clears_all_data(void)
{
    flash_test_setup();

    for (int i = 0; i < RAM_BUFFER_SIZE; i++) {
        PaczkaDanych_t p = make_obd_packet(100, 0, 20);
        storage_save_packet(&p);
    }

    storage_reset_read_and_clear();
    TEST_ASSERT_TRUE(storage_is_empty());
}

/* =========================
 * TEST 9 — manual flush
 * ========================= */

void test_manual_flush_partial_buffer(void)
{
    flash_test_setup();

    for (int i = 0; i < 3; i++) {
        PaczkaDanych_t p = make_obd_packet(500 + i, 30, 85);
        storage_save_packet(&p);
    }

    TEST_ASSERT_EQUAL(ESP_OK, storage_flush_to_flash());
}

/* =========================
 * TEST 10 — EOF
 * ========================= */

void test_get_next_returns_false_at_eof(void)
{
    flash_test_setup();

    PaczkaDanych_t p = make_obd_packet(999, 0, 50);
    storage_save_packet(&p);
    storage_flush_to_flash();

    PaczkaDanych_t out;

    TEST_ASSERT_TRUE(storage_get_next_packet(&out));
    TEST_ASSERT_FALSE(storage_get_next_packet(&out));
}



void app_main(void)
{
    init_flash_storage();

    UNITY_BEGIN();

    RUN_TEST(test_flash_init_ok);
    RUN_TEST(test_storage_empty_after_reset);
    RUN_TEST(test_save_single_packet_no_flush);
    RUN_TEST(test_flush_after_buffer_full);
    RUN_TEST(test_get_next_packet_after_flush);
    RUN_TEST(test_sequential_read_all_packets);
    RUN_TEST(test_gps_packet_survives_flash_roundtrip);
    RUN_TEST(test_reset_clears_all_data);
    RUN_TEST(test_manual_flush_partial_buffer);
    RUN_TEST(test_get_next_returns_false_at_eof);

    UNITY_END();
}