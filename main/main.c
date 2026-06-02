/**
 * @file main.c
 * @brief Demo for the mu60x component on ESP32-S3.
 *
 * Wiring (matches your bench setup):
 *   RFID TX  -> ESP32-S3 GPIO15  (ESP RX)
 *   RFID RX  <- ESP32-S3 GPIO16  (ESP TX)
 *   RFID VCC -> 3V3
 *   RFID GND -> GND
 *   RFID RST -> ESP32-S3 GPIO7
 *
 * Flow:
 *   1. Init UART1 at 115200, ping the reader for firmware version.
 *   2. Set ETSI band + 20 dBm and persist to reader flash if anything changed.
 *   3. Start real-time inventory on antenna 1; print every detected EPC.
 *   4. Read or write a vehicle door number in USER memory for each new tag.
 *   5. Publish device ID, EPC, and optional vehicle door number over BLE.
 */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "bleBeacon.h"
#include "mu60x.h"

static const char *TAG = "main";

#define DEVICE_ID                  "RFID_001"

/*
 * Vehicle door number storage:
 * - Stored in RFID USER bank at word 0 as fixed 10-byte ASCII, zero padded.
 * - Only 0-9, A-Z, and a-z are allowed.
 * - Present one tag at a time while reading/writing USER memory.
 *
 * To write a tag:
 *   1. Set VEHICLE_DOOR_MODE to VEHICLE_DOOR_MODE_WRITE.
 *   2. Set VEHICLE_DOOR_NUMBER to the required value.
 *   3. Build/flash and present the tag until the log confirms the write.
 *   4. Set VEHICLE_DOOR_MODE back to VEHICLE_DOOR_MODE_READ for normal use.
 */
#define VEHICLE_DOOR_MODE_READ     0
#define VEHICLE_DOOR_MODE_WRITE    1
#define VEHICLE_DOOR_MODE          VEHICLE_DOOR_MODE_READ
#define VEHICLE_DOOR_NUMBER        "DOOR0001"
#define VEHICLE_DOOR_MAX_LEN       10
#define VEHICLE_DOOR_WORD_ADDR     0
#define VEHICLE_DOOR_WORD_COUNT    5
#define VEHICLE_DOOR_BYTES         (VEHICLE_DOOR_WORD_COUNT * 2)

#if VEHICLE_DOOR_MODE != VEHICLE_DOOR_MODE_READ && VEHICLE_DOOR_MODE != VEHICLE_DOOR_MODE_WRITE
#error "VEHICLE_DOOR_MODE must be VEHICLE_DOOR_MODE_READ or VEHICLE_DOOR_MODE_WRITE"
#endif

#if VEHICLE_DOOR_BYTES != VEHICLE_DOOR_MAX_LEN
#error "Vehicle door storage must stay exactly 10 bytes"
#endif

static mu60x_t s_dev;

static void log_hex(const char *label, const uint8_t *b, size_t n)
{
    char buf[3 * 64 + 1];
    size_t p = 0;
    for (size_t i = 0; i < n && p + 3 < sizeof(buf); i++) {
        p += sprintf(&buf[p], "%02X ", b[i]);
    }
    buf[p] = '\0';
    ESP_LOGI(TAG, "%s = %s", label, buf);
}

static bool is_vehicle_door_char(char c)
{
    return (c >= '0' && c <= '9') ||
           (c >= 'A' && c <= 'Z') ||
           (c >= 'a' && c <= 'z');
}

static esp_err_t encode_vehicle_door_number(const char *value,
                                            uint8_t out[VEHICLE_DOOR_BYTES])
{
    if (value == NULL || value[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    size_t len = strlen(value);
    if (len > VEHICLE_DOOR_MAX_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    memset(out, 0, VEHICLE_DOOR_BYTES);
    for (size_t i = 0; i < len; i++) {
        if (!is_vehicle_door_char(value[i])) {
            return ESP_ERR_INVALID_ARG;
        }
        out[i] = (uint8_t)value[i];
    }
    return ESP_OK;
}

static esp_err_t decode_vehicle_door_number(const uint8_t raw[VEHICLE_DOOR_BYTES],
                                            char out[VEHICLE_DOOR_MAX_LEN + 1])
{
    size_t len = 0;
    for (size_t i = 0; i < VEHICLE_DOOR_BYTES; i++) {
        if (raw[i] == 0 || raw[i] == 0xFF) {
            break;
        }
        if (!is_vehicle_door_char((char)raw[i])) {
            ESP_LOGW(TAG, "vehicle door data has invalid byte 0x%02X at offset %u",
                     raw[i], (unsigned)i);
            return ESP_ERR_INVALID_RESPONSE;
        }
        out[len++] = (char)raw[i];
    }

    out[len] = '\0';
    return (len > 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

static esp_err_t read_vehicle_door_number(char out[VEHICLE_DOOR_MAX_LEN + 1])
{
    uint8_t raw[VEHICLE_DOOR_BYTES] = {0};
    esp_err_t err = mu60x_read_tag(&s_dev, MU60X_BANK_USER,
                                   VEHICLE_DOOR_WORD_ADDR,
                                   VEHICLE_DOOR_WORD_COUNT,
                                   NULL, raw);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "vehicle door USER read failed: %s", esp_err_to_name(err));
        return err;
    }

    log_hex("  USER", raw, sizeof(raw));
    err = decode_vehicle_door_number(raw, out);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "vehicle door number = %s", out);
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "vehicle door number is empty/unwritten");
    }
    return err;
}

static esp_err_t write_vehicle_door_number(char out[VEHICLE_DOOR_MAX_LEN + 1])
{
    uint8_t raw[VEHICLE_DOOR_BYTES] = {0};
    ESP_RETURN_ON_ERROR(encode_vehicle_door_number(VEHICLE_DOOR_NUMBER, raw),
                        TAG, "vehicle door encode");

    ESP_LOGI(TAG, "writing vehicle door number = %s", VEHICLE_DOOR_NUMBER);
    esp_err_t err = mu60x_write_tag(&s_dev, MU60X_BANK_USER,
                                    VEHICLE_DOOR_WORD_ADDR,
                                    raw,
                                    VEHICLE_DOOR_WORD_COUNT,
                                    NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "vehicle door USER write failed: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    err = read_vehicle_door_number(out);
    if (err == ESP_OK && strcmp(out, VEHICLE_DOOR_NUMBER) == 0) {
        ESP_LOGI(TAG, "vehicle door write verified");
        return ESP_OK;
    }

    ESP_LOGW(TAG, "vehicle door write verification failed");
    return (err == ESP_OK) ? ESP_ERR_INVALID_RESPONSE : err;
}

static esp_err_t sync_vehicle_door_number(char out[VEHICLE_DOOR_MAX_LEN + 1])
{
    if (VEHICLE_DOOR_MODE == VEHICLE_DOOR_MODE_WRITE) {
        return write_vehicle_door_number(out);
    }
    return read_vehicle_door_number(out);
}

static void rfid_task(void *arg)
{
    mu60x_config_t cfg = {
        .uart_num         = UART_NUM_1,
        .tx_gpio          = 16,            /* ESP TX -> RFID RX */
        .rx_gpio          = 15,            /* ESP RX <- RFID TX */
        .rst_gpio         = 7,             /* RFID RST */
        .address          = 0x00,
        .baud_rate        = 115200,
        .region           = MU60X_REGION_ETSI,
        .power_dbm        = 20,
        .persist_settings = true,
    };
    ESP_ERROR_CHECK(mu60x_init(&s_dev, &cfg));

    ESP_ERROR_CHECK(mu60x_start_inventory(&s_dev, 1));
    ESP_LOGI(TAG, "inventory running; show a tag to the antenna...");

    bool tid_demo_done = false;
    bool cached_epc_known = false;
    uint8_t cached_epc[MU60X_EPC_MAX_BYTES] = {0};
    uint8_t cached_epc_len = 0;
    char cached_vehicle_door[VEHICLE_DOOR_MAX_LEN + 1] = {0};

    while (1) {
        mu60x_tag_t tag;
        esp_err_t err = mu60x_poll(&s_dev, 500, &tag);
        if (err == ESP_ERR_TIMEOUT) continue;
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "poll error: %s", esp_err_to_name(err));
            continue;
        }

        ESP_LOGI(TAG, "ant=%u rssi~%d dBm freq=%" PRIu32 " kHz",
                 tag.ant, tag.rssi_dbm, tag.freq_khz);
        log_hex("  EPC", tag.epc, tag.epc_len);

        bool same_cached_epc = cached_epc_known &&
                               cached_epc_len == tag.epc_len &&
                               memcmp(cached_epc, tag.epc, tag.epc_len) == 0;

        if (!same_cached_epc) {
            mu60x_stop_inventory(&s_dev);

            if (!tid_demo_done) {
                tid_demo_done = true;
                uint8_t tid[12] = {0};
                if (mu60x_read_tag(&s_dev, MU60X_BANK_TID, 0, 6, NULL, tid) == ESP_OK) {
                    log_hex("  TID", tid, sizeof(tid));
                } else {
                    ESP_LOGW(TAG, "TID read failed");
                }
            }

            cached_vehicle_door[0] = '\0';
            esp_err_t user_err = sync_vehicle_door_number(cached_vehicle_door);
            if (user_err != ESP_OK && user_err != ESP_ERR_NOT_FOUND) {
                ESP_LOGW(TAG, "vehicle door sync failed; BLE will publish EPC only");
            }

            memcpy(cached_epc, tag.epc, tag.epc_len);
            cached_epc_len = tag.epc_len;
            cached_epc_known = true;

            mu60x_start_inventory(&s_dev, 1);
        }

        const char *vehicle_door = cached_vehicle_door[0] != '\0' ?
                                   cached_vehicle_door : NULL;
        esp_err_t ble_err = ble_beacon_publish_tag_data(DEVICE_ID, tag.epc,
                                                        tag.epc_len,
                                                        vehicle_door);
        if (ble_err != ESP_OK && ble_err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "BLE publish failed: %s", esp_err_to_name(ble_err));
        }
    }
}

void app_main(void)
{
    ble_beacon_init();

    /* 6 KB stack: the polling task carries one tag struct + protocol buffers. */
    xTaskCreate(rfid_task, "rfid", 6144, NULL, 5, NULL);
}
