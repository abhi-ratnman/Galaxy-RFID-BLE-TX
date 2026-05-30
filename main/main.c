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
 *   4. After the first tag is seen, demonstrate read_tag() of TID.
 *   5. (Optional) demonstrate write_tag() round-trip on the EPC bank.
 */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bleBeacon.h"
#include "mu60x.h"

static const char *TAG = "main";

/* Set to 1 to do a write-then-restore round-trip on the first tag's EPC. */
#define DEMO_EPC_WRITE_ROUNDTRIP    0
#define DEVICE_ID                   "RFID_001"

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
    bool write_demo_done = false;
    uint8_t saved_epc[12];
    uint8_t saved_epc_len = 0;

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
        esp_err_t ble_err = ble_beacon_publish_tag(DEVICE_ID, tag.epc, tag.epc_len);
        if (ble_err != ESP_OK && ble_err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "BLE publish failed: %s", esp_err_to_name(ble_err));
        }

        /* --- one-shot: stop inventory, read TID, restart inventory --- */
        if (!tid_demo_done) {
            tid_demo_done = true;
            memcpy(saved_epc, tag.epc, tag.epc_len);
            saved_epc_len = tag.epc_len;

            mu60x_stop_inventory(&s_dev);
            uint8_t tid[12] = {0};
            if (mu60x_read_tag(&s_dev, MU60X_BANK_TID, 0, 6, NULL, tid) == ESP_OK) {
                log_hex("  TID", tid, sizeof(tid));
            } else {
                ESP_LOGW(TAG, "TID read failed");
            }

#if DEMO_EPC_WRITE_ROUNDTRIP
            if (saved_epc_len == 12 && !write_demo_done) {
                const uint8_t test_epc[12] = {
                    0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
                    0x77, 0x88, 0xAA, 0xBB, 0xCC, 0xDD,
                };
                if (mu60x_write_tag(&s_dev, MU60X_BANK_EPC, 2, test_epc, 6, NULL) == ESP_OK) {
                    ESP_LOGI(TAG, "wrote test EPC; restoring original...");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    if (mu60x_write_tag(&s_dev, MU60X_BANK_EPC, 2, saved_epc, 6, NULL) == ESP_OK) {
                        ESP_LOGI(TAG, "EPC restored");
                        write_demo_done = true;
                    } else {
                        ESP_LOGE(TAG, "FAILED to restore EPC! Tag now reports test EPC.");
                    }
                } else {
                    ESP_LOGW(TAG, "EPC write failed (tag may have moved out of field)");
                }
            }
#else
            (void)write_demo_done;
            (void)saved_epc; (void)saved_epc_len;
#endif

            mu60x_start_inventory(&s_dev, 1);
        }
    }
}

void app_main(void)
{
    ble_beacon_init();

    /* 6 KB stack: the polling task carries one tag struct + protocol buffers. */
    xTaskCreate(rfid_task, "rfid", 6144, NULL, 5, NULL);
}
