/**
 * @file mu60x.h
 * @brief Driver for MU60x-series UHF RFID reader modules.
 *
 * Implements the protocol from "MU60x Host Interface Packet Definitions v1.2.1".
 *
 * Frame layout: 0xA0 | Len | Addr | Cmd | Data[] | LRC
 *   LRC = (0x100 - sum(0xA0..last_data)) & 0xFF
 *
 * Polling-style usage:
 *
 *   mu60x_t dev;
 *   mu60x_init(&dev, &(mu60x_config_t){
 *       .uart_num=UART_NUM_1, .tx_gpio=16, .rx_gpio=15, .rst_gpio=-1,
 *       .address=0x00, .baud_rate=115200,
 *       .region=MU60X_REGION_ETSI, .power_dbm=20, .persist_settings=true,
 *   });
 *   mu60x_start_inventory(&dev, 1);
 *   mu60x_tag_t tag;
 *   while (1) {
 *       if (mu60x_poll(&dev, 500, &tag) == ESP_OK) {
 *           // tag.epc, tag.epc_len, tag.rssi_dbm, tag.freq_khz ...
 *       }
 *   }
 *   mu60x_stop_inventory(&dev);
 *
 * Read/write require inventory to be stopped first.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Operating frequency region (manual section 3.33 / 5). */
typedef enum {
    MU60X_REGION_KEEP = 0,   /* don't change whatever the reader is set to */
    MU60X_REGION_FCC,        /* 902.00 – 928.00 MHz */
    MU60X_REGION_ETSI,       /* 865.00 – 868.00 MHz  (India / EU) */
    MU60X_REGION_CHN1,       /* 840.00 – 845.00 MHz */
    MU60X_REGION_CHN2,       /* 920.00 – 925.00 MHz */
} mu60x_region_t;

/* Tag memory banks (cmd 0x81 / 0x82). */
typedef enum {
    MU60X_BANK_RESERVED = 0, /* kill + access passwords */
    MU60X_BANK_EPC      = 1, /* CRC(w0) + PC(w1) + EPC(w2..) */
    MU60X_BANK_TID      = 2, /* read-only on most chips */
    MU60X_BANK_USER     = 3, /* not present on every chip (e.g. UCODE 9 has none) */
} mu60x_bank_t;

#define MU60X_EPC_MAX_BYTES   62
#define MU60X_FRAME_MAX       255   /* protocol Len is one byte */

/* One inventoried tag, decoded from a cmd 0x89 streaming frame. */
typedef struct {
    uint8_t  ant;
    uint8_t  pc[2];
    uint8_t  epc[MU60X_EPC_MAX_BYTES];
    uint8_t  epc_len;          /* bytes of EPC actually populated */
    uint8_t  rssi_raw[4];      /* raw 4 bytes (I/Q phase + magnitude) */
    int      rssi_dbm;         /* coarse signed dBm derived from rssi_raw */
    uint32_t freq_khz;         /* RF frequency this read happened at */
} mu60x_tag_t;

/* Reader firmware identity (cmd 0x72). */
typedef struct {
    uint8_t major;
    uint8_t minor;
    uint8_t model;   /* 0x01=MU601, 0x02=MU602, 0x06=MU606, 0x08=MU608 */
} mu60x_version_t;

/* Configuration passed to init. */
typedef struct {
    uart_port_t      uart_num;
    int              tx_gpio;
    int              rx_gpio;
    int              rst_gpio;   /* -1 if not wired */
    uint8_t          address;    /* reader address, default 0x00 */
    int              baud_rate;  /* 115200 unless you've changed it */

    /* Boot-time configuration. If region != KEEP, the driver will set it
     * during init. If persist_settings is true, the new region+power are
     * also saved to the reader's flash (cmd 0x4A) so subsequent boots
     * keep them. The save is skipped if the reader already matches. */
    mu60x_region_t   region;
    uint8_t          power_dbm;       /* 0..33; ignored if 0 */
    bool             persist_settings;
} mu60x_config_t;

/* Driver handle. Treat as opaque. */
typedef struct {
    uart_port_t uart_num;
    int         rst_gpio;
    uint8_t     address;
    bool        initialised;
    bool        inventory_running;
} mu60x_t;

/* ---- Lifecycle / configuration --------------------------------------- */

esp_err_t mu60x_init        (mu60x_t *dev, const mu60x_config_t *cfg);
esp_err_t mu60x_get_version (mu60x_t *dev, mu60x_version_t *out);
esp_err_t mu60x_set_power   (mu60x_t *dev, uint8_t dbm);          /* 0..33 */
esp_err_t mu60x_get_power   (mu60x_t *dev, uint8_t *dbm);
esp_err_t mu60x_set_region  (mu60x_t *dev, mu60x_region_t region);
esp_err_t mu60x_get_region  (mu60x_t *dev, mu60x_region_t *out);
esp_err_t mu60x_save_params (mu60x_t *dev);                       /* cmd 0x4A */
esp_err_t mu60x_reset       (mu60x_t *dev);                       /* cmd 0x70 */

/* ---- Inventory (streaming, polling style) ---------------------------- */

/**
 * Start real-time inventory (cmd 0x89). The reader will stream frames
 * (one per detected tag, plus periodic "no tag" status frames) until
 * mu60x_stop_inventory() is called.
 *
 * @param ant antenna number (1..4); for single-antenna products use 1.
 */
esp_err_t mu60x_start_inventory(mu60x_t *dev, uint8_t ant);

/**
 * Block up to @p timeout_ms waiting for the next *tag* frame, ignoring
 * "no tag" status frames internally.
 *
 * @return ESP_OK         out has a freshly decoded tag
 *         ESP_ERR_TIMEOUT no tag arrived within timeout_ms
 *         ESP_ERR_INVALID_STATE inventory isn't running
 */
esp_err_t mu60x_poll(mu60x_t *dev, uint32_t timeout_ms, mu60x_tag_t *out);

esp_err_t mu60x_stop_inventory(mu60x_t *dev);                     /* cmd 0x8C */

/* ---- Tag memory (only valid when inventory is stopped) --------------- */

/**
 * Read tag memory (cmd 0x81). Caller buffer @p out must hold @p word_count*2 bytes.
 * @param pwd   4-byte access password, NULL for all-zero (unprotected tags).
 */
esp_err_t mu60x_read_tag (mu60x_t *dev, mu60x_bank_t bank,
                          uint16_t word_addr, uint16_t word_count,
                          const uint8_t pwd[4], uint8_t *out);

/**
 * Write tag memory (cmd 0x82). @p data is @p word_count*2 bytes.
 * @param pwd   4-byte access password, NULL for all-zero.
 *
 * Writing to MU60X_BANK_EPC starting at word 2 changes the tag's identity.
 * Writing to MU60X_BANK_TID will fail on chips that lock it (most do).
 */
esp_err_t mu60x_write_tag(mu60x_t *dev, mu60x_bank_t bank,
                          uint16_t word_addr, const uint8_t *data,
                          uint16_t word_count, const uint8_t pwd[4]);

#ifdef __cplusplus
}
#endif
