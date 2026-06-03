/**
 * @file mu60x.c
 * @brief MU60x UART driver implementation. See mu60x.h for usage.
 */
#include "mu60x.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "mu60x";

#define HEAD                    0xA0
#define UART_RX_BUF_BYTES       2048
#define DEFAULT_CMD_TIMEOUT_MS  500
#define READWRITE_TIMEOUT_MS    1500

/* Toggle TX/RX hex dumps. Useful while bringing things up; turn off for prod. */
#ifndef MU60X_TRACE_UART
#define MU60X_TRACE_UART        0
#endif

static const char *status_name(uint8_t status)
{
    switch (status) {
        case 0x10: return "operation successful";
        case 0x11: return "operation failed";
        case 0x22: return "antenna not connected";
        case 0x32: return "tag read error";
        case 0x33: return "tag write error";
        case 0x36: return "inoperable tag";
        case 0x37: return "inventory ok but access failed";
        case 0x40: return "access tag error or password error";
        case 0x41: return "invalid parameter";
        case 0x43: return "word count exceeds limit";
        case 0x58: return "two-way authentication failed";
        case 0x59: return "two-way authentication successful";
        case 0x60: return "insufficient tag power";
        case 0x61: return "insufficient tag permissions";
        case 0x62: return "memory address out of range";
        case 0x63: return "memory locked";
        case 0x64: return "incorrect operation password";
        case 0x65: return "tag authentication reader failed";
        case 0x66: return "unknown tag error";
        default:   return "unknown status";
    }
}

/* ------------------------------------------------------------------ */
/* LRC + UART helpers                                                 */
/* ------------------------------------------------------------------ */

static uint8_t lrc(const uint8_t *buf, size_t len)
{
    uint32_t s = 0;
    for (size_t i = 0; i < len; i++) s += buf[i];
    return (uint8_t)((0x100 - (s & 0xFF)) & 0xFF);
}

#if MU60X_TRACE_UART
static void hexdump(const char *prefix, const uint8_t *buf, size_t len)
{
    char line[MU60X_FRAME_MAX * 3 + 1];
    size_t pos = 0;
    for (size_t i = 0; i < len && pos + 3 < sizeof(line); i++) {
        pos += sprintf(&line[pos], "%02X ", buf[i]);
    }
    line[pos] = '\0';
    printf("%s [%u] %s\n", prefix, (unsigned)len, line);
}
#else
#define hexdump(p, b, n) ((void)0)
#endif

static int uart_tx(mu60x_t *dev, const uint8_t *buf, size_t len)
{
    int n = uart_write_bytes(dev->uart_num, (const char *)buf, len);
    hexdump("MU60x TX ->", buf, n > 0 ? (size_t)n : 0);
    return n;
}

static int uart_rx(mu60x_t *dev, uint8_t *buf, size_t len, TickType_t ticks)
{
    int n = uart_read_bytes(dev->uart_num, buf, len, ticks);
    if (n > 0) hexdump("MU60x RX <-", buf, (size_t)n);
    return n;
}

/* ------------------------------------------------------------------ */
/* Build/send a frame, read the response frame, validate LRC          */
/* ------------------------------------------------------------------ */

static esp_err_t send_frame(mu60x_t *dev, uint8_t cmd,
                            const uint8_t *data, uint8_t data_len)
{
    if (data_len > MU60X_FRAME_MAX - 4) return ESP_ERR_INVALID_ARG;
    uint8_t buf[MU60X_FRAME_MAX + 1];
    size_t i = 0;
    buf[i++] = HEAD;
    buf[i++] = 3 + data_len;
    buf[i++] = dev->address;
    buf[i++] = cmd;
    if (data_len) { memcpy(&buf[i], data, data_len); i += data_len; }
    buf[i] = lrc(buf, i);
    i++;
    uart_flush_input(dev->uart_num);
    int written = uart_tx(dev, buf, i);
    return (written == (int)i) ? ESP_OK : ESP_FAIL;
}

/**
 * Read one A0-framed packet from the UART. Resyncs on the next 0xA0 if junk
 * comes first. Returns ESP_OK and fills @p frame/@p flen on success.
 */
static esp_err_t recv_frame(mu60x_t *dev, uint32_t timeout_ms,
                            uint8_t *frame, size_t cap, size_t *flen)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    /* Find header */
    while (1) {
        TickType_t remaining = deadline - xTaskGetTickCount();
        if ((int32_t)remaining <= 0) return ESP_ERR_TIMEOUT;
        uint8_t b;
        int n = uart_rx(dev, &b, 1, remaining);
        if (n != 1) return ESP_ERR_TIMEOUT;
        if (b == HEAD) { frame[0] = HEAD; break; }
        /* otherwise resync */
    }
    /* Len byte */
    TickType_t remaining = deadline - xTaskGetTickCount();
    if ((int32_t)remaining <= 0) return ESP_ERR_TIMEOUT;
    uint8_t ln;
    if (uart_rx(dev, &ln, 1, remaining) != 1) return ESP_ERR_TIMEOUT;
    if (ln < 3 || (size_t)(ln + 2) > cap) return ESP_ERR_INVALID_RESPONSE;
    frame[1] = ln;
    /* Body + LRC = ln bytes after Len */
    remaining = deadline - xTaskGetTickCount();
    if ((int32_t)remaining <= 0) return ESP_ERR_TIMEOUT;
    int n = uart_rx(dev, &frame[2], ln, remaining);
    if (n != ln) return ESP_ERR_TIMEOUT;
    size_t total = 2 + ln;
    if (lrc(frame, total - 1) != frame[total - 1]) {
        ESP_LOGW(TAG, "LRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    *flen = total;
    return ESP_OK;
}

/**
 * Send a command and wait for the reader's reply with the same cmd code.
 * Most commands answer with one data byte; some answer with a longer payload.
 */
static esp_err_t transact(mu60x_t *dev, uint8_t cmd,
                          const uint8_t *data, uint8_t data_len,
                          uint32_t timeout_ms,
                          uint8_t *resp, size_t resp_cap, size_t *resp_len)
{
    if (!dev || !dev->initialised) return ESP_ERR_INVALID_STATE;
    if (dev->inventory_running) return ESP_ERR_INVALID_STATE;
    ESP_RETURN_ON_ERROR(send_frame(dev, cmd, data, data_len), TAG, "tx");

    /* The reader may emit unrelated frames before ours (rare but harmless);
     * keep reading until we see one for our cmd or timeout. */
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (1) {
        uint32_t left = pdTICKS_TO_MS(deadline - xTaskGetTickCount());
        if ((int32_t)left <= 0) return ESP_ERR_TIMEOUT;
        esp_err_t err = recv_frame(dev, left, resp, resp_cap, resp_len);
        if (err != ESP_OK) return err;
        if (resp[3] == cmd) return ESP_OK;
        ESP_LOGD(TAG, "ignoring unsolicited cmd 0x%02X while expecting 0x%02X", resp[3], cmd);
    }
}

/* ------------------------------------------------------------------ */
/* Public API: lifecycle / config                                     */
/* ------------------------------------------------------------------ */

esp_err_t mu60x_init(mu60x_t *dev, const mu60x_config_t *cfg)
{
    if (!dev || !cfg) return ESP_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(*dev));
    dev->uart_num = cfg->uart_num;
    dev->rst_gpio = cfg->rst_gpio;
    dev->address  = cfg->address;

    uart_config_t uc = {
        .baud_rate  = cfg->baud_rate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(cfg->uart_num, UART_RX_BUF_BYTES,
                                             0, 0, NULL, 0), TAG, "uart install");
    ESP_RETURN_ON_ERROR(uart_param_config(cfg->uart_num, &uc), TAG, "uart cfg");
    ESP_RETURN_ON_ERROR(uart_set_pin(cfg->uart_num, cfg->tx_gpio, cfg->rx_gpio,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart pins");

    if (cfg->rst_gpio >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << cfg->rst_gpio,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "rst gpio");
        /* We don't pulse it — polarity is undocumented for this module. */
        gpio_set_level(cfg->rst_gpio, 1);
    }
    dev->initialised = true;
    vTaskDelay(pdMS_TO_TICKS(100));  /* let the reader settle */

    /* If a previous ESP run crashed mid-inventory, the reader is still
     * streaming. Send a stop and drain whatever's in the UART so the next
     * transact() doesn't latch onto a stale streaming frame. */
    (void)send_frame(dev, 0x8C, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_flush_input(dev->uart_num);

    /* Sanity-ping the reader so init() fails loudly if comms are wrong. */
    mu60x_version_t ver;
    esp_err_t err = mu60x_get_version(dev, &ver);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no response from reader at %d bps (check wiring/baud)", cfg->baud_rate);
        uart_driver_delete(cfg->uart_num);
        dev->initialised = false;
        dev->inventory_running = false;
        return err;
    }
    ESP_LOGI(TAG, "reader v%u.%u model=0x%02X on UART%d", ver.major, ver.minor,
             ver.model, cfg->uart_num);

    /* Apply boot-time configuration only if it differs from the current state —
     * avoids wearing the reader's flash on every cold boot. */
    bool dirty = false;
    if (cfg->region != MU60X_REGION_KEEP) {
        mu60x_region_t cur;
        if (mu60x_get_region(dev, &cur) == ESP_OK && cur != cfg->region) {
            ESP_RETURN_ON_ERROR(mu60x_set_region(dev, cfg->region), TAG, "set region");
            ESP_LOGI(TAG, "region updated");
            dirty = true;
        }
    }
    if (cfg->power_dbm > 0 && cfg->power_dbm <= 33) {
        uint8_t cur;
        if (mu60x_get_power(dev, &cur) == ESP_OK && cur != cfg->power_dbm) {
            ESP_RETURN_ON_ERROR(mu60x_set_power(dev, cfg->power_dbm), TAG, "set power");
            ESP_LOGI(TAG, "power -> %u dBm", cfg->power_dbm);
            dirty = true;
        }
    }
    if (dirty && cfg->persist_settings) {
        if (mu60x_save_params(dev) == ESP_OK) {
            ESP_LOGI(TAG, "settings persisted to reader flash");
        }
    }
    return ESP_OK;
}

esp_err_t mu60x_get_version(mu60x_t *dev, mu60x_version_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    uint8_t resp[16]; size_t rlen = 0;
    ESP_RETURN_ON_ERROR(transact(dev, 0x72, NULL, 0, DEFAULT_CMD_TIMEOUT_MS,
                                  resp, sizeof(resp), &rlen),
                        TAG, "get_version");
    if (rlen < 4 + 3 + 1) return ESP_ERR_INVALID_RESPONSE;
    out->major = resp[4];
    out->minor = resp[5];
    out->model = resp[6];
    return ESP_OK;
}

static esp_err_t simple_cmd(mu60x_t *dev, uint8_t cmd,
                            const uint8_t *data, uint8_t dlen, uint32_t to_ms)
{
    uint8_t resp[16]; size_t rlen = 0;
    ESP_RETURN_ON_ERROR(transact(dev, cmd, data, dlen, to_ms, resp, sizeof(resp), &rlen),
                        TAG, "simple_cmd");
    /* Most config commands respond with a single status byte: 0x10 = success. */
    if (rlen >= 6 && resp[4] != 0x10) {
        ESP_LOGW(TAG, "cmd 0x%02X failed, status=0x%02X (%s)",
                 cmd, resp[4], status_name(resp[4]));
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t mu60x_set_power(mu60x_t *dev, uint8_t dbm)
{
    if (dbm > 33) return ESP_ERR_INVALID_ARG;
    return simple_cmd(dev, 0x76, &dbm, 1, DEFAULT_CMD_TIMEOUT_MS);
}

esp_err_t mu60x_get_power(mu60x_t *dev, uint8_t *dbm)
{
    if (!dbm) return ESP_ERR_INVALID_ARG;
    uint8_t resp[16]; size_t rlen = 0;
    ESP_RETURN_ON_ERROR(transact(dev, 0x77, NULL, 0, DEFAULT_CMD_TIMEOUT_MS,
                                  resp, sizeof(resp), &rlen),
                        TAG, "get_power");
    if (rlen < 6) return ESP_ERR_INVALID_RESPONSE;
    *dbm = resp[4];
    return ESP_OK;
}

static const uint8_t *region_bytes(mu60x_region_t r)
{
    static const uint8_t fcc[3]  = { 0x01, 0x07, 0x3B };
    static const uint8_t etsi[3] = { 0x02, 0x00, 0x06 };
    static const uint8_t chn1[3] = { 0x03, 0x00, 0x06 };
    static const uint8_t chn2[3] = { 0x05, 0x2B, 0x35 };
    switch (r) {
        case MU60X_REGION_FCC:  return fcc;
        case MU60X_REGION_ETSI: return etsi;
        case MU60X_REGION_CHN1: return chn1;
        case MU60X_REGION_CHN2: return chn2;
        default: return NULL;
    }
}

esp_err_t mu60x_set_region(mu60x_t *dev, mu60x_region_t region)
{
    const uint8_t *b = region_bytes(region);
    if (!b) return ESP_ERR_INVALID_ARG;
    return simple_cmd(dev, 0x78, b, 3, DEFAULT_CMD_TIMEOUT_MS);
}

esp_err_t mu60x_get_region(mu60x_t *dev, mu60x_region_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    uint8_t resp[16]; size_t rlen = 0;
    ESP_RETURN_ON_ERROR(transact(dev, 0x79, NULL, 0, DEFAULT_CMD_TIMEOUT_MS,
                                  resp, sizeof(resp), &rlen),
                        TAG, "get_region");
    if (rlen < 8) return ESP_ERR_INVALID_RESPONSE;
    switch (resp[4]) {
        case 0x01: *out = MU60X_REGION_FCC;  break;
        case 0x02: *out = MU60X_REGION_ETSI; break;
        case 0x03: *out = MU60X_REGION_CHN1; break;
        case 0x05: *out = MU60X_REGION_CHN2; break;
        default:   *out = MU60X_REGION_KEEP; break;
    }
    return ESP_OK;
}

esp_err_t mu60x_save_params(mu60x_t *dev)
{
    return simple_cmd(dev, 0x4A, NULL, 0, DEFAULT_CMD_TIMEOUT_MS);
}

esp_err_t mu60x_reset(mu60x_t *dev)
{
    return simple_cmd(dev, 0x70, NULL, 0, 2000);
}

/* ------------------------------------------------------------------ */
/* Inventory                                                          */
/* ------------------------------------------------------------------ */

esp_err_t mu60x_start_inventory(mu60x_t *dev, uint8_t ant)
{
    if (!dev || !dev->initialised) return ESP_ERR_INVALID_STATE;
    if (dev->inventory_running) return ESP_OK;

    /* Status frame "10 = started" comes back first; subsequent frames are tag data
     * or "36 = no tag". We treat the start command separately because while
     * inventory is running we'll be reading streamed frames, not a reply.
     *
     * If the reader is already running inventory (e.g. our previous boot
     * crashed mid-stream), the first attempt returns 0x11 ("command failed").
     * Retry once after issuing a stop: that recovers cleanly. */
    for (int attempt = 0; attempt < 2; attempt++) {
        uart_flush_input(dev->uart_num);
        ESP_RETURN_ON_ERROR(send_frame(dev, 0x89, &ant, 1), TAG, "start_inv tx");
        uint8_t resp[16]; size_t rlen;
        esp_err_t err = recv_frame(dev, DEFAULT_CMD_TIMEOUT_MS, resp, sizeof(resp), &rlen);
        if (err != ESP_OK) return err;
        if (rlen >= 6 && resp[3] == 0x89 && resp[4] == 0x10) {
            dev->inventory_running = true;
            return ESP_OK;
        }
        ESP_LOGW(TAG, "inventory start status=0x%02X (%s) (attempt %d)",
                 resp[4], status_name(resp[4]), attempt + 1);
        if (attempt == 0) {
            (void)send_frame(dev, 0x8C, NULL, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            uart_flush_input(dev->uart_num);
        }
    }
    return ESP_ERR_INVALID_RESPONSE;
}

esp_err_t mu60x_stop_inventory(mu60x_t *dev)
{
    if (!dev || !dev->initialised) return ESP_ERR_INVALID_STATE;
    if (!dev->inventory_running) return ESP_OK;
    /* Send stop and drain. Streamed tag frames may still arrive after the
     * stop reply, so we discard whatever we read for ~100ms then return. */
    ESP_RETURN_ON_ERROR(send_frame(dev, 0x8C, NULL, 0), TAG, "stop tx");
    TickType_t until = xTaskGetTickCount() + pdMS_TO_TICKS(150);
    uint8_t junk[MU60X_FRAME_MAX + 2];
    size_t jlen;
    while ((int32_t)(until - xTaskGetTickCount()) > 0) {
        if (recv_frame(dev, 50, junk, sizeof(junk), &jlen) != ESP_OK) break;
    }
    dev->inventory_running = false;
    return ESP_OK;
}

/* Decode a streaming 0x89 frame into a mu60x_tag_t. Returns true if it's a
 * tag-data frame, false for status frames (which the caller will skip). */
static bool parse_inventory_frame(const uint8_t *f, size_t flen, mu60x_tag_t *out)
{
    /* Tag-data frames are at least: A0 Len 00 89 Ant PC(2) EPC(>=1) RSSI(4) Freq(3) LRC
       i.e. flen >= 4 + 1 + 2 + 1 + 4 + 3 + 1 = 16. Status frames are 6 bytes. */
    if (flen < 16) return false;
    if (f[3] != 0x89) return false;
    /* data field is f[4 .. flen-2]; length = flen - 6 */
    size_t dlen = flen - 6;
    /* ant(1) pc(2) epc(var) rssi(4) freq(3) -- the var length is dlen-10. */
    if (dlen < 10) return false;
    size_t epc_len = dlen - 10;
    if (epc_len == 0 || epc_len > MU60X_EPC_MAX_BYTES) return false;
    out->ant = f[4];
    out->pc[0] = f[5];
    out->pc[1] = f[6];
    memcpy(out->epc, &f[7], epc_len);
    out->epc_len = (uint8_t)epc_len;
    memcpy(out->rssi_raw, &f[7 + epc_len], 4);
    /* Convert raw RSSI to a coarse signed dBm. The manual's formula combines
     * the I/Q magnitude; we approximate -(rssi_raw[1]/2). Good enough for a
     * comparable "stronger/weaker" reading per tag. */
    out->rssi_dbm = -(int)out->rssi_raw[1] / 2 - 30;
    const uint8_t *fr = &f[7 + epc_len + 4];
    out->freq_khz = ((uint32_t)fr[0] << 16) | ((uint32_t)fr[1] << 8) | fr[2];
    return true;
}

esp_err_t mu60x_poll(mu60x_t *dev, uint32_t timeout_ms, mu60x_tag_t *out)
{
    if (!dev || !dev->initialised || !out) return ESP_ERR_INVALID_ARG;
    if (!dev->inventory_running) return ESP_ERR_INVALID_STATE;

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    uint8_t frame[MU60X_FRAME_MAX + 2];
    size_t flen;
    while ((int32_t)(deadline - xTaskGetTickCount()) > 0) {
        uint32_t left = pdTICKS_TO_MS(deadline - xTaskGetTickCount());
        if (left == 0) left = 1;
        esp_err_t err = recv_frame(dev, left, frame, sizeof(frame), &flen);
        if (err == ESP_ERR_TIMEOUT) return ESP_ERR_TIMEOUT;
        if (err != ESP_OK) continue;             /* swallow bad LRC etc. */
        if (parse_inventory_frame(frame, flen, out)) return ESP_OK;
        /* otherwise it was a "no tag" status frame — keep polling */
    }
    return ESP_ERR_TIMEOUT;
}

/* ------------------------------------------------------------------ */
/* Tag memory read / write                                            */
/* ------------------------------------------------------------------ */

static void copy_pwd(uint8_t dst[4], const uint8_t pwd[4])
{
    if (pwd) memcpy(dst, pwd, 4);
    else     memset(dst, 0, 4);
}

esp_err_t mu60x_read_tag(mu60x_t *dev, mu60x_bank_t bank,
                         uint16_t word_addr, uint16_t word_count,
                         const uint8_t pwd[4], uint8_t *out)
{
    if (!out || word_count == 0) return ESP_ERR_INVALID_ARG;
    /* Data: MemBank(1) | WordAdd(4 BE) | WordCnt(2 BE) | Pwd(4) */
    uint8_t data[11];
    data[0] = (uint8_t)bank;
    data[1] = 0; data[2] = 0;
    data[3] = (word_addr >> 8) & 0xFF;
    data[4] = word_addr & 0xFF;
    data[5] = (word_count >> 8) & 0xFF;
    data[6] = word_count & 0xFF;
    copy_pwd(&data[7], pwd);

    uint8_t resp[MU60X_FRAME_MAX + 2];
    size_t rlen = 0;
    ESP_RETURN_ON_ERROR(transact(dev, 0x81, data, sizeof(data),
                                  READWRITE_TIMEOUT_MS, resp, sizeof(resp), &rlen),
                        TAG, "read tx");

    /* Two reply shapes:
     *   Failure: Len=4, data[0] = error code (e.g. 0x36, 0x40, 0x43)
     *   Success: Len > 4, contains TagCount(2) DataLen(1) ReadData(...) ReadLen(2) AntID(1) ReadCount(1)
     *            ReadData = PC(2)+EPC(var)+CRC(2) + actual read words (word_count*2 bytes at the tail of ReadData).
     */
    if (rlen == 6) {
        ESP_LOGW(TAG, "read failed, status=0x%02X (%s)",
                 resp[4], status_name(resp[4]));
        return ESP_ERR_INVALID_RESPONSE;
    }
    /* The last 4 bytes of the data field are: ReadLen(2) | AntID(1) | ReadCount(1).
     * Before those, the last (word_count*2) bytes are the actual data we asked for. */
    size_t data_field = rlen - 6;   /* bytes between cmd and LRC */
    size_t want = (size_t)word_count * 2;
    if (data_field < 4 + want) return ESP_ERR_INVALID_RESPONSE;
    /* tail layout: ... <wanted bytes> | ReadLen(2) ReadLen | AntID | ReadCount */
    const uint8_t *tail = &resp[4 + data_field - 4 - want];
    memcpy(out, tail, want);
    return ESP_OK;
}

esp_err_t mu60x_write_tag(mu60x_t *dev, mu60x_bank_t bank,
                          uint16_t word_addr, const uint8_t *data_in,
                          uint16_t word_count, const uint8_t pwd[4])
{
    if (!data_in || word_count == 0) return ESP_ERR_INVALID_ARG;
    size_t payload_bytes = (size_t)word_count * 2;
    if (11 + payload_bytes > MU60X_FRAME_MAX - 3) return ESP_ERR_INVALID_SIZE;

    /* Data: Pwd(4) | MemBank(1) | WordAdd(4 BE) | WordCnt(2 BE) | WordData(N) */
    uint8_t buf[MU60X_FRAME_MAX];
    size_t i = 0;
    copy_pwd(&buf[i], pwd); i += 4;
    buf[i++] = (uint8_t)bank;
    buf[i++] = 0; buf[i++] = 0;
    buf[i++] = (word_addr >> 8) & 0xFF;
    buf[i++] = word_addr & 0xFF;
    buf[i++] = (word_count >> 8) & 0xFF;
    buf[i++] = word_count & 0xFF;
    memcpy(&buf[i], data_in, payload_bytes); i += payload_bytes;

    uint8_t resp[MU60X_FRAME_MAX + 2];
    size_t rlen = 0;
    ESP_RETURN_ON_ERROR(transact(dev, 0x82, buf, (uint8_t)i,
                                  READWRITE_TIMEOUT_MS, resp, sizeof(resp), &rlen),
                        TAG, "write tx");
    /* Failure: Len=4, data[0] = error code.
     * Success: long frame ending with ... | 0x10 | retry_count. */
    if (rlen == 6) {
        if (resp[4] == 0x10) return ESP_OK;
        ESP_LOGW(TAG, "write failed, status=0x%02X (%s)",
                 resp[4], status_name(resp[4]));
        return ESP_ERR_INVALID_RESPONSE;
    }
    /* second-to-last byte before LRC is the result. */
    if (rlen < 8) return ESP_ERR_INVALID_RESPONSE;
    uint8_t result = resp[rlen - 3];
    if (result != 0x10) {
        ESP_LOGW(TAG, "write failed, result=0x%02X (%s)",
                 result, status_name(result));
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}
