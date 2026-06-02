#include "bleBeacon.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_check.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define TAG "BLE_BEACON"

#define BLE_BEACON_ENCRYPTION_ENABLED 0
#define BLE_BEACON_VERSION 1
#define BLE_COMPANY_ID_LSB 0xFF
#define BLE_COMPANY_ID_MSB 0xFF
#define BLE_AD_TYPE_COMPLETE_LOCAL_NAME 0x09
#define BLE_AD_TYPE_MANUFACTURER_DATA 0xFF
#define BLE_NONCE_LEN 13
#define BLE_CCM_TAG_LEN 4
#define BLE_EXT_ADV_INSTANCE 0
#define BLE_EXT_ADV_MAX_LEN 229
#define BLE_PLAINTEXT_MAX_LEN 160

#if BLE_BEACON_ENCRYPTION_ENABLED
#include "esp_random.h"
#include "mbedtls/ccm.h"

static const uint8_t s_ble_key[16] = {
    0x52, 0x46, 0x49, 0x44, 0x5F, 0x42, 0x4C, 0x45,
    0x5F, 0x4B, 0x45, 0x59, 0x5F, 0x30, 0x30, 0x31,
};
#endif

static bool s_params_ready;
static bool s_adv_started;

#if BLE_BEACON_ENCRYPTION_ENABLED
static uint64_t s_tx_counter;
static mbedtls_ccm_context s_ccm_ctx;

static uint32_t device_id_hash(const char *id)
{
    uint32_t h = 0x811c9dc5u;
    while (id != NULL && *id != '\0') {
        h ^= (uint8_t)(*id++);
        h *= 0x01000193u;
    }
    return h;
}

static void build_nonce(uint8_t nonce[BLE_NONCE_LEN], uint32_t hash, uint64_t counter)
{
    nonce[0] = 0;
    nonce[1] = (uint8_t)(hash >> 24);
    nonce[2] = (uint8_t)(hash >> 16);
    nonce[3] = (uint8_t)(hash >> 8);
    nonce[4] = (uint8_t)hash;
    nonce[5] = (uint8_t)(counter >> 56);
    nonce[6] = (uint8_t)(counter >> 48);
    nonce[7] = (uint8_t)(counter >> 40);
    nonce[8] = (uint8_t)(counter >> 32);
    nonce[9] = (uint8_t)(counter >> 24);
    nonce[10] = (uint8_t)(counter >> 16);
    nonce[11] = (uint8_t)(counter >> 8);
    nonce[12] = (uint8_t)counter;
}
#endif

static esp_err_t epc_to_hex(const uint8_t *epc, uint8_t epc_len,
                            char *out, size_t out_size)
{
    static const char hex[] = "0123456789ABCDEF";
    if (epc == NULL || out == NULL || epc_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (out_size < ((size_t)epc_len * 2u) + 1u) {
        return ESP_ERR_INVALID_SIZE;
    }

    for (uint8_t i = 0; i < epc_len; i++) {
        out[i * 2u] = hex[epc[i] >> 4];
        out[i * 2u + 1u] = hex[epc[i] & 0x0F];
    }
    out[(size_t)epc_len * 2u] = '\0';
    return ESP_OK;
}

static esp_err_t build_plaintext(const char *device_id,
                                 const uint8_t *epc,
                                 uint8_t epc_len,
                                 const char *vehicle_door,
                                 char plaintext[BLE_PLAINTEXT_MAX_LEN],
                                 size_t *plaintext_len)
{
    char epc_hex[125];
    ESP_RETURN_ON_ERROR(epc_to_hex(epc, epc_len, epc_hex, sizeof(epc_hex)),
                        TAG, "epc hex");

    int written;
    if (vehicle_door != NULL && vehicle_door[0] != '\0') {
        written = snprintf(plaintext, BLE_PLAINTEXT_MAX_LEN, "%s,%s,%s",
                           device_id, epc_hex, vehicle_door);
    } else {
        written = snprintf(plaintext, BLE_PLAINTEXT_MAX_LEN, "%s,%s",
                           device_id, epc_hex);
    }
    if (written <= 0 || written >= BLE_PLAINTEXT_MAX_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    *plaintext_len = (size_t)written;
    return ESP_OK;
}

static esp_err_t start_advertising(void)
{
    esp_ble_gap_ext_adv_t ext_adv = {
        .instance = BLE_EXT_ADV_INSTANCE,
        .duration = 0,
        .max_events = 0,
    };
    return esp_ble_gap_ext_adv_start(1, &ext_adv);
}

static esp_err_t append_ad_field(uint8_t *adv_data,
                                 size_t adv_cap,
                                 size_t *idx,
                                 uint8_t type,
                                 const uint8_t *payload,
                                 size_t payload_len)
{
    if (adv_data == NULL || idx == NULL || payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (payload_len + 1u > UINT8_MAX || *idx + payload_len + 2u > adv_cap) {
        return ESP_ERR_INVALID_SIZE;
    }

    adv_data[(*idx)++] = (uint8_t)(payload_len + 1u);
    adv_data[(*idx)++] = type;
    memcpy(&adv_data[*idx], payload, payload_len);
    *idx += payload_len;
    return ESP_OK;
}

static esp_err_t set_adv_data(const char *device_id,
                              const uint8_t *plaintext,
                              size_t plaintext_len)
{
    uint8_t adv_data[BLE_EXT_ADV_MAX_LEN] = {0};
    uint8_t mfg_data[BLE_EXT_ADV_MAX_LEN] = {0};
    size_t adv_idx = 0;
    size_t mfg_idx = 0;

    ESP_RETURN_ON_ERROR(append_ad_field(adv_data, sizeof(adv_data), &adv_idx,
                                        BLE_AD_TYPE_COMPLETE_LOCAL_NAME,
                                        (const uint8_t *)device_id,
                                        strlen(device_id)),
                        TAG, "adv name");

    mfg_data[mfg_idx++] = BLE_COMPANY_ID_LSB;
    mfg_data[mfg_idx++] = BLE_COMPANY_ID_MSB;
    mfg_data[mfg_idx++] = BLE_BEACON_VERSION;

#if BLE_BEACON_ENCRYPTION_ENABLED
    uint8_t nonce[BLE_NONCE_LEN];
    build_nonce(nonce, device_id_hash(device_id), s_tx_counter);

    if (mfg_idx + BLE_NONCE_LEN + plaintext_len + BLE_CCM_TAG_LEN > sizeof(mfg_data)) {
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(&mfg_data[mfg_idx], nonce, BLE_NONCE_LEN);
    mfg_idx += BLE_NONCE_LEN;

    uint8_t *ct_dst = &mfg_data[mfg_idx];
    uint8_t *tag_dst = ct_dst + plaintext_len;
    int rc = mbedtls_ccm_encrypt_and_tag(&s_ccm_ctx, plaintext_len,
                                         nonce, BLE_NONCE_LEN,
                                         NULL, 0,
                                         plaintext, ct_dst,
                                         tag_dst, BLE_CCM_TAG_LEN);
    if (rc != 0) {
        ESP_LOGE(TAG, "AES-CCM encrypt failed: -0x%04X", -rc);
        return ESP_FAIL;
    }

    mfg_idx += plaintext_len + BLE_CCM_TAG_LEN;
    s_tx_counter++;
#else
    if (mfg_idx + plaintext_len > sizeof(mfg_data)) {
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(&mfg_data[mfg_idx], plaintext, plaintext_len);
    mfg_idx += plaintext_len;
#endif

    ESP_RETURN_ON_ERROR(append_ad_field(adv_data, sizeof(adv_data), &adv_idx,
                                        BLE_AD_TYPE_MANUFACTURER_DATA,
                                        mfg_data, mfg_idx),
                        TAG, "adv manufacturer");

    return esp_ble_gap_config_ext_adv_data_raw(BLE_EXT_ADV_INSTANCE,
                                               (uint16_t)adv_idx,
                                               adv_data);
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        if (param->ext_adv_set_params.status == ESP_BT_STATUS_SUCCESS) {
            s_params_ready = true;
            ESP_LOGI(TAG, "extended advertising params ready");
        } else {
            ESP_LOGE(TAG, "set adv params failed, status=%d",
                     param->ext_adv_set_params.status);
        }
        break;

    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        if (param->ext_adv_data_set.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "adv data set failed, status=%d",
                     param->ext_adv_data_set.status);
            break;
        }
        if (!s_adv_started) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(start_advertising());
        }
        break;

    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
        if (param->ext_adv_start.status == ESP_BT_STATUS_SUCCESS) {
            s_adv_started = true;
            ESP_LOGI(TAG, "extended advertising started");
        } else {
            ESP_LOGE(TAG, "adv start failed, status=%d",
                     param->ext_adv_start.status);
        }
        break;

    default:
        break;
    }
}

static esp_err_t init_nvs_for_bt(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase");
        err = nvs_flash_init();
    }
    return err;
}

void ble_beacon_init(void)
{
    esp_err_t err = init_nvs_for_bt();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(err));
        return;
    }

#if BLE_BEACON_ENCRYPTION_ENABLED
    s_tx_counter = ((uint64_t)esp_random() << 32) | esp_random();

    mbedtls_ccm_init(&s_ccm_ctx);
    int rc = mbedtls_ccm_setkey(&s_ccm_ctx, MBEDTLS_CIPHER_ID_AES,
                                s_ble_key, sizeof(s_ble_key) * 8u);
    if (rc != 0) {
        ESP_LOGE(TAG, "AES-CCM key setup failed: -0x%04X", -rc);
        mbedtls_ccm_free(&s_ccm_ctx);
        return;
    }
#endif

    err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "classic BT mem release: %s", esp_err_to_name(err));
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "bt controller init failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "bt controller enable failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_bluedroid_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "bluedroid init failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "bluedroid enable failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_ble_gap_register_callback(esp_gap_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gap callback register failed: %s", esp_err_to_name(err));
        return;
    }

    esp_ble_gap_ext_adv_params_t ext_adv_params = {
        .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_NONCONN_NONSCANNABLE_UNDIRECTED,
        .interval_min = 0x30,
        .interval_max = 0x60,
        .channel_map = ADV_CHNL_ALL,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
        .tx_power = EXT_ADV_TX_PWR_NO_PREFERENCE,
        .primary_phy = ESP_BLE_GAP_PHY_1M,
        .max_skip = 0,
        .secondary_phy = ESP_BLE_GAP_PHY_1M,
        .sid = 0,
        .scan_req_notif = false,
    };

    err = esp_ble_gap_ext_adv_set_params(BLE_EXT_ADV_INSTANCE, &ext_adv_params);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set adv params request failed: %s", esp_err_to_name(err));
    }
}

esp_err_t ble_beacon_publish_tag_data(const char *device_id,
                                      const uint8_t *epc,
                                      uint8_t epc_len,
                                      const char *vehicle_door)
{
    if (device_id == NULL || device_id[0] == '\0' || epc == NULL || epc_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_params_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    char plaintext[BLE_PLAINTEXT_MAX_LEN];
    size_t plaintext_len = 0;
    ESP_RETURN_ON_ERROR(build_plaintext(device_id, epc, epc_len,
                                        vehicle_door, plaintext, &plaintext_len),
                        TAG, "build plaintext");

    esp_err_t err = set_adv_data(device_id, (const uint8_t *)plaintext, plaintext_len);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "advertising %s", plaintext);
    } else {
        ESP_LOGW(TAG, "publish failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t ble_beacon_publish_tag(const char *device_id,
                                 const uint8_t *epc,
                                 uint8_t epc_len)
{
    return ble_beacon_publish_tag_data(device_id, epc, epc_len, NULL);
}
