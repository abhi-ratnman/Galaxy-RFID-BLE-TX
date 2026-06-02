#ifndef __BLEBEACON_H__
#define __BLEBEACON_H__

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void ble_beacon_init(void);

esp_err_t ble_beacon_publish_tag(const char *device_id,
                                 const uint8_t *epc,
                                 uint8_t epc_len);

esp_err_t ble_beacon_publish_tag_data(const char *device_id,
                                      const uint8_t *epc,
                                      uint8_t epc_len,
                                      const char *vehicle_door);

#ifdef __cplusplus
}
#endif

#endif // __BLEBEACON_H__
