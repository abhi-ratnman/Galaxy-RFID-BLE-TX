# RFID BLE Transmitter Design

## Goal

Broadcast each detected MU60x RFID tag EPC over BLE advertising with a
hardcoded device ID and optional vehicle door number read from RFID EPC
memory.

## Scope

This firmware is transmitter-only:

- RFID input comes from the MU60x UART driver.
- BLE output uses non-connectable, non-scannable extended advertising.
- The BLE payload contains `device_id,epc_hex[,vehicle_door_number]`.

Receiver/scanner code, GATT services, Wi-Fi, display publishing, and unrelated
GASM dependencies are out of scope.

## Data Flow

1. `app_main()` calls `ble_beacon_init()`.
2. `app_main()` starts `rfid_task`.
3. `rfid_task` initializes the MU60x reader and starts real-time inventory.
4. `mu60x_poll()` returns a decoded `mu60x_tag_t` when a tag is detected.
5. For each new EPC, `rfid_task` stops inventory and can write the vehicle door
   number into EPC memory when write mode is enabled.
6. `rfid_task` logs the EPC and calls:

```c
ble_beacon_publish_tag_data(DEVICE_ID, tag.epc, tag.epc_len, vehicle_door);
```

7. The BLE component updates the extended advertising payload to the latest
   observed tag.

## RFID EPC Data

Vehicle door data is stored in EPC bank word address `2`, word count `6`.
That gives 12 bytes of EPC storage: up to 10 ASCII bytes plus zero padding.

Valid values are 1 to 10 characters using only `0-9`, `A-Z`, and `a-z`.
Write mode is compile-time controlled from `main/main.c`, then normal firmware
operation should be returned to read mode.

This changes the RFID tag identity. A door value such as `A123BC4567` is stored
as EPC bytes `41 31 32 33 42 43 34 35 36 37 00 00`.

## BLE Manufacturer Data

Plaintext logical payload:

```text
<device_id>,<epc_hex>[,<vehicle_door_number>]
```

Advertising data includes:

```text
Complete Local Name: RFID_001
Manufacturer data: company id 0xFFFF | version | payload...
```

Encrypted manufacturer payload:

```text
nonce
ciphertext
auth tag
```

Nonce format:

```text
reserved byte + 32-bit FNV-1a(device_id) + 64-bit boot counter
```

The receiver must know the company ID, version, nonce layout, AES key, and
4-byte CCM tag length.

## Error Handling

RFID polling does not depend on BLE success. If BLE is not ready when a tag is
detected, `ble_beacon_publish_tag_data()` returns `ESP_ERR_INVALID_STATE` and
the RFID task continues.

Encryption or payload-size failures are logged and the previous advertisement
payload remains active.

## Verification

Build verification:

```powershell
idf.py build
```

Runtime verification:

1. Flash and monitor the ESP32-S3.
2. Confirm BLE logs `extended advertising started`.
3. Present an RFID tag.
4. Confirm the monitor logs the EPC.
5. Confirm BLE advertising manufacturer data changes for the latest EPC.
