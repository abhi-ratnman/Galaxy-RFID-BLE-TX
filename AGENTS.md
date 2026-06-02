# Agent Notes

## Project State

- ESP-IDF firmware for ESP32-S3 using an MU60x UHF RFID reader on UART1.
- BLE advertising is plaintext for debugging:
  - Device name: `RFID_001`
  - Manufacturer company ID: `0xFFFF`
  - Payload: `RFID_001,<epc_hex>[,<vehicle_door_number>]`
- The current RFID tag failed USER-bank access, so vehicle door data is now
  stored in EPC memory instead of USER memory.

## Door Number In EPC

Door data is written to EPC bank word address `2`, word count `6`.
That is 12 EPC bytes: up to 10 alphanumeric ASCII bytes plus zero padding.

This changes the RFID tag identity. Present only one tag while write mode is
enabled.

Write mode in `main/main.c`:

```c
#define VEHICLE_DOOR_MODE          VEHICLE_DOOR_MODE_WRITE
#define VEHICLE_DOOR_NUMBER        "A123BC4567"
```

After seeing `vehicle door EPC write verified`, switch back to:

```c
#define VEHICLE_DOOR_MODE          VEHICLE_DOOR_MODE_READ
```

Then flash again for normal scanning.

## Verification Notes

- `main.c` passed direct ESP-IDF compiler check after the EPC-storage change.
- Full `idf.py build` / Ninja build has repeatedly hung in this sandbox, so
  prefer checking on the user's ESP-IDF PowerShell.
- `build/` and `sdkconfig` are ignored/generated.
- The screenshot in `reference/WhatsApp Image 2026-06-02 at 12.54.48 PM.jpeg`
  is local/untracked and has not been pushed.

## Known Issues

- RSSI/frequency decode from MU60x inventory frames appears offset or model
  specific; EPC reads are valid, but logged RSSI/frequency values may be wrong.
- ESP32 flash warning may show 8 MB detected but 2 MB configured. This is not
  blocking RFID/BLE tests, but can be fixed later in ESP-IDF flash-size config.
