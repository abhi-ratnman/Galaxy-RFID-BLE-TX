# Galaxy-RFID-BLE-TX

ESP-IDF firmware for an ESP32-S3 that reads MU60x-series UHF RFID tags over
UART and rebroadcasts the latest EPC over BLE extended advertising.

The firmware has two runtime paths:

- `components/mu60x`: polling UART driver for the MU60x host-interface protocol.
- `components/bleBeacon`: transmitter-only BLE extended advertiser for
  `device_id,epc_hex` payloads.

## Hardware Wiring

| RFID module | ESP32-S3 | Notes |
|-------------|----------|-------|
| TX | GPIO15 | RFID TX to ESP RX |
| RX | GPIO16 | ESP TX to RFID RX |
| VCC | 3V3 | Match the module power requirement |
| GND | GND | Common ground |
| RST | Not wired | The tested module runs without reset control |

UART is `UART1` at 115200 bps, 8N1, reader address `0x00`.

## Build

From an ESP-IDF PowerShell:

```powershell
$env:IDF_TOOLS_PATH="C:\Espressif"
$env:IDF_PYTHON_ENV_PATH="C:\Espressif\python_env\idf5.5_py3.11_env"
$env:IDF_PATH="C:\Espressif\frameworks\esp-idf-v5.5.3"
& "$env:IDF_PATH\export.ps1"
idf.py set-target esp32s3
idf.py build
```

Flash and monitor:

```powershell
idf.py -p COMx flash monitor
```

## Runtime Behavior

1. `app_main()` initializes BLE and starts the RFID polling task.
2. The MU60x driver configures the reader for ETSI, 20 dBm, and real-time
   inventory on antenna 1.
3. Each detected EPC is logged and published through BLE manufacturer data.
4. The first detected tag also triggers a one-shot TID read demo.

The device ID is currently hardcoded in `main/main.c`:

```c
#define DEVICE_ID "RFID_001"
```

## BLE Payload

Logical plaintext before optional encryption:

```text
<device_id>,<epc_hex>
```

Example:

```text
RFID_001,E28069150000502DA3F7C197
```

Manufacturer data layout when encryption is enabled:

```text
AD length | AD type 0xFF | company id 0x0059 | version | nonce | ciphertext | auth tag
```

The firmware uses AES-128-CCM with:

- Company ID: `0x0059`
- Version: `1`
- Nonce: reserved byte + FNV-1a device-id hash + 64-bit boot counter
- Auth tag length: 4 bytes

Set `BLE_BEACON_ENCRYPTION_ENABLED` to `0` in
`components/bleBeacon/bleBeacon.c` for plaintext BLE debugging, then restore it
to `1` before shipping.

## RFID Region

The tested reader shipped configured for FCC. ETSI tags used in India/EU were
not detected until the reader region was changed to ETSI. The firmware applies
ETSI + 20 dBm during initialization and saves the reader parameters only when
they differ, avoiding unnecessary reader flash writes.

Change `.region` and `.power_dbm` in `main/main.c` if your deployment region
or RF power limit differs.

## PC Test Tool

`tools/mu60x_test.py` is an interactive host-side MU60x test harness for a
USB-to-TTL adapter.

```powershell
pip install pyserial
python tools\mu60x_test.py --port COM5
mu60x> ver
mu60x> setregion etsi
mu60x> save
mu60x> rt 1
```

## Protocol Reference

The MU60x host-interface manual is kept in `reference/`:

```text
reference/UHF RFID Reader_MU60x Series_Host Interface Packet Definitions_v1.2.1_EN.pdf
```

## Not Implemented

This firmware intentionally does not expose RFID kill/lock commands, BLE
scanning, GATT services, Wi-Fi, NVS key provisioning, tag history buffering, or
receiver-side BLE decryption.
