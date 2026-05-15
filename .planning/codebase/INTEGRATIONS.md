---
last_updated: 2026-05-15
focus: tech
---

# External Integrations

## Hardware Modules

**Ai-Thinker BU04 (UWB ranging module):**
- Chip: Decawave/Qorvo DW3000 (UWB), STM32F103 (host MCU inside module)
- Dual antenna — enables PDOA (Phase Difference of Arrival) angle measurement
- Spec: BU04 规格书 V1.0.0 (pin table, power requirements)
- AT command reference: BU03/BU04 AT指令 V1.0.6
- Power: 3V3, peak current ≥500 mA; powered from ESP32-C3 3V3 rail
- **One BU04 per ESP32-C3 board** — three boards for TWR mode, two for PDOA mode

## Communication Protocols

### UART (ESP32-C3 ↔ BU04)
- Interface: UART1 on ESP32-C3 (`HardwareSerial bu04(1)` in `src/main.cpp`)
- Baud rate: 115200 (`BU04_BAUD` in `include/config.h`)
- Pins: GPIO2 (TX → BU04 PA3/USART2_RX), GPIO3 (RX ← BU04 PA2/USART2_TX)
- Fallback pins: GPIO2 → PA10/USART1_RX, GPIO3 ← PA9/USART1_TX
- Protocol: ASCII AT commands terminated with `\r\n`; responses terminated with `OK` or `ERR`

### ESP-NOW (ANCHOR2 → ANCHOR1, TWR mode only)
- Library: `<esp_now.h>` + `<WiFi.h>` (ESP-IDF bundled)
- Direction: unidirectional — ANCHOR2 pushes distance packet to ANCHOR1
- Addressing: ANCHOR1 MAC address hard-coded in `include/config.h` (`ANCHOR1_MAC`)
- Packet marker: `ESPNOW_MAGIC = 0xBB04` (filters out foreign packets)
- Payload: float `d2_m` (distance from anchor2 to tag, in metres)
- Not used in PDOA mode

### USB Serial (ESP32-C3 → PC)
- Interface: Native USB-CDC on ESP32-C3 (`-DARDUINO_USB_CDC_ON_BOOT=1`)
- Baud rate: 115200
- Purpose: CSV telemetry output + debug log to host PC

### UWB (BU04 ↔ BU04, air interface)
- Chip: DW3000, managed entirely inside BU04 firmware
- Frequency: CH5 (6489.5 MHz) via `BU04_CHANNEL=1`; CH9 (7987.2 MHz) is alternative
- Data rate: 6.8 Mbit/s (only supported rate, `BU04_RATE=1`)
- Group filtering: `BU04_GROUP=1` for anchors, `0` for tags
- **TWR mode:** Two-Way Ranging — time-of-flight distance measurement
- **PDOA mode:** Phase Difference of Arrival — single anchor measures distance + angle

## AT Command Interface

All BU04 interaction is via UART AT commands. Defined as string constants in `include/config.h`.

### General Commands
| AT Command | Response | Purpose |
|------------|----------|---------|
| `AT` | `OK` | Connectivity check / ping |
| `AT+SAVE` | `OK` + reboot (~3 s) | Persist configuration to NVM and reset |
| `AT+RESTART` | — | Soft reboot |
| `AT+RESTORE` | — | Factory reset |
| `AT+GETVER` | `getver software:V1.0.0,hardware:V1.0.0` | Firmware / hardware version |
| `AT+GETCFG` | See below | Read current configuration |
| `AT+SETCFG=ID,Role,CH,Rate` | `OK` | Set device config (4 params; Group not supported) |

`AT+GETCFG` response format (TWR mode):
```
getcfg ID:X, Role:X, CH:X, Rate:X, Group:X
```
`AT+GETCFG` response format (PDOA mode):
```
getcfg Dlist:N KList:N Net:XXXX AncID:N Rate:N Filter:N UserCmd:N pdoaOffset:N rngOffset:N
```

`AT+SETCFG` parameter encoding:
- `ID`: device address (0–10)
- `Role`: `0` = tag, `1` = anchor
- `CH`: `0` = CH9 (7987.2 MHz), `1` = CH5 (6489.5 MHz)
- `Rate`: always `1` (6.8 Mbit/s)

### TWR Measurement Commands
| AT Command | Response | Purpose |
|------------|----------|---------|
| `AT+DISTANCE` | `distance: X.XXXXXX` or `distance:ID X.XXXXXX` | Request distance to paired tag (metres) |
| `AT+GETDEV` | coefficients | Read device calibration (delay, Kalman params, correction coefficients) |

Response parsing: `src/main.cpp` → `parseDistance()` handles both single-anchor and multi-anchor response formats.

### UWB Mode Commands
| AT Command | Purpose |
|------------|---------|
| `AT+GETUWBMODE` | Read current UWB mode (TWR or PDOA) |
| `AT+SETUWBMODE=0` | Set TWR mode (written to RAM; requires `AT+SAVE` to persist) |
| `AT+SETUWBMODE=1` | Set PDOA mode (written to RAM; requires `AT+SAVE` to persist) |
| `AT+SETWORKMODE=0` | Normal mode — starts node/tag task on boot |
| `AT+SETWORKMODE=1` | AT-only mode — stays in AT loop forever, node/tag never starts |

### PDOA-Specific Commands (anchor1_pdoa only)
| AT Command | Purpose |
|------------|---------|
| `AT+DECA$` | PDOA authentication (must be sent before PDOA operations) |
| `AT+GETDLIST` | List detected (unregistered) tags |
| `AT+GETKLIST` | List registered (paired) tags |
| `AT+ADDTAG=LongAddr64,ShortAddr,MinRate,MaxRate,Mode` | Register a tag (MaxRate ≤ 64) |
| `AT+DELTAG=LongAddr64` | Remove a tag from pair list |
| `AT+USER_CMD=0` | Set output format to JSON (PDOA data stream) |
| `AT+USER_CMD=1` | Set output format to HEX |
| `AT+PDOAOFF` | Read/set PDOA angle offset correction |
| `AT+RNGOFF` | Read/set ranging distance offset correction |
| `AT+FILTER` | Enable/disable measurement filtering |
| `AT+PDOAGETCFG` | Read PDOA configuration parameters |
| `AT+PDOASETCFG=p1,...,p7` | Set PDOA configuration (7 parameters) |

### TAG Configuration — Two-Phase Strategy
Direct `AT+SETCFG` to role=0 (tag) causes DW3000 init failure (`INIT FAILED`) on cold boot because `tag_start()` skips `reset_DWIC`. Workaround implemented in `src/main.cpp` → `configureBU04()`:

1. **Phase 1** (ID=65535 or unconfigured → PDOA anchor): send `AT+SETCFG=id,1,...` + `AT+SETUWBMODE=1` + `AT+SAVE`
   → `node_start()` runs `reset_DWIC` → DW3000 initialised → reboot → device becomes PDOA anchor
2. **Phase 2** (PDOA anchor → tag): send `AT+SETCFG=id,0,...` + `AT+SAVE`
   → `tag_start()` called from PDOA event loop (DW3000 already live) → reboot → device becomes tag

## Wireless Communication

### ESP-NOW
- Role: carries ANCHOR2's measured distance (`d2_m`) to ANCHOR1 in real time
- No router required — direct peer-to-peer Wi-Fi layer 2
- Packet contains: magic marker (`0xBB04`) + float distance value
- Data freshness enforced: `ANCHOR2_STALE_MS = 1000 ms` in `include/config.h`
- Setup: flash anchor1, read its MAC from Serial Monitor, paste into `ANCHOR1_MAC` in `include/config.h`, then flash anchor2

### UWB (DW3000 inside BU04)
- **TWR (Two-Way Ranging):**
  - Each anchor polls `AT+DISTANCE` every `POLL_INTERVAL_MS` (200 ms)
  - ANCHOR1 collects d1 (own) + d2 (from ANCHOR2 via ESP-NOW)
  - Trilateration formula (in `src/main.cpp`):
    - `x = (d1² − d2² + B²) / (2·B)`
    - `y = √(d1² − x²)`
    - `angle = atan2(y, x)` (angle from ANCHOR1→ANCHOR2 axis)
  - Requires: ANCHOR1 (ID=0), ANCHOR2 (ID=1), TAG (ID=0)

- **PDOA (Phase Difference of Arrival):**
  - Single anchor with dual antennas; measures both range and angle in one pass
  - Anchor streams JSON packets when tag is registered via `AT+ADDTAG`
  - Activated with `AT+SETUWBMODE=1` + `AT+SAVE`
  - Does not use ANCHOR2 or ESP-NOW

## Data Output

### Serial Telemetry (USB-CDC, 115200 baud)
CSV lines printed to `Serial` by ANCHOR1 (or TAG) for logging/visualisation on host PC.

**TWR mode formats:**
```
ANCHOR1,seq,d1_m,avg_m,ts_ms
ANCHOR2,seq,d2_m,ts_ms
PLATFORM,seq,d1_m,d2_m,angle_deg,x_m,y_m,ts_ms
TAG,seq,d_m,avg_m,ts_ms
```

**PDOA mode format:**
```
PDOA,addr_hex,seq,range_m,angle_deg,x_m,y_m,ts_ms
```

Field descriptions:
- `seq`: rolling sequence counter (uint8, wraps at 255)
- `d1_m`, `d2_m`, `range_m`: UWB distance in metres (float)
- `avg_m`: sliding-window average over `MOVING_AVG_SAMPLES=5` readings
- `angle_deg`: computed bearing in degrees
- `x_m`, `y_m`: Cartesian coordinates relative to ANCHOR1 origin
- `ts_ms`: `millis()` timestamp at measurement time
- `addr_hex`: tag's 64-bit long address (PDOA mode only)

### Diagnostic / Pass-Through (test_bu04 environment)
- Environment: `test_bu04` (uses `src/test_bu04.cpp`)
- Sends `AT` ping every 3 seconds; prints raw hex + ASCII dump of BU04 response
- Interactive pass-through: Serial Monitor input → BU04, BU04 output → Serial Monitor
- Auto-sequence on boot: AT → GETCFG → SETCFG → GETCFG → SAVE → wait → AT → GETCFG

---

*Integration audit: 2026-05-15*
