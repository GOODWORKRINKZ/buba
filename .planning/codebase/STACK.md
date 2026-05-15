---
last_updated: 2026-05-15
focus: tech
---

# Tech Stack

## Languages

**Primary:**
- C++ (Arduino framework, C++11/14) — all application logic in `src/main.cpp` and `src/test_bu04.cpp`
- C — underlying ESP-IDF / Arduino core (transitive)

## Runtime & Platform

**MCU:**
- Espressif ESP32-C3 (RISC-V, single-core, 160 MHz)
- Board target: `esp32-c3-devkitc-02` (set in `platformio.ini`)
- USB CDC boot enabled: `-DARDUINO_USB_CDC_ON_BOOT=1 -DARDUINO_USB_MODE=1`

**Framework:**
- Arduino (via PlatformIO `framework = arduino`)
- PlatformIO platform: `espressif32`

## Build System

**Tool:** PlatformIO (`platformio.ini`)

**Build commands:**
```bash
pio run -e anchor1      -t upload   # flash ANCHOR1 (TWR)
pio run -e anchor2      -t upload   # flash ANCHOR2 (TWR)
pio run -e tag          -t upload   # flash TAG
pio run -e anchor1_pdoa -t upload   # flash ANCHOR1 (PDOA)
pio run -e test_bu04    -t upload   # flash UART diagnostic tool
pio device monitor -b 115200        # open Serial Monitor
```

**Environments defined in `platformio.ini`:**
| Environment | Build Flag | Purpose |
|-------------|------------|---------|
| `anchor1` | `-DROLE_ANCHOR1=1` | Main anchor, computes direction (TWR) |
| `anchor1_pdoa` | `-DROLE_ANCHOR1=1 -DPDOA_MODE=1` | Single anchor with phase-difference angle (PDOA) |
| `anchor2` | `-DROLE_ANCHOR2=1` | Secondary anchor, relays d2 over ESP-NOW (TWR) |
| `tag` | `-DROLE_TAG=1` | Tag carried by user |
| `test_bu04` | `-DROLE_TEST=1` | UART diagnostic; uses `src/test_bu04.cpp` only |

**Source filtering:** `test_bu04` environment excludes `main.cpp` via `build_src_filter`.

**No external library dependencies** — BU04 is controlled entirely via UART AT commands; no UWB SDK or SPI library is required.

## Key Libraries & Dependencies

All libraries come from the Arduino / ESP-IDF core bundled with `espressif32` platform:

| Library | Header | Purpose |
|---------|--------|---------|
| Arduino core | `<Arduino.h>` | Setup/loop lifecycle, `String`, `Serial`, `millis()`, `delay()` |
| HardwareSerial | `<HardwareSerial.h>` | UART1 to BU04 (AT commands) |
| WiFi | `<WiFi.h>` | ESP-NOW transport layer (ANCHOR1, ANCHOR2 only) |
| ESP-NOW | `<esp_now.h>` | Peer-to-peer packet relay (ANCHOR2 → ANCHOR1) |
| ESP-IDF version | `<esp_idf_version.h>` | Compile-time IDF version check |
| math | `<math.h>` | `atan2()`, `sqrt()` for trilateration / angle computation |

## Configuration

**Primary config file:** `include/config.h`

Key compile-time constants:

| Constant | Value | Purpose |
|----------|-------|---------|
| `PIN_BU04_TX` | `2` | ESP32-C3 GPIO2 → BU04 USART2_RX (PA3) |
| `PIN_BU04_RX` | `3` | ESP32-C3 GPIO3 ← BU04 USART2_TX (PA2) |
| `BU04_BAUD` | `115200` | UART baud rate to BU04 |
| `SERIAL_BAUD` | `115200` | USB Serial (telemetry output) |
| `BU04_CHANNEL` | `1` | UWB CH5 (6489.5 MHz) |
| `BU04_RATE` | `1` | 6.8 Mbit/s (only supported rate) |
| `BU04_GROUP` | `1` | UWB group for anchors (tags = 0) |
| `BU04_ID_ANCHOR1` | `0` | Device ID for anchor1 |
| `BU04_ID_ANCHOR2` | `1` | Device ID for anchor2 |
| `BU04_ID_TAG` | `0` | Device ID for tag (must equal ANCHOR1 ID in TWR) |
| `BASELINE_M` | `0.30f` | Physical distance between anchor1 and anchor2 (metres) |
| `ANCHOR1_MAC` | `{0xFF,...}` | ESP-NOW MAC of ANCHOR1 (must be updated manually) |
| `ESPNOW_MAGIC` | `0xBB04` | Packet marker to filter foreign ESP-NOW traffic |
| `POLL_INTERVAL_MS` | `200` | AT+DISTANCE polling period (ms) |
| `ANCHOR2_STALE_MS` | `1000` | anchor2 data expiry timeout (ms) |
| `MOVING_AVG_SAMPLES` | `5` | Sliding window for distance averaging |
| `MAX_VALID_DIST_M` | `50.0f` | Maximum accepted UWB distance (metres) |

**Build flags (common, from `platformio.ini`):**
```
-DSERIAL_BAUD=115200
-DCORE_DEBUG_LEVEL=5
-DARDUINO_USB_CDC_ON_BOOT=1
-DARDUINO_USB_MODE=1
```

**Monitor filters:** `esp32_exception_decoder`, `time` (via PlatformIO monitor).

## Hardware Platform

**Host MCU:** ESP32-C3 SuperMini
- Architecture: RISC-V 32-bit single-core
- USB: native USB-CDC (no separate USB-UART chip)
- GPIO2 = TX to BU04, GPIO3 = RX from BU04
- 3V3 power supply must source ≥500 mA peak for BU04

**UWB Module:** Ai-Thinker BU04
- UWB chip: Decawave/Qorvo DW3000
- Microcontroller inside: STM32F103
- Antenna: dual-antenna (enables PDOA angle measurement)
- Interface to host: UART AT commands only (DW3000 is not directly accessible via SPI)
- Primary UART: USART2 — PA2 (TX, pin 4), PA3 (RX, pin 5)
- Fallback UART: USART1 — PA9 (TX, pin 26), PA10 (RX, pin 27)
- AT command spec: BU03/BU04 AT指令 V1.0.6
- Hardware spec: BU04 规格书 V1.0.0

**UWB Frequency:**
- CH5: 6489.5 MHz (configured via `BU04_CHANNEL=1`)
- CH9: 7987.2 MHz (alternative, `BU04_CHANNEL=0`)

---

*Stack analysis: 2026-05-15*
