# buba – BU04 Ai-Thinker UWB distance telemetry

Test project for the **Ai-Thinker BU04 UWB module** connected to an
**ESP32-C3 SuperMini**.  
One module is mounted on a mobile platform (**TAG**) and the other is
carried by the user (**ANCHOR**).  
The firmware measures the distance between them using
**Double-Sided Two-Way Ranging (DS-TWR)** and streams telemetry over USB-Serial.

---

## Hardware

| Qty | Part |
|-----|------|
| 2   | Ai-Thinker BU04 UWB module (Decawave DW1000) |
| 2   | ESP32-C3 SuperMini dev board |
| 2   | USB-C cables (flashing / power) |

### Wiring (same for both devices)

```
BU04 pin   Signal     ESP32-C3 SuperMini pin
────────   ──────     ──────────────────────
VCC        3.3 V      3V3
GND        GND        GND
SCK        SPI CLK    GPIO 4
MISO       SPI MISO   GPIO 5
MOSI       SPI MOSI   GPIO 6
NSS/CS     SPI CS     GPIO 7
RST        RESET      GPIO 8
IRQ        INT        GPIO 9
```

---

## Project structure

```
buba/
├── platformio.ini      – two build environments: anchor / tag
├── include/
│   └── config.h        – pin map, protocol constants, tuning
└── src/
    └── main.cpp        – DS-TWR firmware (role selected at compile time)
```

---

## Building & flashing

Install [PlatformIO CLI](https://platformio.org/install/cli) or use the
VS Code extension.

```bash
# Flash the ANCHOR firmware (device the user carries)
pio run -e anchor -t upload

# Flash the TAG firmware (device on the mobile platform)
pio run -e tag -t upload
```

Open Serial monitors (115 200 baud) on both devices:

```bash
pio device monitor -b 115200
```

---

## Telemetry output (CSV over Serial)

**TAG** (mobile platform):
```
# CSV: TAG,seq,distance_m,distance_mm,avg_m,timestamp_ms
TAG,1,1.234,1234,1.234,5012
TAG,2,1.251,1251,1.242,5212
```

**ANCHOR** (user-held):
```
# CSV: ANCHOR,seq,distance_m,distance_mm,timestamp_ms
ANCHOR,1,1.234,1234,5011
ANCHOR,2,1.251,1251,5211
```

| Field | Description |
|-------|-------------|
| `seq` | Sequence number (increments each round) |
| `distance_m` | Measured distance in metres (3 decimal places) |
| `distance_mm` | Distance in whole millimetres |
| `avg_m` | (TAG only) Moving average over last 5 samples |
| `timestamp_ms` | `millis()` on the device at measurement time |

---

## Protocol overview (DS-TWR)

```
TAG                                ANCHOR
 │                                    │
 │──── POLL (t1) ────────────────────►│ t2
 │                                    │
 │◄─── POLL_ACK (t3) ─────────────────│
 │ t4                                 │
 │                                    │
 │──── FINAL [t1,t4,t5] (t5) ────────►│ t6
 │                                    │
 │◄─── RANGE_REPORT [dist_mm] ────────│
 │                                    │
```

Distance formula:

```
Ra = t4 - t1  (tag round-trip)
Rb = t6 - t3  (anchor round-trip)
Da = t5 - t4  (tag reply delay)
Db = t3 - t2  (anchor reply delay)

ToF   = (Ra·Rb − Da·Db) / (Ra + Rb + Da + Db)
dist  = ToF × 299 702 547 m/s × 15.65 ps/tick
```

---

## Tuning

Edit `include/config.h` to adjust:

| Constant | Default | Description |
|----------|---------|-------------|
| `RANGING_INTERVAL_MS` | 200 | Time between TAG ranging rounds (ms) |
| `RX_TIMEOUT_MS` | 200 | Reply timeout per message (ms) |
| `MOVING_AVG_SAMPLES` | 5 | Smoothing window size |
| `MAX_VALID_DISTANCE_M` | 300 | Max accepted distance for sanity filter |

---

## Notes on direction / AoA

The BU04 module (single antenna, DW1000 chip) provides **distance only**.
Direction / Angle of Arrival (AoA) requires either:
* a multi-antenna module (e.g. DW3000-based), **or**
* deploying 3+ anchors at known positions and doing trilateration.

For the current telemetry phase, distance data alone is sufficient to
characterise the setup and validate hardware behaviour.
