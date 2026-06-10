# Phase 3: BU04 Robot Following — Research

**Date:** 2026-06-10
**Source:** Consolidated from `.planning/research/ROBOT-FOLLOWING.md` + `03-CONTEXT.md`
**Status:** Research complete — ready for planning

---

## 1. Key Finding: 3-Base-Station Trilateration (Patent-Backed)

### CN105828431A Patent — Exact Analytic Solution

**University of Shanghai for Science and Technology (2016)** — THE reference for on-robot UWB following.

**Setup:**
- 3× UWB base stations on robot in **equilateral triangle** (side = a)
- 1× UWB tag on human
- TOA distance measurement → closed-form analytic solution (no iteration)

**Formula:**
```
Given: d1, d2, d3 = distances from A, B, C to tag M
       a = triangle side length (30-50 cm)

L = √((d1⁴ + d2⁴ + d3⁴ - d1²d2² - d1²d3² - d2²d3²) / (3·a²))
θ = arctan(√3(d3² - d2²) / (-2d1² + d2² + d3²))

Output: (L, θ) — polar coordinates from robot center
Claimed accuracy: ±10 cm
```

**Coordinate system:** O(0,0,0) = midpoint AB, y-axis = robot forward, H = triangle center.

### Supporting Paper: ETH Zurich UWBTracker

**Nägeli, Hepp, Hilliges** — 4× UWB on drone + 1 tag, IEKF, 10 cm @ 4 m.
- Active-Passive Ranging for high update rate
- Handles occlusion (UWB penetrates obstacles that block cameras)
- Omni-directional — no limited FOV

### Supporting Paper: IFAC 2024 — UWB + RGB-D Hybrid

UWB for long-range detection (no FOV limits), RGB-D camera for close-range precision.

### Supporting Paper: Appl. Sci. 2024 — Polar Robot Following

UWB works where cameras/LiDAR fail (snow glare, fog, polar conditions).

---

## 2. Architecture Decision: 4× BU04 + RP2040 Co-Processor

```
                 ┌──────────────────────────────────────┐
                 │         RP2040 (UWB Co-Processor)     │
                 │                                      │
BU04-A──UART0──→ │  d1 ┐                                │
BU04-B──UART1──→ │  d2 ├─ analytic_solver(L,θ) ──UART──→│ ROBOT
BU04-C──PIO ───→ │  d3 ┘  + Kalman (optional)   I2C ──→│ (ROS2/STM32)
                 │                                      │
                 │  Parse "BTN:1" from UWB data frames  │
                 │  → forward command to robot          │
                 └──────────────────────────────────────┘

TAG (on person):
  BU04 + LiPo 500mAh + omni antenna (IPEX) + button (GPIO)
  TWR responses: automatic (stock firmware)
  Button: GPIO → dwt_writetxdata("BTN1") → over UWB
```

### Why RP2040 (not ESP32)

| Feature | RP2040 | ESP32 |
|---------|--------|-------|
| PIO (unlimited UART) | ✅ | ❌ (3 HW max) |
| Deterministic timing | ✅ No RTOS | ❌ FreeRTOS jitter |
| Power | ~20 mA | ~80 mA+ |
| Price | ~$4 | ~$5 |

### Why UART (not I2C/SPI) for BU04↔RP2040

BU04 stock firmware is UART-only for AT commands. I2C hardware exists on PB6/PB7 but firmware does not listen on it. Custom STM32 firmware via SWD is massive effort. RP2040 PIO provides unlimited UARTs for free.

### Output Protocol to Robot

```
TAG:150,25\r\n      — tag at 150 cm, +25° azimuth
TAG:120,0\r\n       — tag at 120 cm, straight ahead
TAG:LOST\r\n         — all 3 anchors lost the tag
TAG:NOISE\r\n        — measurements too noisy, hold position
```

ASCII, 115200 baud, single `scanf` parse on robot side.

---

## 3. BU04 Antenna Constraints

- Two PCB antennas on opposite edges (RF1, RF2) — directional, ~120° beam (±60°)
- Back side = ~0% signal
- **TWR trilateration:** Mount each BU04 with antenna edge facing OUTWARD from triangle → 3×120° = 360° combined
- **Mounting:** Plastic bracket ≥5 cm above metal; antenna area must overhang or have cutout below
- **Tag:** Replace PCB antenna with external omni via IPEX connector (~$2)

---

## 4. Data-over-UWB: Commands Through DW3000

Stock AT firmware doesn't expose data TX, but SDK makes it trivial:

```c
// Add to cmd_fn.c:
int f_send_btn(int opt, int argc, char* argv[]) {
    uint8_t data[] = {0xC5, 0, 'B','T','N', (uint8_t)atoi(argv[0])};
    dwt_writetxdata(sizeof(data), data, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    return 0;
}
// Register: {"AT+BUTTON", f_send_btn},
```

DW3000 API supports up to 1023 bytes at 6.8 Mbps. No extra radio needed.
Existing SDK example `ex_03a_tx_wait_resp` already sends "DECAWAVE" over UWB.

---

## 5. Tag Hardware — Minimal Design

Tag = BU04 in TAG mode, no extra MCU needed. STM32 handles TWR responses autonomously.

| Component | Spec |
|-----------|------|
| BU04 | TAG mode, `SETCFG=0,1,5,2` |
| Battery | LiPo 1S 500 mAh (~10 h runtime) |
| Antenna | External omni via IPEX (~$2) |
| Charger | TP4056 USB-C |
| Button | GPIO → `AT+BUTTON` for "follow me" |
| Optional | LED, buzzer |

---

## 6. Hardware BOM

| Item | Qty | Unit Price | Total |
|------|-----|-----------|-------|
| BU04 module (buy) | 2 | ~$15 | ~$30 |
| BU04 module (have) | 2 | — | $0 |
| RP2040 board | 1 | ~$4 | ~$4 |
| LiPo 1S 500 mAh | 1 | ~$5 | ~$5 |
| IPEX omni antenna | 1 | ~$2 | ~$2 |
| TP4056 charger | 1 | ~$1 | ~$1 |
| **TOTAL** | | | **~$42** |

---

## 7. Architecture Alternatives (Rejected)

| Alternative | Why Rejected |
|-------------|-------------|
| Single PDOA anchor | 120° coverage only, loses tag easily |
| Dual TWR anchors | Left/right ambiguity (±y) |
| Room-fixed anchors (Pozyx) | Requires room setup, 6 modules, Phase 6+ |
| 4-receiver IEKF (ETH) | Extra module cost, complex filter, overkill for v1 |
| BU03 omni modules | No PDOA, different firmware, inconsistent |

---

## 8. Gray Areas for Planner

| # | Question | Hint |
|---|----------|------|
| G1 | Triangle side **a** = ? | 30 cm fits small robot, 50 cm = better geometry |
| G2 | RP2040 poll rate? | 4 Hz minimum, 10 Hz desired |
| G3 | Output protocol? | `TAG:L,θ` ASCII vs binary 4-byte |
| G4 | 2-of-3 anchors lost → continue or stop? | Geometry degrades, but can estimate |
| G5 | TWR per-module calibration needed? | APS014 describes procedure |
| G6 | Tag battery indicator? Auto-sleep? | ~10 h with 500 mAh LiPo |
| G7 | Kalman on RP2040 or robot? | Start without, add if noisy |

---

## 9. SDK Extension Points

- **AT command table:** `STM32F103-BU0x_SDK/Components/APP/cmd_fn.c` — `known_commands[]` array
- **DW3000 TX API:** `deca_device_api.h` — `dwt_writetxdata()`, `dwt_starttx()`
- **Reference example:** `ex_03a_tx_wait_resp` — sends data over UWB
- **AT command framework:** `cmd.c` / `cmd.h` — `EXECUTE_CMD`, `SET_CMD`, `QUERY_CMD` modes

---

## 10. Deferred / Out of Scope

- Kalman filter on RP2040 (add after basic trilateration works)
- Room-fixed anchors for absolute positioning (Phase 6+)
- Multi-tag support (Phase 6+)
- ROS2 `biba_uwb_follow` node implementation (Phase 6+)
- BU04 custom STM32 firmware for I2C slave mode (not needed — UART works)

---

## References

- Full research: `.planning/research/ROBOT-FOLLOWING.md`
- BU04 spec: `docs/datasheets/bu04_v1.0.0_specification-20240801.pdf`
- AT commands: `docs/datasheets/BU03_BU04_AT_command_cn_V1.0.6.pdf`
- SDK: `STM32F103-BU0x_SDK/`
- Patent: `docs/patents/CN105828431A.pdf`
- ETH paper: `docs/uwb-papers/UWBTracker.pdf`
- IFAC paper: `docs/uwb-papers/1-s2.0-...main.pdf`
- Appl. Sci. paper: `docs/uwb-papers/applsci-14-06918.pdf`
- Qorvo APS011 (TWR errors): `docs/app-notes/Qorvo_APS011_TWR_Error_Sources.pdf`
- Qorvo APS014 (antenna cal): `docs/app-notes/Qorvo_APS014_Antenna_Delay_Calibration.pdf`
