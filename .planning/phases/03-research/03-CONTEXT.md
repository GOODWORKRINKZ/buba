# Phase 3: BU04 Robot Following — Context & Decisions

**Date:** 2026-06-10
**Status:** Research complete — decisions locked for downstream planning

---

## 1. Phase Goal

Determine how many BU04 modules are needed, which operating modes, and the system architecture
for a robot that follows a human wearing a UWB tag. Produce research-backed recommendations
that downstream phases (planning, execution) can act on without re-investigation.

---

## 2. Key Decisions (LOCKED)

| # | Decision | Value | Rationale |
|---|----------|-------|-----------|
| D1 | **Module count** | **4× BU04** (3 anchors + 1 tag) | CN105828431A patent: 3-base-station trilateration gives 360° coverage with analytic solution |
| D2 | **Operating mode** | **TWR** (not PDOA) | PDOA = 120° only, needs line-of-sight. TWR trilateration = 360° via geometry |
| D3 | **Anchor arrangement** | Equilateral triangle, side a=30-50cm | Patent-optimized geometry; 3× BU04 directional 120° × 3 = 360° combined |
| D4 | **Co-processor** | **RP2040** | PIO = unlimited UARTs (3 inputs + 1 output), deterministic, 20mA, $4 |
| D5 | **BU04↔RP2040 interface** | **UART** (AT commands) | Stock firmware is UART-only; PIO provides 3+ UARTs for free |
| D6 | **RP2040→Robot output** | **UART** or **I2C** | Clean `TAG:L,θ` ASCII packets at 115200 baud |
| D7 | **Tag hardware** | BU04 + LiPo 500mAh + external omni antenna (IPEX) | No extra MCU needed; STM32 handles TWR responses autonomously |
| D8 | **"Follow me" button** | **Data-over-UWB** (no extra radio) | DW3000 `dwt_writetxdata()` supports up to 1023 bytes; SDK already has TX example |
| D9 | **Positioning algorithm** | **Analytic (patent CN105828431A)** | Closed-form, no Kalman needed for initial version; ±10cm claimed accuracy |
| D10 | **Purchase** | Buy **2× BU04** (~$30 Taobao) | Already have 2; need 4 total |

---

## 3. Architecture Overview

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
  BU04 + LiPo + omni antenna (IPEX) + button (GPIO)
  TWR responses: automatic (stock firmware)
  Button: GPIO → dwt_writetxdata("BTN1") → UWB data frame
```

---

## 4. Patent Formula (CN105828431A)

3 base stations A, B, C in equilateral triangle (side = a).
Tag M at unknown position. Measured distances: d1, d2, d3.

```
L = √((d1⁴ + d2⁴ + d3⁴ - d1²d2² - d1²d3² - d2²d3²) / (3·a²))
θ = arctan( √3(d3² - d2²) / (-2d1² + d2² + d3²) )

Output: (L, θ) — polar coordinates from robot center
Accuracy: ±10cm (patent claim)
```

Coordinate system: O(0,0,0) = midpoint AB, y-axis = robot forward, H = triangle center.

[Source: CN105828431A patent, University of Shanghai for Science and Technology, 2016]

---

## 5. BU04 Antenna Orientation

- Two PCB antennas on opposite edges (RF1, RF2) — used for PDOA phase difference
- Radiation pattern: directional, ~120° beam (±60°), back side = ~0%
- For TWR trilateration: mount each BU04 with antenna edge facing OUTWARD from triangle
- Mounting: plastic bracket ≥5cm above metal; antenna area must overhang or have cutout below
- Tag: replace PCB antenna with external omni via IPEX connector (~$2)

[Source: BU04 Specification V1.0.0, Ai-Thinker, 2024 — `/buba/bu04_v1.0.0_specification-20240801.pdf`]

---

## 6. Data-over-UWB for Commands

Stock AT firmware doesn't expose data TX, but SDK makes it trivial to add:

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

DW3000 API: `dwt_writetxdata(len, buf, offset)` — up to 1023 bytes at 6.8 Mbps.
Existing SDK example: `ex_03a_tx_wait_resp` sends "DECAWAVE" over UWB.

[Source: STM32F103-BU0x_SDK, `deca_device_api.h`, `cmd_fn.c`, `ex_03a_tx_wait_resp.c`]

---

## 7. Academic Sources

| Paper | Key Finding | DOI/Source |
|-------|-------------|------------|
| **CN105828431A** (2016) | 3-base-station analytic trilateration, ±10cm | `CN105828431A.pdf` |
| **ETH UWBTracker** (Nägeli et al.) | 4 UWB on drone + IEKF, 10cm @ 4m, handles occlusion | `UWBTracker.pdf` |
| **IFAC 2024** (Janousek et al.) | UWB + RGB-D camera hybrid, ROS mobile robot | `1-s2.0-S2405896324004932-main.pdf` |
| **Appl. Sci. 2024** (Kwon et al.) | UWB for polar robots — works where cameras fail | `applsci-14-06918.pdf` |

---

## 8. Hardware BOM

| Item | Qty | Unit Price | Total | Link |
|------|-----|-----------|-------|------|
| BU04 module | 2 (buy) | ~$15 | ~$30 | Taobao |
| BU04 module | 2 (have) | — | $0 | — |
| RP2040 board | 1 | ~$4 | ~$4 | AliExpress |
| LiPo 1S 500mAh | 1 | ~$5 | ~$5 | — |
| IPEX omni antenna | 1 | ~$2 | ~$2 | Taobao |
| TP4056 charger | 1 | ~$1 | ~$1 | — |
| **TOTAL** | | | **~$42** | |

---

## 9. Deferred / Out of Scope

- Kalman filter on RP2040 (do after basic trilateration works)
- Room-fixed anchors for absolute positioning (Phase 6+)
- Multi-tag support (Phase 6+)
- ROS2 `biba_uwb_follow` node implementation (Phase 6+)
- BU04 custom STM32 firmware for I2C slave mode (not needed — UART works)

---

## 10. Next Steps

1. ✅ Research complete — `ROBOT-FOLLOWING.md` (600+ lines, all sources)
2. 🔜 **Phase 4: Calibration** — measure PDOA offsets, update `config.h`
3. 🔜 **Phase 5: Accuracy Verification** — 3×3 grid RMSE test
4. 🔜 **Phase 6: Robot Following Implementation** — buy 2× BU04, build RP2040 co-processor, flash custom AT+BUTTON firmware

---

## References

- Full research: `.planning/research/ROBOT-FOLLOWING.md`
- BU04 spec: `bu04_v1.0.0_specification-20240801.pdf`
- AT commands: `BU03_BU04_AT_command_cn_V1.0.6.pdf`
- SDK: `STM32F103-BU0x_SDK/`
- Patent: `CN105828431A.pdf`
- ETH paper: `UWBTracker.pdf`
- IFAC paper: `1-s2.0-S2405896324004932-main.pdf`
- Appl. Sci. paper: `applsci-14-06918.pdf`
