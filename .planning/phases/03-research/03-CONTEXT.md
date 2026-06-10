# Phase 3: BU04 Robot Following — Context & Decisions

**Date:** 2026-06-10
**Status:** Discuss complete → ready for planning

---

## 1. Phase Goal

Determine how many BU04 modules are needed, which operating modes, and the system architecture
for a robot that follows a human wearing a UWB tag. Produce research-backed recommendations
that downstream phases (planning, execution) can act on without re-investigation.

---

## 2. Research Methodology (HOW we search)

### Search engines & queries

| Engine | Query pattern | Why |
|--------|---------------|-----|
| **DuckDuckGo** | `DW3000 follow robot UWB tag tracking` | Best results for practical projects, no filter bubble |
| **DuckDuckGo** | `UWB trilateration robot following 3 anchors` | Academic + patent results |
| **DuckDuckGo** | `BU04 BU03 Ai-Thinker DW3000 project tutorial` | Module-specific usage |
| **Google Patents** | `UWB trilateration robot following base station status:GRANT` | ~220 patents, filter by cited |
| **GitHub topics** | `uwb-positioning`, `dw3000` | Filter: has README, has demo, stars > 5 |
| **Qorvo/Decawave** | Direct PDF download of APS notes | Primary source for TWR errors, antenna cal |
| **Vendor forums** | forum.qorvo.com, forum.arduino.cc | Real engineers, real problems |

### Filtering criteria

| Keep | Skip |
|------|------|
| Has code (GitHub) OR has numbers (paper) OR has product (commercial) | Blog post with no data |
| Published ≥ 2018 (DW3000 era) | DW1000-only unless concept is transferable |
| Cites specific accuracy (±X cm) | "We achieved good results" |
| Open-access PDF | Paywalled unless critical |

### What we DON'T search (waste of time)

- Reddit, Hackaday — opinions, no data
- YouTube — demos without code/methodology
- arXiv generic queries — "UWB tracking robot" → 0 results
- IEEE Xplore direct — blocked, use Google Scholar/DuckDuckGo as proxy

### PDF organization

```
docs/
├── datasheets/          ← Vendor specs, pinouts, AT commands
├── patents/             ← Patent PDFs
├── uwb-papers/          ← Academic papers
└── app-notes/           ← Qorvo APS/APH application notes
```

### What to search NEXT (for planning phase)

1. Download Qorvo DW3000 User Manual (the ~200pp reference)
2. Fetch `orisharabi/unitree-go2-follow-system` — real UWB robot following code
3. Read APS011 fully — extract DS-TWR formula validation
4. Check if BU03 vs BU04 have same AT firmware (SDK comparison)
5. Search: `"component-wise error correction" UWB target following` — paper PMC8838499

---

## 3. Collected Materials Inventory

### PDFs collected (14 files, ~12MB)

| File | Pages | Source | Key content |
|------|-------|--------|-------------|
| `datasheets/bu04_v1.0.0_specification-20240801.pdf` | 21 | Ai-Thinker | Pinout, antenna, mounting rules |
| `datasheets/bu04_v1.0.0规格书20240801.pdf` | 21 | Ai-Thinker (CN) | Same, Chinese |
| `datasheets/BU03_BU04_AT_command_cn_V1.0.6.pdf` | ~30 | Ai-Thinker | All AT commands |
| `patents/CN105828431A.pdf` | ~15 | Google Patents | 3-base-station analytic formula |
| `uwb-papers/UWBTracker.pdf` | 8 | ETH Zurich | 4-UWB on drone, IEKF, 10cm |
| `uwb-papers/1-s2.0-...main.pdf` | 6 | IFAC 2024 | UWB + RGB-D camera hybrid |
| `uwb-papers/applsci-14-06918.pdf` | 15 | MDPI 2024 | Polar robot UWB following |
| `uwb-papers/UWB_Observer_Based_Human_Following.pdf` | ~10 | Cloudfront | Observer-based control |
| `app-notes/Qorvo_APS011_TWR_Error_Sources.pdf` | 22 | Qorvo | **THE source** on TWR errors |
| `app-notes/Qorvo_APS014_Antenna_Delay_Calibration.pdf` | ~10 | Qorvo | Antenna delay tuning |
| `app-notes/Qorvo_APS017_Maximizing_Range.pdf` | ~10 | Qorvo | Range optimization |
| `app-notes/Qorvo_APH301_DW3000_HW_Design.pdf` | ~15 | Qorvo | Hardware design guide |

### Online resources (linked in ROBOT-FOLLOWING.md)

| Resource | What |
|----------|------|
| Circuit Digest tutorial | Full ESP32+DWM3000 DS-TWR code, Python viz |
| KunYi/esp32-uwb-positioning | GitHub: 2-10 anchors, web viz, simulator |
| unitree-go2-follow-system | Real Unitree Go2 UWB following code |
| Fhilb/DW3000_Arduino | Arduino DW3000 DS-TWR library |
| OpenELAB BU03 guide | BU03 vs BU04, TWR vs PDOA comparison |
| Pozyx technology | Commercial UWB RTLS reference |
| Makerfabs DW3000 examples | Reference ESP32 implementation |
| Qorvo forum (AndyA) | Tag/anchor architecture insights |

---

## 4. What Planning Needs (handoff)

### Decisions already LOCKED (do not re-discuss):
- D1-D10 above are final

### Gray areas for planner to resolve:
| # | Question | Hint |
|---|----------|------|
| G1 | Triangle side **a** = ? | 30cm fits small robot, 50cm = better geometry |
| G2 | RP2040 poll rate? | 4Hz minimum, 10Hz desired |
| G3 | Output protocol? | `TAG:L,θ` ASCII vs binary 4-byte |
| G4 | 2-of-3 anchors lost → continue or stop? | Geometry degrades, but can estimate |
| G5 | TWR per-module calibration needed? | APS014 describes procedure |
| G6 | Tag battery indicator? Auto-sleep? | ~10h with 500mAh LiPo |
| G7 | Kalman on RP2040 or robot? | Start without, add if noisy |

### Files planner should read:
1. `03-CONTEXT.md` (this file) — all decisions
2. `.planning/research/ROBOT-FOLLOWING.md` — full research with sources
3. `docs/app-notes/Qorvo_APS011_TWR_Error_Sources.pdf` — TWR error budget
4. `docs/patents/CN105828431A.pdf` — the formula
5. `STM32F103-BU0x_SDK/Components/APP/cmd_fn.c` — AT command extension point

---

## 5. Key Decisions (LOCKED)

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

## 6. Architecture Overview

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

## 7. Patent Formula (CN105828431A)

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

## 8. BU04 Antenna Orientation

- Two PCB antennas on opposite edges (RF1, RF2) — used for PDOA phase difference
- Radiation pattern: directional, ~120° beam (±60°), back side = ~0%
- For TWR trilateration: mount each BU04 with antenna edge facing OUTWARD from triangle
- Mounting: plastic bracket ≥5cm above metal; antenna area must overhang or have cutout below
- Tag: replace PCB antenna with external omni via IPEX connector (~$2)

[Source: BU04 Specification V1.0.0, Ai-Thinker, 2024 — `/buba/bu04_v1.0.0_specification-20240801.pdf`]

---

## 9. Data-over-UWB for Commands

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
