# Phase 2: Python Visualizer — Research

**Researched:** 2026-06-09
**Sources:** 
- BU04 Specification V1.0.0 (PDF)
- BU03/BU04 AT Command V1.0.6 (PDF)
- Ai-Thinker STM32F103-BU0x_SDK
- Project phase 1 research (PITFALLS.md, STACK.md, ARCHITECTURE.md)
- YouTube: "How to Set Up and Calibrate the AI Thinker BU03 UWB" (OpenELAB)
- Hackster.io: "Quick Start of UWB Module — Power-On Test" (Ai-Thinker)
- Ai-Thinker official page: en.ai-thinker.com/pro_view-159.html
- GitHub: Ai-Thinker-Open/STM32F103-BU0x_SDK (SDK source, driver docs)
- Ai-Thinker Feishu Wiki: https://fcniufr8ibx1.feishu.cn/wiki/space/7454451041846034460 (UWB tutorials)

---

## 1. Antenna Orientation & Placement

### Physical Setup
- **Boresight:** Anchor antenna plane must face the expected tag operating zone. 0° = straight ahead, perpendicular to antenna plane.
- **Height:** Both anchor and tag at **1.0–1.2 m** above floor. Consistent height during calibration AND operation.
- **Wall clearance:** Anchor ≥1.5 m from reflective walls (reduces multipath phase corruption).
- **Metal isolation:** Plastic bracket if on robot chassis. Metal surfaces cause ±15–30° angle errors.
- **Floor reflections:** Tag <0.5 m height → dominant floor multipath → angle degradation.

### PDOA Angle Limits
- **Safe operating range:** ±50° from boresight
- **Physical limit:** ±60° — beyond this, DW3000 PDOA saturates/wraps with NO warning flag
- **Silent failure mode:** Reports exactly ±60° indefinitely → robot "death spiral"

### Multipath Behavior
- **Range stays accurate** (ToF is robust), **angle degrades** (phase is sensitive)
- Error is position-dependent — works at one spot, fails at another
- High-reflectivity materials (aluminum, copper) are worst offenders

---

## 2. Accuracy Specifications

| Parameter | Uncalibrated | After Calibration | Target |
|-----------|-------------|-------------------|--------|
| Range | ±15 cm bias | ±5–10 cm | <10 cm RMSE |
| Angle (PDOA) | ±15–40° offset | ±5–15° | <15° RMSE |
| Update rate | ~4–5 Hz | — | ≥5 Hz |

### DW3000 Chip Specs
- Range accuracy: ±10 cm (datasheet)
- PDOA angle accuracy: ±5° at close range LOS
- BU04 module adds ~±5–10° due to PCB trace asymmetry

---

## 3. Calibration Methods

### Method A: PDOA Offset (our current approach)
```
AT+PDOAOFF=<degrees>    # angle correction (±90°)
AT+RNGOFF=<cm>          # range correction (±200 cm)
AT+SAVE
```
- **Angle:** Place tag at 0° (straight ahead), 1–1.5 m → collect 100 samples → mean = PDOAOFF
- **Range:** Place tag at exactly 1.000 m → collect 100 samples → correction = 100 - mean_cm = RNGOFF
- Our tool: `python3 tools/visualizer.py --calibrate`

### Method B: Linear Regression (TWR — from YouTube + Feishu Wiki)

**Official Ai-Thinker calibration procedure ("测距精度矫正"):**

1. Download template: `d_数据标定模板.xlsx` (calibration spreadsheet)
2. Set up equipment at measured distances (1m intervals, 15-20 points)
3. Record actual distance vs device-reported distance in spreadsheet
4. Spreadsheet computes linear regression: `actual = para_a * reported + para_b`
5. Apply correction via:
   ```
   AT+SETDEV=10,16336,1,0.018,0.642,<para_a>,<para_b>,0,0
   AT+SAVE
   ```
   Example: `AT+SETDEV=10,16336,1,0.018,0.642,0.9924,-317.68,0,0`

**AT+SETDEV parameters:**
| Param | Name | Description | Default |
|-------|------|-------------|---------|
| 1 | cap | Tag capacity / refresh slots | 10 |
| 2 | anndelay | Antenna delay (DW3000 calibration) | 16336 |
| 3 | kalman_enable | Kalman filter on/off | 1 |
| 4 | kalman_Q | Process noise covariance | 0.018 |
| 5 | kalman_R | Measurement noise covariance | 0.642 |
| 6 | **para_a** | **Correction slope** (from regression) | 1.0000 |
| 7 | **para_b** | **Correction intercept** (from regression) | 0.00 |
| 8 | pos_enable | Positioning enable | 0 |
| 9 | pos_dimen | Positioning dimensions | 0 |

**Note:** Change only para_a and para_b; keep other parameters at defaults.

### Method C: Kalman Filter Tuning
Current BU04 Kalman settings (from `AT+GETDEV`):
- `kalman_enable:1` — enabled
- `kalman_Q:0.018` — process noise (low = trusts motion model)
- `kalman_R:0.642` — measurement noise (high = distrusts raw measurements)
- Tune via `AT+SETDEV` for your specific environment

---

## 4. Multi-Device Accuracy Improvements

### TWR Mode (2 anchors + 1 tag)
- Triangulation from two fixed anchors with known baseline (20–50 cm)
- ESP-NOW wireless link between anchors for d₂ forwarding
- Better angle accuracy than PDOA (geometry-based, not phase-based)
- Requires 3 modules total

### PDOA Mode (1 anchor + 1 tag) — our current setup
- Single anchor with dual antennas
- Simpler hardware, phase-based angle measurement
- Accuracy limited by antenna spacing and calibration

### Future: Multi-Anchor PDOA
- Multiple PDOA anchors at known positions
- Each measures independent range+angle
- Combine via triangulation → better accuracy than single PDOA

---

## 5. Environmental Best Practices

### Official Ai-Thinker Guidelines (from Feishu Wiki "UWB 测距")

**Critical accuracy rules (directly from manufacturer):**

1. **Antenna clear zone (净空区域):** Keep the space between tag and anchor **completely unobstructed**. Any barrier causes large measurement errors or severe range reduction.
2. **DO NOT place on desk/table:** Placing anchor or tag on a desktop causes large errors. Elevate modules on non-conductive stands.
3. **Calibration:** After confirming (1) and (2), use calibration commands (`AT+PDOAOFF`, `AT+RNGOFF`) to further improve accuracy.

### Official TWR 2D Positioning Setup (from Feishu Wiki "TWR 二维定位测试")

**Physical requirements (CRITICAL):**
- **Anchor height: ≥1.5 m above ground**, vertically mounted on tripods
- **Open area with NO obstructions** — completely clear space
- Anchors placed at known real-world coordinates
- 3 anchors + 1 tag minimum for 2D positioning (triangulation)
- USB connection to anchor #0 for data output

**TWR vs PDOA comparison:**
| Aspect | TWR | PDOA |
|--------|-----|------|
| Min anchors | 3 (for 2D) | 1 |
| Position method | Triangulation (distances) | Phase difference (angle) |
| Angle accuracy | Geometry-based (better) | Phase-based (±5-15°) |
| Setup complexity | High (3 tripods, coordinates) | Low (1 anchor) |
| Range | 20m demonstrated | ~10m documented |

**HEX Protocol (TWR binary format):**
The BU04 outputs binary TWR frames over USB. Frame structure:
- Header: "CmdM:4" (6 bytes)
- Timer (4B), TagID (2B), AncID (2B), Seq (1B), Mask (1B)
- Raw ranges to 8 anchors (4B each)
- Kalman-filtered ranges to 8 anchors (4B each)
- Position: X(4B), Y(4B), Z(4B)
---

## 8. PDOA JSON Protocol — OFFICIAL SPEC (from Feishu Wiki "PDOA 跟随测试")

### Full field description:
```json
JS006C{"TWR": {
  "a16":"4096",    // Tag short address (hex)
  "R":115,         // Tag sequence number (wraps at 16-bit)
  "T":0,           // DW3000 timestamp
  "D":76,          // Distance in CM
  "P":-123,        // PDOA phase difference / angle (signed, degrees)
  "Xcm":-57,       // X coordinate in CM (from anchor perspective)
  "Ycm":50,        // Y coordinate in CM (from anchor perspective)
  "O":408,         // Clock offset (carrier frequency offset)
  "V":49152,       // Tag status info (bitfield)
  "X":0,           // Tag accelerometer X axis
  "Y":0,           // Tag accelerometer Y axis
  "Z":0            // Tag accelerometer Z axis
}}
```

### Angle calculation (from Xcm/Ycm):
```
angle = arctan(Xcm / Ycm) × 180 / π
```
Example: Xcm=14, Ycm=32 → angle ≈ 23.62°

**Note:** The `P` field in JSON IS the PDOA angle directly. The Xcm/Ycm formula is an alternative calculation.

### HEX protocol (binary format):
| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1B | head | 0x2A |
| 1 | 1B | len | Frame length |
| 2 | 1B | sn | Sequence number |
| 3 | 2B | Addr | Tag address |
| 5 | 4B | Angle | PDOA angle (signed LE, 2's complement if negative) |
| 9 | 4B | Distance | Distance in cm |
| 13 | 2B | usercmd | Status bitfield |
| 15 | 4B | F_Path | First path signal strength |
| 19 | 4B | RX_Level | Signal strength |
| 23 | 2B | Acc_X | Accelerometer X |
| 25 | 2B | Acc_Y | Accelerometer Y |
| 27 | 2B | Acc_Z | Accelerometer Z |
| 29 | 1B | Check | XOR checksum |
| 30 | 1B | Foot | 0x23 |

**Angle decoding (HEX):** Positive = direct decimal. Negative = 2's complement.
Example: `FB FF FF FF` (little-endian) → `FF FF FF FB` → 2's complement → -5°.

### PDOA setup (official):
1. Configure anchor: `AT+SETCFG=0,1,1,1` + `AT+SETUWBMODE=1` + `AT+SAVE`
2. Configure tag: `AT+SETCFG=0,0,1,1` + `AT+SETUWBMODE=1` + `AT+SAVE`
3. USB to BU04-Kit for data
4. Method A: PDOA upper computer → auto tag discovery
5. Method B: `AT+ADDTAG=<tagID>,8834,1,64,0` + `AT+SAVE` (manual, persists)

### BU04 antenna:
- **Type:** DIRECTIONAL (定向天线) — PCB antenna
- **Beam width:** 120° (±60°)
- **Forward direction:** Antenna points = forward
- **⚠️ Back side has NO signal (or very weak)!** Tag behind anchor = invisible.
- **Orientation:** VERTICAL, antenna plane facing tag
- **External antenna:** Supports IPEX connectors (requires resistor rework)
- **Calibration:** Tag on centerline, >1m distance
- **Role:** Best as PDOA anchor (base station)

### BU03 antenna (comparison):
- **Type:** OMNI-DIRECTIONAL (全向天线) — ceramic chip antenna
- **Best practice:** Place antenna outside PCB edge; surrounding copper/components degrade accuracy
- **Role:** Best as PDOA tag/beacon (no orientation worries)

### UserCmd bitfield (`V` field):
```c
typedef struct {
    uint16_t is_lowbattery:1;  // Low battery
    uint16_t is_alarm:1;       // Button alarm
    uint16_t reserver:14;
} user_cmd_t;
```

### Additional Best Practices

1. **LOS is critical** — any obstruction between anchor and tag ruins PDOA angle
2. **Consistent height** — different heights between calibration and operation = accuracy loss
3. **Antenna orientation** — both anchor antennas must be parallel to each other, perpendicular to tag direction
4. **Power stability** — VDDAON jumper (already applied), clean 3.3V supply
5. **Avoid Wi-Fi/Bluetooth interference** — UWB channel 5 (6.5 GHz) is away from 2.4/5 GHz Wi-Fi, but keep other RF sources at distance
6. **Temperature** — DW3000 has temperature-dependent group delay; re-calibrate if environment temperature changes >10°C

---

## 6. AT Commands Quick Reference (from spec V1.0.6 + video)

### Configuration
| Command | Purpose |
|---------|---------|
| `AT+SETCFG=ID,Role,CH,Rate` | Set device ID, role (0=tag,1=anchor), channel, rate |
| `AT+SETUWBMODE=0\|1` | 0=TWR, 1=PDOA |
| `AT+SAVE` | Save to flash (triggers reboot) |
| `AT+RESTART` | Software restart |

### Calibration
| Command | Purpose |
|---------|---------|
| `AT+PDOAOFF=<deg>` | PDOA angle offset (±90°) |
| `AT+RNGOFF=<cm>` | Range offset (±200 cm) |
| `AT+SETDEV=...,a,b,...` | Linear correction (TWR): a=scale, b=offset |
| `AT+FILTER=0\|1` | Enable/disable motion filter |

### Operation
| Command | Purpose |
|---------|---------|
| `AT+DECA$` | PDOA node handshake |
| `AT+GETDLIST` | Get discovered tag list |
| `AT+GETKLIST` | Get known (paired) tag list |
| `AT+ADDTAG=addr64,addr16,fast,64,mode` | Register tag |
| `AT+DELTAG=addr64` | Remove tag |
| `AT+USER_CMD=0\|1` | Output format: 0=JSON, 1=Hex |

---

## 7. Video Transcript — Key Points

From OpenELAB's "How to Set Up and Calibrate AI Thinker BU03 UWB with AT Commands" (3:15):

1. **Basic config:** `AT+SETCFG` → `AT+SAVE` (for both tag and anchor)
2. **Calibration tool:** Official Excel template for correction coefficient calculation
3. **Procedure:** Record actual vs reported distance at multiple ranges → linear regression → `para_a` and `para_b`
4. **Apply:** `AT+SETDEV=...,para_a,para_b,...` → `AT+SAVE`
5. **Result:** "With calibration, measurements are more accurate"

The video covers TWR calibration specifically, but the linear regression method can be adapted for PDOA range correction as an alternative to simple RNGOFF.
