# UWB Следование за человеком — Комплексное исследование

**Дата:** 2026-06-10
**Источники:** DuckDuckGo (20+ запросов), Google Patents (7 патентов), GitHub (11 репозиториев), Qorvo Forum, CSDN, Zhihu, Bilibili, ResearchGate, arXiv, IEEE, MDPI, 4 академические статьи
**Язык:** Русский

---

## 0. Academic Papers — Key Findings

### 0.1 ETH Zurich — Omni-directional Person Tracking on Flying Robot (UWBTracker)
**Nägeli, Hepp, Hilliges — ETH Zurich AIT Lab**

**The Gold Standard for On-Robot UWB Tracking:**
- **4 UWB receivers** on quadrotor + **1 tag** on target
- **No environment instrumentation** — all receivers on robot
- **Trilateration** with TOA (not TDOA) — better for small baseline
- **Active-Passive Ranging Algorithm** — measurement rate independent of unit count
- **IEKF** (Iterated Extended Kalman Filter) for position estimation
- **Result: 10cm average position error** for 4m×4m square
- **Handles occlusion** — UWB penetrates obstacles that block cameras
- **Omnidirectional** — no limited field of view
- Follows person via **path mapping** (implicit obstacle avoidance)

**Key Architecture:**
```
Tracker (robot): 4× UWB in rigid config (master + 3 listeners)
Target (human):  1× UWB slave
Ranging: Active-Passive (master↔slave active, listeners passive sniff)
Filter:   IEKF on 3D position
Control:  Follow mapped path (not shortest line)
```

### 0.2 CN105828431A — UWB Autonomous Following Robot (Patent)
**University of Shanghai for Science and Technology (2016)**

**THE KEY REFERENCE — Exact Geometric Solution with 3 Base Stations:**

**Setup:**
- **3 UWB base stations** on robot in **equilateral triangle** (side = a)
- **1 UWB tag** on human (wrist, waist, or neck)
- TOA algorithm for distance measurement

**Closed-Form Analytic Solution (no iteration needed!):**
```
Given: d1, d2, d3 = distances from 3 base stations A, B, C to tag M
       a = side length of equilateral triangle ABC

Step 1 — Tag-to-center distance L:
  L = M'H = √((d1⁴+d2⁴+d3⁴ - d1²d2² - d1²d3² - d2²d3²) / (3·a²))

Step 2 — Azimuth angle θ (from robot's forward axis):
  θ = ∠M'HA = arctan( √3(d3² - d2²) / (-2d1² + d2² + d3²) )

Output: (L, θ) — polar coordinates of target relative to robot center
Accuracy: ±10cm
```

**Coordinate System:**
- O(0,0,0) = midpoint of AB side
- y-axis = forward direction of robot (⊥ BC, through H)
- H = center of equilateral triangle
- z = tag height above base station plane
- M'(x,y) = projection of tag onto base station plane

**Advantages claimed:**
- No cumulative error (each measurement independent)
- Low computation (analytic, not iterative)
- Combines positioning + communication in one
- Indoor AND outdoor capable
- ±10cm accuracy

**This is directly implementable with 3× BU04 on our robot!**

### 0.3 IFAC 2024 — Target-Following Robot with UWB + Depth Camera
**Janousek, Slanina, Walendziuk — VSB-TU Ostrava + Bialystok University**

**Hybrid approach:**
- UWB for **long-range person detection** (no FOV limits)
- RGB-D camera for **close-range precision** + obstacle detection
- UWB solves the "limited FOV" problem of cameras
- Camera solves the "limited accuracy" problem of UWB
- ROS-based mobile robot platform

### 0.4 Appl. Sci. 2024 — UWB Human-Following for Polar Exploration Robots
**Kwon, Lee, Lee, Lee, Kim, Uhm, Choi — KIRO Korea**

**Why UWB for extreme environments:**
- Cameras/LiDAR **fail** in: strong sunlight, snow glare, fog, polar conditions
- UWB **works regardless** of weather/lighting
- UWB triangulation for operator position
- Integrated with **obstacle + crevasse avoidance** path planning
- Real-time local obstacle mapping
- Validated in simulation + real polar experiments

**Key insight:** UWB isn't just "cheap GPS alternative" — it's the ONLY reliable option when cameras fail due to environmental conditions.

---

## 1. Reference Projects

### 1.1 Makerfabs AOA Development Kit (STM32F103 + DW3000)
**Repo:** [Makerfabs/UWB-AOA-with-Display-STM32F103C8T6](https://github.com/Makerfabs/UWB-AOA-with-Display-STM32F103C8T6)

**Key specs (same hardware as BU04!):**
| Parameter | Value |
|-----------|-------|
| Coverage radius | **30m @ 6.8Mbps** |
| Angle (AOA) | **±60°** |
| Angle error | **±5°** |
| Ranging error | **<10cm** |
| Positioning error | **<10cm** |
| Chip | STM32F103C8T6 + DW3000 |
| Frequency | CH5 (6.5 GHz) / CH9 (8 GHz) |
| Interface | SPI (between MCU and DW3000) |

**Architecture:**
- Anchor: UWB-X3-AOA module (DW3000 with dual antennas)
- Tag: UWB-X3-MAX module (with PA — Power Amplifier for extended range)
- QT-based AOA System software for PC visualization
- OLED display for standalone operation
- Open-source firmware (STM32CubeIDE)

**Tag PA modification (for extended range):**
```c
// In deca_vals.h:
#define PMSC_TXFINESEQ_DISABLE 0x0d20010  // Enable PA

// In config_options.h:
#define HW_PA  // Enable hardware PA support
```

### 1.2 Robot Chase Human-Path (fcaponetto)
**Repo:** [fcaponetto/robot-follow-path-uwb](https://github.com/fcaponetto/robot-follow-path-uwb)

**Approach:**
- Uses **DW1000** (older chip, predecessor to DW3000)
- Robot tank with 2 UWB modules
- **Particle filter** + statistics correlation matrix for position estimation
- Max range: **50m**
- Arduino + ST board, C/C++
- GeoGebra geometry models for path planning

**Key algorithm:**
```
1. Calculate distance between person and robot (ToF)
2. Detect polar coordinates (range + angle)
3. Apply particle filter for smoothing
4. Control wheels via follow-path algorithm
```

### 1.3 Pozyx-Based Human Following Robot
**Source:** [Arduino Forum](https://forum.arduino.cc/t/uwb-based-unmanned-indoor-human-following-robot-using-pozyx/632222)

**Setup:**
- Pozyx UWB system (commercial, expensive but well-documented)
- 4 anchors in room corners
- 1 tag on robot, 1 tag on human
- Robot calculates position relative to human
- P-controller for distance + angle

### 1.4 Makerfabs UWB Positioning (MaUWB ESP32S3)
**Repo:** [Makerfabs/MaUWB_ESP32S3-with-STM32-AT-Command](https://github.com/Makerfabs/MaUWB_ESP32S3-with-STM32-AT-Command)

**Features:**
- ESP32S3 + STM32 (AT command interface — same as BU04!)
- Supports **8 anchors + 64 tags**
- AT command configuration
- Python (pygame + pyserial) visualization
- 10m² test area with 4 anchors
- Indoor positioning via triangulation

---

## 2. Robot Following Architectures — Comparison

### Architecture A: Single PDOA Anchor (our current setup)
```
Robot: 1× BU04 (PDOA anchor)
Human: 1× BU04 (PDOA tag)

Output: distance + angle (±60°)
Pros: Simple, minimal hardware
Cons: Limited 120° coverage, ±5-15° angle error, no redundancy
```

### Architecture B: Dual TWR Anchor (on-robot triangulation)
```
Robot: 2× BU04 TWR anchors (known baseline B)
Human: 1× BU04 TWR tag

Output: (x,y) relative to robot
Pros: Full 240°, geometry-based angle (better accuracy)
Cons: Left/right ambiguity (±y), needs 3 modules total
```

### Architecture C: Multi-Anchor Room Positioning (Pozyx-style)
```
Room: 4× fixed anchors (corners)
Robot: 1× tag
Human: 1× tag

Output: absolute (x,y) for both robot and human
Pros: Full 360°, absolute positioning, no line-of-sight issues between robot and human
Cons: Room setup, 6 modules total, anchor placement critical
```

### Architecture D: AOA Array on Robot (Makerfabs approach)
```
Robot: 1× AOA anchor (dual antenna PDOA)
Human: 1× tag (with PA for extended range)

Output: distance + angle (±60°, ±5° error)
Pros: 30m range, ±5° accuracy, simple (2 modules)
Cons: Limited 120° coverage, tag needs PA
```

---

## 3. Control Algorithms

### 3.1 P-Controller (Simple, our target for v2)
```
speed = Kp_distance * (measured_distance - target_distance)
turn_rate = Kp_angle * measured_angle

Dead zones:
- Distance: ±10cm (no movement)
- Angle: ±10° (no turning)
```

### 3.2 Particle Filter (from fcaponetto project)
- Estimates true position from noisy UWB measurements
- Handles multi-path and outliers
- Computationally heavier but more robust

### 3.3 Kalman Filter (BU04 built-in)
- Already enabled: `kalman_enable:1, Q:0.018, R:0.642`
- 1D constant-velocity model
- Additional client-side Kalman in visualizer.py

---

## 4. Critical Design Decisions for Robot Following

### 4.1 Number of UWB Modules Needed

| Setup | Modules | Coverage | Angle Accuracy | Reliability |
|-------|---------|----------|----------------|-------------|
| 1 PDOA anchor | 2 | 120° front | ±5-15° | Low (easy to lose) |
| 2 TWR anchors | 3 | 240° | Geometry-based | Medium (± ambiguity) |
| 3 TWR anchors | 4 | 360° | Triangulation | High |
| Room anchors (4) | 6 | 360° absolute | Triangulation | Highest |

### 4.2 Antenna Constraints (from official docs)
- **BU04:** Directional, 120° beam, BACK = 0% signal
- **BU03:** Omnidirectional, but no PDOA (TWR only, no angle)
- **Antenna height:** ≥1.5m on tripods for best accuracy
- **Metal isolation:** Plastic bracket on robot chassis

### 4.3 Robot-Specific Challenges
1. **Metal chassis** → antenna detuning → plastic bracket mandatory
2. **Motor EMI** → can interfere with UWB (6.5 GHz is far from motor frequencies)
3. **Robot rotation** → anchor orientation changes → person "disappears"
4. **Body blocking** → person's body blocks UWB signal to tag
5. **Height mismatch** → robot anchor at 20cm, human tag at 1m → floor reflections

### 4.4 Recommended Mounting
```
Robot top view:
         FRONT (0°)
    ┌───────┬───────┐
    │ ⚓A1   │   ⚓A2 │  ← Two BU04 at 30cm spacing
    │ 120°  │  120° │     facing forward ±30°
    └───────┴───────┘
    
    Height: ≥15cm above robot chassis
    Material: Plastic/wood bracket, NOT metal
```

---

## 5. User's ROS2 Workspace

Found at `~/Downloads/biba/ros2_ws/src/biba_uwb_follow`:
- Existing ROS2 package for UWB following
- Robot description (URDF)
- Launch files
- Ready for integration with BU04 data

---

## 6. Key Performance Targets (from Makerfabs AOA Kit)

| Metric | Target | BU04 Capability |
|--------|--------|-----------------|
| Range | 30m | ~10m (tested) |
| Angle accuracy | ±5° | ±5-15° (before cal) |
| Range accuracy | <10cm | ±5-10cm |
| Update rate | ? | ~4 Hz |
| Coverage angle | ±60° | 120° beam |
| Tags supported | 64 | 1 (PDOA) / 10 (TWR) |

---

## 7. YouTube References

- [Makerfabs UWB AOA vs Bluetooth AOA](https://www.youtube.com/watch?v=5k938MZiHXY)
- [Additional UWB robot video](https://www.youtube.com/watch?v=aKHBPLJneuA)

---

## 8. Purchase Options

| Module | Price | Link |
|--------|-------|------|
| BU04 (Ai-Thinker) | ~$15 | Taobao |
| BU03-Kit | ~$25 | Taobao |
| MaUWB STM32 AOA Kit (Makerfabs) | $69.80 | makerfabs.com |
| MaUWB ESP32S3 | ~$20 | makerfabs.com |

---

## 9. Recommended Path Forward

### Phase 3-4 (Calibration + Accuracy):
1. Calibrate existing 2× BU04 setup (PDOAOFF, RNGOFF)
2. 3×3 grid accuracy test → validate ±10cm, ±15°

### Phase 5 (Robot Following v2):

**Option A — Minimal (2 modules, existing hardware):**
- 1 BU04 on robot (PDOA, forward-facing)
- 1 BU04 tag on human
- P-controller, robot actively turns to keep person in 120° cone
- Risk: easy to lose, only works when person is in front

**Option B — RECOMMENDED: 3-Base-Station Trilateration (CN105828431A patent):**
- **3× BU04 TWR anchors** on robot in equilateral triangle (side a ≈ 30-50cm)
- **1× BU04 TWR tag** on human (wrist/waist)
- **Total: 4 BU04 modules** (buy 2 more)
- **Algorithm** — direct analytic solution (no Kalman needed!):

```python
def solve_tag_position(d1, d2, d3, a):
    """CN105828431A patent — 3 base station equilateral triangle.
    d1,d2,d3 = distances from A,B,C to tag
    a = triangle side length (meters)
    Returns: (L, theta) — range and azimuth from robot center
    """
    import math
    
    # Distance from tag projection to triangle center
    L_sq = (d1**4 + d2**4 + d3**4 
            - d1**2 * d2**2 - d1**2 * d3**2 - d2**2 * d3**2) / (3 * a**2)
    L = math.sqrt(max(0, L_sq))
    
    # Azimuth from robot forward (y-axis)
    numerator = math.sqrt(3) * (d3**2 - d2**2)
    denominator = -2*d1**2 + d2**2 + d3**2
    theta = math.atan2(numerator, denominator)
    
    return L, theta
```

- **Pros:** Full 360° coverage, analytic (no iteration), ±10cm, proven in patent
- **Cons:** Needs 4 modules total, equilateral triangle geometry constrains mounting

**Option C — ETH Zurich 4-Receiver IEKF (UWBTracker paper):**
- 4× UWB on robot + 1 tag
- Active-passive ranging for high update rate
- IEKF for robust 3D tracking
- 10cm accuracy @ 4m coverage
- Best for: flying robots, complex occlusion environments

**Option D — Full Room Infrastructure:**
- Room-fixed anchors for absolute positioning
- Robot knows its position AND human position independently
- Best for complex environments but requires setup

---

## 10. Implementation Blueprint — Option B (3× BU04 Trilateration)

### Hardware:
```
Robot top view (equilateral triangle):
         FRONT (0°)
            ▲
           /|\
          / | \
         /  |  \
        A---+---B    ← A,B,C = BU04 TWR anchors
         \  |  /        side a = 40cm
          \ | /
           \|/
            ▼
            C
    y-axis = forward
    O = midpoint of AB
    H = triangle center
```

### BU04 Configuration:
- All 3 anchors: `SETCFG=0,0,5,2` (Anchor mode, CH5, 6.8M)
- Tag: `SETCFG=0,1,5,2` (Tag mode)
- TWR mode, not PDOA — we calculate angle from trilateration

### Data Flow:
```
BU04_A ──UART──┐
BU04_B ──UART──┤── ESP32/STM32 bridge ──USB── ROS2 node
BU04_C ──UART──┘       ↓
                   d1,d2,d3 → analytic_solver(L,θ) → cmd_vel
```

### ROS2 Node (biba_uwb_follow):
```python
class UWBTrilaterationFollower:
    def __init__(self):
        self.a = 0.40  # triangle side in meters
        self.target_distance = 1.5  # desired following distance
        
    def range_callback(self, d1, d2, d3):
        L, theta = solve_tag_position(d1, d2, d3, self.a)
        
        # P-controller
        distance_error = L - self.target_distance
        linear_vel = self.Kp_dist * distance_error
        angular_vel = self.Kp_angle * theta
        
        # Dead zones
        if abs(distance_error) < 0.10:  # ±10cm
            linear_vel = 0
        if abs(theta) < math.radians(10):  # ±10°
            angular_vel = 0
            
        self.publish_cmd_vel(linear_vel, angular_vel)
```

### Accuracy Budget:
| Source | Error |
|--------|-------|
| BU04 TWR ranging | ±5-10cm |
| Geometric dilution (a=40cm, L=1-5m) | ±5-15cm |
| Patent claimed total | **±10cm** |
| Angular (at L=2m, ±10cm) | **±3°** |
| Angular (at L=5m, ±10cm) | **±1°** |

### Buy List:
- 2× BU04 (~$30 total from Taobao)
- Already have 2 BU04 → total 4 modules

---

## 11. RP2040 UWB Co-Processor Architecture (2026-06-10)

### Why a dedicated co-processor?

Instead of connecting 3× BU04 directly to the robot controller (which wastes cycles polling 3 UARTs and computing trilateration), we use an **RP2040 as a dedicated UWB co-processor**:

```
                  ┌──────────────────────────────────────┐
                  │         RP2040 (UWB Co-Processor)     │
                  │                                      │
BU04-A ──UART──→ │ UART0 (HW)  ┐                        │
BU04-B ──UART──→ │ UART1 (HW)  ├─ trilateration ──→ UART │──→ ROBOT
BU04-C ──UART──→ │ PIO UART    ┘   CN105828431A     I2C  │    controller
                  │              + Kalman filter    SPI   │    (ROS2/STM32)
                  │              + outlier reject         │
                  └──────────────────────────────────────┘

INPUT:  3× raw TWR distances (d1, d2, d3) via AT commands
CORE:   Patent analytic formula → (L, θ) → (x, y)
OUTPUT: Single clean position packet to robot
```

### RP2040 interfaces:

| Direction | Interface | Pins | Notes |
|-----------|-----------|------|-------|
| IN: BU04-A | UART0 (HW) | GP0/GP1 | 115200-921600 baud, AT commands |
| IN: BU04-B | UART1 (HW) | GP4/GP5 | 115200-921600 baud, AT commands |
| IN: BU04-C | PIO UART | GP8/GP9 | RP2040 PIO = unlimited UARTs |
| OUT: Robot | UART (PIO) | GP12/GP13 | Clean position output |
| OUT: Robot | I2C (HW) | GP14/GP15 | Alternative if robot uses I2C |
| DBG: USB | USB CDC | GP16/GP17 | Debug/programming + serial monitor |

### Why not I2C for BU04?

BU04 stock firmware is **UART-only** for AT commands. I2C hardware exists on PB6/PB7
but firmware does not listen on it. No SPI slave mode either.
Options to use I2C would require:
- Writing custom STM32 firmware (SWD on pins 30-31) — massive effort
- Using an external UART→I2C bridge — adds complexity, no benefit over PIO UART

**Verdict: UART via PIO is the practical path.**

### Output protocol to Robot:

```
TAG:150,25\r\n      — tag at 150cm, +25° azimuth
TAG:120,0\r\n       — tag at 120cm, straight ahead
TAG:LOST\r\n         — all 3 anchors lost the tag
TAG:NOISE\r\n        — measurements too noisy, hold position
TAG:250,45,0.8\r\n   — 250cm, 45°, confidence 0.8
```

Simple ASCII, 115200 baud, robot parses in one `scanf` call.

### Why RP2040 (not ESP32)?

| Feature | RP2040 | ESP32 |
|---------|--------|-------|
| PIO (unlimited UART) | ✅ Yes | ❌ No (3 HW UART max) |
| Deterministic timing | ✅ No RTOS | ❌ FreeRTOS jitter |
| Power consumption | ~20mA | ~80mA+ |
| Price | ~$4 | ~$5 |
| I2C slave mode | ✅ PIO | ❌ HW only |
| USB host | ❌ Limited | ❌ Limited |
| WiFi/BT | ❌ No | ✅ Yes |

RP2040 wins for this: PIO = unlimited UARTs, deterministic timing for trilateration,
and we don't need WiFi on the co-processor (robot already has ROS2 communication).

---

## 12. Tag Side — What Does the Tag BU04 Need?

### Tag = BU04 in TAG mode — minimal hardware

The tag is remarkably simple. It only needs to **respond to TWR pings** from the 3 anchors.
No computation, no sensors, no display.

```
            BU04 TAG (on person)
┌──────────────────────────────────┐
│                                  │
│  BU04 (TAG mode, Role=1)         │
│  SETCFG=0,1,5,2                 │
│                                  │
│  Power: 3.3V battery             │
│  ┌──────────┐                    │
│  │ LiPo 1S  │──→ LDO 3.3V       │
│  │ 500mAh   │                    │
│  └──────────┘                    │
│                                  │
│  ┌─ Optional ─────────────────┐  │
│  │ LED: power/status          │  │
│  │ Button: on/off             │  │
│  │ Buzzer: lost warning       │  │
│  │ Charger: TP4056 USB-C      │  │
│  └────────────────────────────┘  │
│                                  │
└──────────────────────────────────┘
```

### Antenna consideration — directional vs omni:

| Tag antenna | Pros | Cons |
|-------------|------|------|
| BU04 PCB antenna (120°) | +3dB gain = more range | Person must face robot |
| BU04 + IPEX omni antenna | Full 360°, ~$2 external | Slight gain loss |
| BU03 (omni, separate module) | 360° out of box | Different module, different firmware |

**Recommendation: BU04 + external omni antenna via IPEX connector.**
BU04 supports IPEX external antenna (spec section 1.1: "支持板载天线，兼容 IPEX 座外接天线").
An omni rubber duck antenna costs ~$2 on Taobao. Best of both worlds:
unified hardware (4× BU04) with omni coverage on the tag.

### Does the tag need an extra MCU? **No.**

BU04 has STM32F103 built-in. The AT firmware handles all UWB protocol — TWR responses are automatic.
No external MCU needed on the tag. Just power it and it works.

### Tag power budget:

| Component | Current | 
|-----------|---------|
| BU04 TX active | ~150mA peak |
| BU04 RX/idle | ~30mA |
| LED | ~5mA |
| **Total avg** | **~50mA** |

With 500mAh LiPo → **~10 hours** continuous operation.
With tag refresh rate 4Hz → could optimize for lower power with sleep between pings.

### Tag form factor options:
1. **Wristband** — patent recommends wrist; BU04 is 35×33mm, fits in a watch-sized case
2. **Belt clip** — larger battery, more stable orientation
3. **Neck lanyard** — easy to put on, but antenna orientation varies

### Tag vs Anchor — what's different:
- Tag runs in TAG mode (Role=1), anchors in ANCHOR mode (Role=0)
- Tag only responds, never initiates ranging
- Tag needs battery, anchors are robot-powered
- Tag antenna should be omni (or at least wide); anchor antenna is directional (120° per anchor, 3 combined = 360°)

---

## 13. Data-over-UWB: Sending Commands Through BU04 (2026-06-10)

### Key Finding: BU04 SDK supports arbitrary data TX/RX!

Analysis of `STM32F103-BU0x_SDK` reveals that DW3000 **fully supports sending custom
data frames** — up to 1023 bytes per frame at 6.8 Mbps. The stock firmware doesn't
expose this via AT commands, but the SDK makes it trivial to add.

### DW3000 Data TX API (from SDK `deca_device_api.h`):

```c
// Send arbitrary data (1-1023 bytes):
dwt_writetxdata(len, buffer, offset);   // Write payload to TX buffer
dwt_writetxfctrl(frameLen, offset, 0);  // Configure frame control
dwt_starttx(DWT_START_TX_IMMEDIATE);    // Transmit now

// Receive data:
dwt_readrxdata(rx_buffer, frame_len, 0); // Read received payload
```

**TX Modes available:**
| Mode | Description |
|------|-------------|
| `DWT_START_TX_IMMEDIATE` | Transmit now |
| `DWT_START_TX_IMMEDIATE \| DWT_RESPONSE_EXPECTED` | TX then auto-RX response |
| `DWT_START_TX_DELAYED` | Scheduled TX at precise timestamp |
| `DWT_START_TX_CCA` | TX only if channel is clear |

### Existing SDK Example (ex_03a_tx_wait_resp):

```c
// Already sends "DECAWAVE" over UWB — proof it works!
static uint8_t tx_msg[] = {0xC5, 0, 'D','E','C','A','W','A','V','E', 0x43, 0x02, 0, 0};

dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
dwt_writetxfctrl(sizeof(tx_msg), 0, 0);
dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
```

### AT Command Framework — Add Custom Commands in 3 Lines

In `cmd_fn.c`, the command table `known_commands[]` is trivially extensible:

```c
// 1. Write handler function:
int f_send_btn(int opt, int argc, char* argv[]) {
    uint8_t data[] = {0xC5, 0, 'B', 'T', 'N', (uint8_t)atoi(argv[0])};
    dwt_writetxdata(sizeof(data), data, 0);
    dwt_writetxfctrl(sizeof(data), 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    return 0;  // 0 = success → "OK", -1 = error → "ERROR"
}

// 2. Register in known_commands[]:
//    Query:  {"AT+CMD", handler}  → AT+CMD? calls handler(QUERY_CMD, ...)
//    Execute: {"AT+CMD", handler} → AT+CMD  calls handler(EXECUTE_CMD, ...)
//    Set:     {"AT+CMD", handler} → AT+CMD=val calls handler(SET_CMD, ...)
{"AT+BUTTON", f_send_btn},  // ← one line to add!
```

### "Follow Me" Button — Complete Architecture Over UWB Only:

```
ТЕГ BU04 (на человеке):               АНКЕР BU04 (на роботе):
┌──────────────────────────┐         ┌──────────────────────────┐
│ PB6 ← кнопка (GPIO)      │         │                          │
│ Прерывание по нажатию:   │         │ RX-прерывание:           │
│  → dwt_writetxdata(...)  │  UWB    │  → dwt_readrxdata(...)   │
│  → dwt_starttx(...)      │────────→│  → buffer = "BTN1"      │
│                          │  DATA   │  → UART TX("BTN:1\r\n") │
│ (UWB TWR продолжается    │         │                          │
│  параллельно!)           │         │ RP2040 получает команду  │
└──────────────────────────┘         └──────────────────────────┘

Без HC-12, без BLE, без дополнительного радио!
UWB = позиция + команды в одном флаконе.
```

### Command Protocol (proposed):

| Command | UWB Payload | Meaning |
|---------|-------------|---------|
| Button Follow | `BTN1` | Start following |
| Button Stop | `BTN0` | Stop following |
| Button Faster | `BTN+` | Increase speed |
| Button Slower | `BTN-` | Decrease speed |
| Emergency Stop | `BTN!` | Immediate halt |
| Heartbeat | `HB##` | Tag battery/status |

### Why This Is Better Than HC-12/BLE:

| Approach | Extra HW | Range | Latency | Complexity |
|----------|----------|-------|---------|------------|
| HC-12 (433MHz) | $2 ×2 | 1km | ~50ms | Extra wiring |
| BLE module | $3 ×2 | 10m | ~30ms | Pairing, stack |
| **UWB data frame** | **$0** | **= TWR range** | **<5ms** | **3 lines in cmd_fn.c** |

UWB data is literally free — the radio is already transmitting. Adding a few bytes
to the payload costs nothing in hardware, range, or update rate.

### Implementation Effort:

1. **Tag firmware** (`cmd_fn.c`): Add `AT+BUTTON` handler + GPIO interrupt → ~20 lines
2. **Anchor firmware** (`cmd_fn.c`): Add RX handler that forwards to UART → ~15 lines
3. **RP2040 firmware**: Parse `BTN:1` from anchor UART → ~10 lines
4. **Robot controller**: React to button commands → existing

**Total: ~45 lines of C + 10 lines of Python. No new hardware.**

# UWB Следование за человеком — Комплексное исследование

**Дата:** 2026-06-10
**Источники:** DuckDuckGo (20+ запросов), Google Patents (7 патентов), GitHub (11 репозиториев), Qorvo Forum, CSDN, Zhihu, Bilibili, ResearchGate, arXiv, IEEE, MDPI
**Язык документа:** Русский

---

## 14. Результаты поиска DuckDuckGo (2026-06-10)

**Поиск выполнен:** 10 июня 2026, DuckDuckGo (без пузыря фильтров) + Google Patents + GitHub topics + Qorvo Forum
**Всего запросов:** 20+ по 3 группам + китайский поиск + патентный поиск
**Найдено:** 30+ стоящих результатов | **Пропущено:** ~50+ (блоги без данных, DW1000-only, paywall, Reddit/Hackaday/YouTube)

**Выполненные запросы:**
| Группа | Запросов | Примеры запросов |
|--------|---------|-----------------|
| A: DW3000 документация | 6 | "DW3000 User Manual PDF Qorvo", "DW3000 register map API dwt_configure", "DW3000 datasheet specifications" |
| B: UWB роботы-следователи | 7 | "UWB robot following human tracking github", "DW3000 TWR robot follow tag", "UWB follower robot", "UWB follow me robot", "esp32 uwb robot follow" |
| C: Альтернативы + калибровка | 5 | "UWB PDOA vs TWR accuracy comparison", "trilateration 3 anchors closed form", "DW3000 calibration antenna delay" |
| D: Китайские разработки | 4 | "超宽带 UWB 跟随 机器人 专利", "UWB 跟随 机器人 STM32 DW3000 BU04", "安信可 BU04 项目" |
| E: Патенты | 3 | "site:patents.google.com UWB trilateration robot following", "UWB follow robot patent CN105 CN106" |

---

### 14.1 Найденная документация DW3000

| # | Документ | URL | Статус | Размер | Ключевое содержание |
|---|----------|-----|--------|--------|---------------------|
| 1 | **DW3000 Datasheet** | Qorvo/Mouser | ✅ СКАЧАН | 6.2 МБ, 255 стр | Полные электрические характеристики, распиновка, карта регистров |
| 2 | **DW3000 User Manual (краткая версия)** | forum.qorvo.com | ✅ СКАЧАН | 68 КБ | Базовое описание, обзор программирования. НЕ полный мануал на ~200 стр |
| 3 | **DW3xxx API Guide** | forum.qorvo.com | ✅ СКАЧАН | 2.3 МБ | Все функции dwt_*: dwt_configure, dwt_readfromdevice, dwt_writetxdata и др. |
| 4 | **DW3000 User Manual (HTML)** | caramelfur.dev/docs/DW3000-User-Manual/ | 🌐 ОНЛАЙН | HTML | Полный мануал в HTML — каналы, RX/TX, MAC, описания регистров |
| 5 | **APS011 TWR Error Sources** | Qorvo (уже есть) | ✅ ЕСТЬ | 612 КБ, 22 стр | Анализ дрейфа часов, формула SDS-TWR, таблица калибровочных расстояний |
| 6 | **APS014 Antenna Delay Cal** | Qorvo (уже есть) | ✅ ЕСТЬ | 412 КБ | Процедура измерения задержки антенны |
| 7 | **APS017 Maximizing Range** | Qorvo (уже есть) | ✅ ЕСТЬ | 544 КБ | Оптимизация дальности DW3000 |
| 8 | **APH301 HW Design Guide** | Qorvo (уже есть) | ✅ ЕСТЬ | 1.5 МБ | Разводка платы, согласование антенны, питание |
| 9 | **DW3000_notes.md** | gist.github.com/egnor | 🌐 GIST | — | Заметки сообщества: особенности SPI, тайминги, грабли с регистрами |

**Ключевой вывод:** Полный DW3000 User Manual (~200 стр), вероятно, требует NDA/регистрации на qorvo.com. Версия 68 КБ — это краткая выжимка. HTML-версия на caramelfur.dev — community-hosted полный мануал.

**Ключевые функции DW3000 API Guide:**
- `dwt_configure()` — настройка канала, PRF, скорости данных, преамбулы
- `dwt_readfromdevice()` / `dwt_writetodevice()` — доступ к регистрам по SPI
- `dwt_writetxdata()` — передача произвольных данных (до 1023 байт) — КЛЮЧ для кнопки data-over-UWB!
- `dwt_readrxtimestamp()` — RX метка времени для TWR
- `dwt_setantennadelay()` — компенсация задержки антенны

**Применимость к нашему проекту:** ⭐⭐⭐⭐⭐ API Guide критичен для реализации кнопки data-over-UWB. Полный мануал (HTML) покрывает все регистры DW3000 для продвинутой реализации TWR.

---

### 14.2 Найденные UWB роботы-следователи (GitHub + проекты)

#### 14.2.1 unitree-go2-follow-system (orisharabi)
- **URL:** https://github.com/orisharabi/unitree-go2-follow-system
- **Язык:** Python | **Звёзд:** — 
- **UWB:** Использует ВСТРОЕННЫЙ UWB Unitree Go2 (не внешние BU04/DWM3000). Доступ через DDS сообщение `UwbState_`.
- **Алгоритм:** UWB даёт distance_est, yaw_est. Follow-контроллер: мёртвая зона 1.2м, max_vx=0.9 м/с. YOLOv8 для режима подхода к объекту.
- **Архитектура:** 3-состояния: FOLLOW (UWB) → APPROACH (YOLO) → HOLD → FOLLOW
- **Якорей:** 1 на роботе + 1 тег (НЕ трилатерация — один якорь, расстояние+угол)
- **Что берём:** Параметры FollowController (мёртвая зона 1.2м, smooth_alpha=0.2), паттерн поведенческого FSM, мониторинг UWB-кнопки
- **Применимость:** ⭐⭐⭐ (только архитектурные паттерны, аппаратно несовместимо)

#### 14.2.2 kk9six/dw3000 — ESP32 UWB DW3000 (NAIST, Япония)
- **URL:** https://github.com/kk9six/dw3000
- **Язык:** C++ (PlatformIO) | **Звёзд:** 31 | **Статья:** UMotion — CVPR 2025
- **Модули:** Makerfabs ESP32 DW3000 (чип DW3000)
- **Алгоритм:** Anchor-Tag (AT) DS-TWR + SS-TWR; режим Distance-Matrix (DM) для all-to-all ranging
- **Режимы:** `at_dstwr` — 1 тег + до 6 якорей, double-sided TWR. `dm_dstwr` — все узлы измеряют расстояния друг до друга
- **Ключевые файлы:** `src/at_dstwr/uwb.cpp` (протокол TWR), `src/dm_dstwr/uwb.cpp` (матрица расстояний)
- **Что берём:** Production-quality реализация DS-TWR, optimal inter-distance ranging protocol, планирование anchor-tag. Код протокола — референс для кастомной прошивки BU04
- **Применимость:** ⭐⭐⭐⭐ (чип DW3000 тот же в BU04, протокол напрямую портируем)

#### 14.2.3 Makerfabs ESP32-UWB-DW3000 (Референсная библиотека)
- **URL:** https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000
- **Язык:** C++ (Arduino) | **Звёзд:** 154 | **Форков:** 54
- **Библиотека:** DW3000 от NConcepts, поддерживается Makerfabs
- **Примеры:** range_tx/range_rx — базовое измерение расстояния (НЕ multi-anchor)
- **Ключевые файлы:** `Dw3000/src/dw3000_device_api.cpp` — HAL, `example/range/` — примеры TX/RX
- **Что берём:** Самая распространённая open-source библиотека DW3000. Можно адаптировать с ESP32 Arduino на STM32 (внутренний STM32F103 в BU04)
- **Применимость:** ⭐⭐⭐ (референс библиотеки, но BU04 работает через UART AT-команды, не прямой SPI)

#### 14.2.4 kimkihyun97/Hubito — UWB Human Following Robot
- **URL:** https://github.com/kimkihyun97/Hubito
- **Описание:** UWB-based human following robot. Корейский университетский проект.
- **Применимость:** ⭐⭐⭐ (концепт + архитектурный референс)

#### 14.2.5 KunYi/esp32-uwb-positioning-system — Многоякорная UWB система
- **URL:** https://github.com/KunYi/esp32-uwb-positioning-system
- **Описание:** 2-10 якорей, веб-визуализация, симулятор, ESP32 + DW3000
- **Что берём:** Production-quality multi-anchor система с веб-интерфейсом. Архитектура масштабирования якорей
- **Применимость:** ⭐⭐⭐⭐⭐ (самый близкий аналог к нашей архитектуре)

#### 14.2.6 Roiquiem/MaUWB_DW3000-with-STM32-AT-Command
- **URL:** https://github.com/Roiquiem/MaUWB_DW3000-with-STM32-AT-Command
- **Описание:** STM32 + DW3000 с AT-командами. MaUWB — ещё один производитель DW3000 модулей
- **Что берём:** Реализация AT-команд на STM32 — прямой референс для кастомизации BU04 прошивки
- **Применимость:** ⭐⭐⭐⭐⭐ (STM32 + AT-команды + DW3000 — точь-в-точь наша архитектура)

#### 14.2.7 Другие найденные проекты

| Проект | URL | Тип | Ценность |
|--------|-----|-----|----------|
| **L348350841/DW3000** | github.com/L348350841/DW3000 | UWB定位 | ⭐⭐⭐⭐ |
| **2411752523/UWB-Positioning-Car** | github.com/2411752523/UWB-Positioning-Car | UWB小车 | ⭐⭐⭐ |
| **DhamuVkl/ESP32-DWM3000-UWB-Indoor-RTLS-Tracker** | github.com/DhamuVkl | RTLS | ⭐⭐⭐⭐ |
| **KlemenBr/uwb_positioning** | github.com/KlemenBr/uwb_positioning | Обработка | ⭐⭐⭐ |
| **krebsbstn/uwb-tracking** | github.com/krebsbstn/uwb-tracking | Трекинг | ⭐⭐⭐ |
| **zerocompany/UWB-List** | github.com/zerocompany/UWB-List | Курируемый список | ⭐⭐⭐⭐⭐ |
| **ESP32-DW3000-AppleNearbyInteraction** | github.com/maa-x | Apple U1 | ⭐⭐ |
| **esphome-uwb-dw3000** | github.com/realzoulou | ESPHome | ⭐⭐⭐⭐ |

---

### 14.3 Альтернативные подходы и сравнения

| # | Ресурс | URL | Тип | Ключевой вывод |
|---|--------|-----|-----|----------------|
| 1 | **Qorvo Forum: Antenna Delay в DW3000 TWR** | forum.qorvo.com/t/17255 | Форум | Практические шаги настройки antenna delay на уровне регистров |
| 2 | **NiceRF: DW3000 UWB Indoor Positioning** | nicerf.com | Статья | TWR калибровка, рекомендации по размещению якорей |
| 3 | **Bluetooth.com.cn: Precise TWR with DW3000** | bluetooth.com.cn | Статья | Калибровка на уровне регистров, оптимизация оценки расстояния |
| 4 | **cliansang/positioning-algorithms-for-uwb-matlab** | github.com/cliansang | GitHub | MATLAB: трилатерация, мультилатерация, Калман, EKF для UWB |
| 5 | **Qorvo Forum: Trilateration Calculation Method** | forum.qorvo.com/t/9003 | Форум | Обсуждение инженеров — практическая реализация трилатерации |
| 6 | **MDPI: Improved Trilateration with Anchor Node** | mdpi.com | Статья | Взвешенная трилатерация — лучше чем обычный LS |
| 7 | **NTU Singapore: LIDAR+UWB Robot Following** | dr.ntu.edu.sg | Статья | Гибрид LIDAR-UWB для надёжного следования |
| 8 | **arXiv:2403.10194** — UWB на ESP32 и DWM3000 | arxiv.org | Статья (2024) | Система позиционирования UWB на базе ESP32 и DWM3000 |

**Сравнение PDOA vs TWR (из результатов поиска):**
- **PDOA:** ±60° угол обзора на BU04, точность ±10-15°, нужна прямая видимость. Один якорь измеряет угол. Хорошо для 2D плоскости где тег спереди
- **TWR:** 360° покрытие (с направленными антеннами в треугольнике), точность расстояния ±10см. Нужно 3 якоря для триангуляции. Работает через некоторые препятствия
- **Наше решение (D2):** TWR — 360° покрытие критично когда человек обходит робота

**Данные точности TWR (APS011 + сообщество):**
- DS-TWR (Double-Sided): ошибка дрейфа часов ~1-3см при кварцах 20ppm
- SS-TWR (Single-Sided): ошибка дрейфа часов ~20-60см при 20ppm — НЕПРИЕМЛЕМО
- Antenna delay: ~515нс (типичный DW3000), нужна калибровка ~1нс для ±30см → требуется поканальная калибровка
- Калибровочное расстояние: 2м рекомендуется (APS011 Таблица 3)

---

### 14.4 Найденные патенты (Google Patents + SIPO)

| Патент | Название | Год | Ключевое утверждение | Релевантность |
|--------|----------|-----|---------------------|---------------|
| CN105828431A | UWB autonomous following robot positioning method | 2016 | Аналитическая формула с 3 базовыми станциями (НАШ референс) | ⭐⭐⭐⭐⭐ |
| CN113282085A | Robot following system based on UWB | 2021 | UWB+vision fusion для следования | ⭐⭐⭐⭐ |
| CN116300613A | UWB+IMU intelligent following service robot | 2023 | Мульти-сенсор, behavior-aware control | ⭐⭐⭐⭐ |
| CN115239759A | Mobile robot following based on vision + UWB | 2022 | Визуально-UWB гибрид, re-identification цели | ⭐⭐⭐ |
| CN114625122A | Robot following + obstacle avoiding with UWB | 2022 | Планирование пути с объездом препятствий по UWB | ⭐⭐⭐⭐ |
| US20230008482A1 | Object following robot using UWB | 2023 | US патент на UWB следование с несколькими якорями | ⭐⭐⭐⭐ |
| KR20230007877A | Method for object following robot using UWB | 2023 | Корейский вариант — Samsung? | ⭐⭐⭐ |

**Вывод из патентного ландшафта:** CN105828431A (2016, наш референс) — foundational patent для аналитической 3-якорной трилатерации. Новые патенты (2021-2023) ФОКУСИРУЮТСЯ на UWB+vision/IMU fusion — подтверждает что чисто UWB следование это решённая задача, а передний край это мульти-сенсорная интеграция. Наш v1 подход (чистый UWB с аналитической формулой) — правильная стартовая точка; сенсорный фьюжн это v2+.

**Коммерческая патентная активность (китайские компании):**
- **耀晟智能 (YaoSheng AI)** — патент 2025 на UWB робота-следователя с behavior perception + dynamic control
- **华为 (Huawei)** — патент на оптимизацию UWB связи (chip-level)
- **汇顶科技 (Goodix)** — патент на UWB чип + метод связи

---

### 14.5 Китайские разработки и ресурсы

#### GitHub — китайскоязычные UWB проекты

| Проект | Описание | Применимость |
|--------|----------|-------------|
| **L348350841/DW3000** | UWB 人员定位 — позиционирование персонала, DW3000 + STM32 | ⭐⭐⭐⭐ |
| **Roiquiem/MaUWB_DW3000-with-STM32-AT-Command** | STM32 AT Command интерфейс для MaUWB DW3000 | ⭐⭐⭐⭐⭐ Прямой референс! |
| **2411752523/UWB-Positioning-Car** | UWB小车 — планирование пути по координатам UWB | ⭐⭐⭐ |
| **KunYi/esp32-uwb-positioning-system** | 2-10 якорей, веб-виз, симулятор, ESP32+DW3000 | ⭐⭐⭐⭐⭐ |

#### Китайские технические статьи (CSDN, Zhihu, Bilibili)

| Ресурс | Платформа | Содержание |
|--------|-----------|------------|
| UWB自动跟随技术原理、算法融合、优化 | CSDN | Глубокий разбор UWB auto-follow: принцип TWR, математика трилатерации, Калман, типичные грабли |
| DW3000+STM32定位通信模块设计 | CSDN | Проектирование модуля позиционирования DW3000+STM32 со схемами |
| BU04 UWB室内定位测距模块教程 | CSDN/Zhihu | Туториал по BU04: AT-команды, данные замеров |
| 安信可UWB模组智能跟随 | 什么值得买 | Гайд по Ai-Thinker UWB follow-me с практической настройкой |
| DW3000+STM32 B站视频 | Bilibili | Видео-туториал: проектирование и тестирование DW3000 |
| UWB与毫米波雷达融合智能跟随小车 | CSDN | UWB + mmWave radar fusion для машинки-следователя |
| 基于UWB定位的智能跟随车系统设计 | JICES (2023) | Академическая статья: дизайн системы UWB следования |

#### Китайские научные статьи

| Статья | Источник | Год | Ключевой вывод |
|--------|----------|-----|----------------|
| 智能跟随车 UWB定位系统设计 | JICES | 2023 | Полный дизайн системы: UWB позиционирование + управление машиной, тестированная точность |
| UWB+毫米波雷达融合跟随小车 | CSDN/Журнал | 2024 | Мульти-сенсор: UWB для позиции + mmWave для детекции препятствий |
| 华为UWB通信测距优化 | Sohu/Huawei | 2024 | Оптимизация ranging на уровне чипа — показывает инвестиции индустрии |

---

### 14.6 Анализ APS011 — формула DS-TWR и источники ошибок

**Источник:** Qorvo APS011 "Sources of Error in Two-Way Ranging (TWR) Schemes" (22 стр)

**Формула DS-TWR (Symmetric Double-Sided TWR):**
DS-TWR использует 3 сообщения для устранения дрейфа часов:
```
T_prop = (T_round1 × T_round2 − T_reply1 × T_reply2) / (T_round1 + T_round2 + T_reply1 + T_reply2)
```
Где T_prop = время полёта (истинное расстояние = T_prop × c), T_round = round-trip время, T_reply = задержка ответа.

**Источники ошибок (APS011):**

| Источник ошибки | Величина | Метод компенсации |
|----------------|----------|-------------------|
| Дрейф часов (кварц 20ppm) | ±2-3см для DS-TWR, ±20-60см для SS-TWR | Использовать DS-TWR; вариант SDS-TWR[4] для лучшей точности |
| Задержка антенны | 515нс типично (≈77м эквивалент!) | Калибровать по процедуре APS014; установить через `dwt_setantennadelay()` или AT+RNGOFF |
| Многолучевость | ±5-50см зависит от среды | Использовать leading edge detection, увеличить длину преамбулы |
| Шум (зависит от SNR) | ±1-5см типично | Усреднять несколько измерений, выше PRF |
| Range bias (Friis path loss) | Частотно-зависимый | Применить таблицу bias correction (APS011 Figure 11) |

**Процедура калибровки (APS011 + APS014):**
1. Разместить якорь и тег на расстоянии РОВНО 2.000м (измерить лазером/рулеткой)
2. Собрать 100+ замеров TWR в режиме DS-TWR
3. Вычислить среднюю ошибку: `bias = mean(измеренное) − 2.000м`
4. Перевести в коррекцию задержки: `delay_correction_ns = bias_m / c` где c = 0.2997 м/нс
5. Применить: AT+RNGOFF=<correction_cm> (BU04 AT прошивка) ИЛИ `dwt_setantennadelay()` (SDK)
6. Повторить замер — проверить остаточную ошибку < ±5см

**Калибровочные расстояния по каналам (APS011 Таблица 3):**
| Канал | PRF | Рекомендуемое расстояние |
|-------|-----|-------------------------|
| 5 (6.5GHz) | 64MHz | 2.0м |
| 9 (8GHz) | 64MHz | 2.0м |

**Наш план реализации:**
1. Установить BU04 в режим TWR: AT+SETUWBMODE=2
2. Разместить тег на 2.000м от каждого якоря A, B, C по очереди
3. Собрать 100 замеров расстояния с каждого якоря: AT+DISTANCE
4. Вычислить RNGOFF для каждого якоря: `AT+RNGOFF=<round(mean_error_cm)>`
5. Сохранить: AT+SAVE
6. Ожидаемая точность после калибровки: ±10см (патент) до ±15см (сообщество)

---

### 14.7 Сравнение прошивок BU03 vs BU04

**Источник:** STM32F103-BU0x_SDK/Components/APP/cmd_fn.c (Ai-Thinker SDK)

**Полная таблица AT-команд (29 команд):**

| # | Команда | BU03 | BU04 | TWR | PDOA | Описание |
|---|---------|------|------|-----|------|----------|
| 1 | AT | ✅ | ✅ | ✅ | ✅ | Тест AT-фреймворка |
| 2 | AT+GETVER | ✅ | ✅ | ✅ | ✅ | Версия ПО |
| 3 | AT+SAVE | ✅ | ✅ | ✅ | ✅ | Сохранить конфигурацию |
| 4 | AT+RESTART | ✅ | ✅ | ✅ | ✅ | Перезагрузка |
| 5 | AT+RESTORE | ✅ | ✅ | ✅ | ✅ | Сброс к заводским |
| 6 | AT+GETCFG | ✅ | ✅ | ✅ | ✅ | Получить конфигурацию |
| 7 | AT+SETCFG | ✅ | ✅ | ✅ | ✅ | Установить конфигурацию |
| 8 | AT+GETDEV | ✅ | ✅ | ✅ | ✅ | Информация об устройстве |
| 9 | AT+SETDEV | ✅ | ✅ | ✅ | ✅ | Установить параметры устройства |
| 10 | AT+GETWORKMODE | ✅ | ✅ | ✅ | ✅ | Режим работы (0=норм, 1=заводской) |
| 11 | AT+SETWORKMODE | ✅ | ✅ | ✅ | ✅ | Установить режим |
| 12 | AT+GETSENSOR | ✅ | ✅ | ✅ | ✅ | Данные LIS2DH12 акселерометра |
| 13 | AT+TESTLED | ✅ | ✅ | ✅ | ✅ | Тест светодиода |
| 14 | AT+TESTOLED | ✅ | ✅ | ✅ | ✅ | Тест OLED дисплея |
| **15** | **AT+DISTANCE** | ✅ | ✅ | ✅ | ❌ | **Получить TWR расстояние — КЛЮЧ для трилатерации** |
| 16 | AT+DECA$ | ✅ | ✅ | ✅ | ✅ | Прямая команда Decawave |
| 17 | AT+GETDLIST | ✅ | ✅ | ✅ | ✅ | Список устройств |
| 18 | AT+GETKLIST | ✅ | ✅ | ✅ | ✅ | Список известных устройств |
| 19 | AT+ADDTAG | ✅ | ✅ | ✅ | ✅ | Добавить тег |
| 20 | AT+DELTAG | ✅ | ✅ | ✅ | ✅ | Удалить тег |
| 21 | AT+PDOAOFF | ❌ | ✅ | ❌ | ✅ | Калибровка PDOA смещения |
| 22 | AT+RNGOFF | ✅ | ✅ | ✅ | ❌ | Калибровка смещения дальности |
| 23 | AT+FILTER | ✅ | ✅ | ✅ | ✅ | Параметры фильтра |
| 24 | AT+PDOASETCFG | ❌ | ✅ | ❌ | ✅ | Установить PDOA конфигурацию |
| 25 | AT+PDOAGETCFG | ❌ | ✅ | ❌ | ✅ | Получить PDOA конфигурацию |
| 26 | AT+UARTRATE | ✅ | ✅ | ✅ | ✅ | Скорость UART |
| **27** | **AT+USER_CMD** | ✅ | ✅ | ✅ | ✅ | **Пользовательская команда — точка расширения!** |
| 28 | AT+GETUWBMODE | ✅ | ✅ | ✅ | ✅ | Режим UWB (1=PDOA, 2=TWR) |
| 29 | AT+SETUWBMODE | ✅ | ✅ | ✅ | ✅ | Установить режим UWB |

**Анализ исходников SDK:**

1. **НЕТ условной компиляции:** Нет `#ifdef BU03` / `#ifdef BU04` в cmd_fn.c. Одинарный бинарный файл прошивки для обоих модулей.
2. **PDOA функции** (f_pdoaoff, f_rngoff, f_pdoasetcfg, f_pdoagetcfg) скомпилированы, но BU03 аппаратно не имеет двух антенн → PDOA команды дают HW ошибку на BU03.
3. **AT+USER_CMD** (f_user_cmd) — точка расширения для своих команд. Сейчас пустая. Сюда добавляем обработчик кнопки.
4. **known_commands[]** таблица на строке ~871 в cmd_fn.c — простой массив пар {"AT+CMD", function_ptr}. Добавить новую команду = 1 строка.
5. **dwt_writetxdata()** доступна через HAL DW3000 — SDK имеет полный TX путь. Data-over-UWB это ~20 строк в cmd_fn.c.

**Вердикт:**
- **BU03 как ТЕГ:** ✅ ДА. Всенаправленная антенна (IPEX) даёт лучшее 360° покрытие. PDOA не нужен для тега. Stock AT+DISTANCE отлично работает для TWR ответов.
- **BU04 как ЯКОРЬ:** ✅ ДА. Направленный луч 120° × 3 в треугольнике = 360° комбинированное покрытие. TWR ranging работает без использования PDOA функций.
- **Кнопка:** Добавить `AT+BUTTON` обработчик в f_user_cmd → читать GPIO → `dwt_writetxdata("BTN:1")`. ~20 строк C.

---

### 14.8 Статус статьи PMC8838499

**Статус:** ❌ СКАЧИВАНИЕ НЕ УДАЛОСЬ — PubMed Central возвращает HTML редирект, не PDF. Файл был 1.8KB HTML заглушка.

**Что знаем из предварительного анализа:**
- Метод: Component-wise error correction — декомпозиция ошибки позиционирования на компоненты (antenna delay, clock drift, multipath bias) и коррекция каждого отдельно
- Заявленная точность: ±X см (нужен доступ к полному тексту)

**Промежуточный вердикт:** ОТЛОЖИТЬ до v2. Патент CN105828431A + калибровка APS011 DS-TWR достаточны для точности ±10-15см в v1.

---

### 14.9 Сравнительный анализ: GitHub UWB роботы-следователи

| Проект | Чип UWB | Алгоритм | Якорей на роботе | Контроллер | Точность | Качество архитектуры | Совпадение с нами |
|--------|---------|----------|-----------------|------------|----------|---------------------|-------------------|
| unitree-go2-follow | Go2 встроенный | Один расстояние+угол | 1 | Go2 (ARM Linux) | ~20см | ⭐⭐⭐⭐ Python, чистый FSM | ⭐⭐⭐ |
| kk9six/dw3000 | DW3000 | DS-TWR, 1тег+N якорей | 0 (инфраструктура) | ESP32 | ~10см | ⭐⭐⭐⭐⭐ C++ PlatformIO | ⭐⭐⭐⭐ |
| Makerfabs DW3000 | DW3000 | Базовый TWR | 0 | ESP32 | ~10см | ⭐⭐⭐ Реф библиотека | ⭐⭐⭐ |
| Hubito | DW3000? | TWR following | ? | ? | ? | ⭐⭐⭐ | ⭐⭐⭐ |
| KunYi UWB | DW3000 | Multi-anchor TWR | 0 | ESP32 | ? | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| MaUWB DW3000 | DW3000 | AT-команды | ? | STM32 | ? | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |

**Ключевой вывод:** НЕТ существующего open-source проекта с 3-якорной трилатерацией НА роботе для BU04 модулей. Наш проект НОВЫЙ в комбинации:
1. 3× BU04 якоря на роботе (равносторонний треугольник)
2. Сопроцессор RP2040 с PIO UART
3. Аналитическая формула патента CN105828431A
4. AT+DISTANCE через стоковую прошивку (без своего SPI кода)
5. Data-over-UWB для кнопки (расширение AT+USER_CMD)

Этой комбинации нет ни в одном публичном репозитории — мы строим нечто новое.

---

### 14.10 Дополнительные научные статьи (раунд 2 поиска)

| Статья | Источник | Год | Ключевой вывод |
|--------|----------|-----|----------------|
| **Adaptive Robot Localization with UWB Novelty Detection** | arXiv:2505.05903 | 2025 | Адаптивная локализация с детекцией новизны — самый свежий research |
| **Survey: UWB localization for mobile autonomous robots** | ScienceDirect | 2025 | Полный обзор области — state-of-the-art на 2025 год |
| **Global UWB System for Mobile Robot Localization** | IEEE | 2024 | Высокоточная UWB локализация мобильных роботов |
| **Hybrid Human Tracking UWB + Monocular Visual** | ResearchGate | 2025 | UWB + монокулярная камера — сенсорный фьюжн |
| **Novel UWB Full-Range Multi-Angle Following Robot** | fyust.edu.cn | 2026 | Китайский университет — полно-диапазонное следование |
| **Autonomous Human Tracking UWB for Mobile Robots** | Cloudfront | — | Observer-based control подход |
| **UWB Side-by-Side Following** | ResearchGate | — | Адаптивное следование бок-о-бок |

---

## 15. Новые открытые вопросы

1. **DW3000 полный User Manual** — за Qorvo NDA? Попробовать запросить через sales@qorvo.com
2. **BU06/BU07** — что за новые модули Ai-Thinker? Проверить specs на docs.ai-thinker.com
3. **RP2040 PIO UART надёжность** — 3× UART RX на 115200 одновременно с потоком BU04 — эмпирический тест в Phase 4
4. **Оптимальный размер треугольника a** — патент говорит 30-50см, протестировать оба в Phase 4
5. **Многолучевость внутри помещений** — APS011 показывает 5-50см ошибки, проверить в реальной среде
6. **MaUWB DW3000 модуль** (Roiquiem) — ещё один производитель? Проверить совместимость с BU04 AT-командами
7. **耀晟智能 патент (2025)** — коммерческий UWB робот-следователь — конкурентный анализ?
8. **JICES 2023 статья** — полный дизайн UWB машинки-следователя — достать PDF для референса архитектуры
9. **zerocompany/UWB-List** — дособрать статьи из курируемого списка
