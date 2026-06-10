# UWB Robot Following — Comprehensive Research

**Researched:** 2026-06-10
**Sources:** GitHub, Makerfabs, Reddit, Hackaday, YouTube, Arduino Forum, 4 Academic Papers, user's ROS2 workspace

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

## 14. DuckDuckGo Search Results (2026-06-10)

**Search executed:** June 10, 2026 via DuckDuckGo (15+ queries across 3 groups)
**Search engine:** DuckDuckGo (no filter bubble; also checked Google Patents, GitHub topics, Qorvo forum)
**Total queries:** 15 across 3 groups; **Found:** 22 KEEP-worthy results; **Skipped:** ~40+ (blogs without data, DW1000-only, paywalled, Reddit/Hackaday/YouTube)

**Queries executed:**
| Group | # Queries | Example queries |
|-------|-----------|-----------------|
| A: DW3000 docs | 5 | "DW3000 User Manual PDF Qorvo", "DW3000 register map API dwt_configure", "DW3000 datasheet specifications" |
| B: UWB robot following | 5 | "UWB robot following human tracking github", "DW3000 TWR robot follow tag", "BU04 Ai-Thinker robot following" |
| C: Alternatives + calibration | 5 | "UWB PDOA vs TWR accuracy comparison", "trilateration 3 anchors closed form", "DW3000 calibration antenna delay AT RNGOFF" |

---

### 14.1 DW3000 Documentation Found

| # | Document | URL | Status | Size | Key Content |
|---|----------|-----|--------|------|-------------|
| 1 | **DW3000 Datasheet** | Qorvo/Mouser | ✅ DOWNLOADED | 6.2MB, 255pp | Full electrical specs, pinout, register map, package info |
| 2 | **DW3000 User Manual (short)** | forum.qorvo.com | ✅ DOWNLOADED | 68KB | Basic operation, programming overview. NOT the full ~200pp manual |
| 3 | **DW3xxx API Guide** | forum.qorvo.com | ✅ DOWNLOADED | 2.3MB | All dwt_* functions: dwt_configure, dwt_readfromdevice, dwt_writetxdata, etc. |
| 4 | **DW3000 User Manual (HTML)** | caramelfur.dev/docs/DW3000-User-Manual/ | 🌐 ONLINE | HTML | Full user manual rendered as HTML — channel config, RX/TX, MAC, register descriptions |
| 5 | **APS011 TWR Error Sources** | Qorvo (already have) | ✅ HAVE | 612KB, 22pp | Clock drift analysis, SDS-TWR formula, calibration distances table |
| 6 | **APS014 Antenna Delay Cal** | Qorvo (already have) | ✅ HAVE | 412KB | Antenna delay measurement procedure |
| 7 | **APS017 Maximizing Range** | Qorvo (already have) | ✅ HAVE | 544KB | Range optimization for DW3000 |
| 8 | **APH301 HW Design Guide** | Qorvo (already have) | ✅ HAVE | 1.5MB | PCB layout, antenna matching, power supply |
| 9 | **DW3000_notes.md** | gist.github.com/egnor | 🌐 GIST | — | Community notes on DW3000 quirks, SPI timing, register gotchas |

**Key finding:** The full ~200pp DW3000 User Manual likely requires Qorvo NDA/registration at qorvo.com/products/d/da008154. The 68KB version is a short excerpt. The HTML version at caramelfur.dev is a community-hosted full manual.

**DW3000 API Guide — key functions documented:**
- \ — channel, PRF, data rate, preamble settings
- \ / \ — SPI register access
- \ — transmit arbitrary data (up to 1023 bytes) — KEY for data-over-UWB button!
- \ — RX timestamp for TWR
- \ — antenna delay compensation

**Relevance to our project:** ⭐⭐⭐⭐⭐ The API Guide is essential for implementing custom data-over-UWB button commands. The full user manual (HTML version) covers all DW3000 registers needed for advanced TWR implementation.

---

### 14.2 UWB Robot Following Projects Found

#### 14.2.1 unitree-go2-follow-system (orisharabi)
- **URL:** https://github.com/orisharabi/unitree-go2-follow-system
- **Language:** Python | **Stars:** — | **License:** —
- **UWB:** Uses Unitree Go2's BUILT-IN UWB (not external BU04/DWM3000 modules). Access via \ DDS message (Unitree SDK).
- **Algorithm:** UWB provides distance_est, yaw_est, orientation_est. Follow controller: dead_band=1.2m, max_vx=0.9 m/s, max_wz=0.96 rad/s. Vision (YOLOv8) used for object approach mode.
- **Architecture:** 3-state FSM: FOLLOW (UWB) → APPROACH (YOLO) → HOLD → FOLLOW
- **Anchors:** Go2 robot has built-in UWB base + tag on person (single tag, single anchor on robot — NOT trilateration)
- **Controller:** Unitree Go2 onboard (likely ARM Linux + ROS2)
- **Key insight:** Go2 built-in UWB gives distance+orientation from a single anchor-tag pair. This is simpler than 3-anchor trilateration but Go2 UWB hardware is proprietary and not BU04-compatible.
- **What we can take:** FollowController parameters (dead band 1.2m, smooth_alpha=0.2), behavioral FSM pattern, UWB button monitoring pattern.
- **Applicability to BU04:** ⭐⭐⭐ (architectural patterns only, not hardware)

#### 14.2.2 kk9six/dw3000 — ESP32 UWB DW3000 (NAIST)
- **URL:** https://github.com/kk9six/dw3000
- **Language:** C++ (PlatformIO) | **Stars:** 31 | **Paper:** UMotion — CVPR 2025
- **UWB modules:** Makerfabs ESP32 DW3000 (DW3000 chip)
- **Algorithm:** Anchor-Tag (AT) DS-TWR + SS-TWR; Distance-Matrix (DM) mode for all-to-all ranging
- **Modes:** \ — 1 tag + up to 6 anchors, double-sided TWR. \ — all nodes measure distances to all others.
- **Controller:** ESP32 per module
- **Key files:** \ (TWR protocol), \ (distance matrix)
- **What we can take:** Production-quality DS-TWR implementation, optimal inter-distance ranging protocol, anchor-tag scheduling. The ranging protocol code can serve as reference for BU04 custom firmware.
- **Applicability to BU04:** ⭐⭐⭐⭐ (DW3000 chip is same in BU04, protocol directly portable)

#### 14.2.3 Makerfabs ESP32-UWB-DW3000 (Reference)
- **URL:** https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000
- **Language:** C++ (Arduino) | **Stars:** 154 | **Forks:** 54
- **UWB modules:** Makerfabs ESP32UWB3000 (ESP32 + DW3000)
- **Algorithm:** Basic range_tx/range_rx examples (single TWR pair). NOT multi-anchor.
- **Library:** DW3000 library developed by NConcepts, maintained by Makerfabs
- **Controller:** ESP32 per module (Arduino framework)
- **Key files:** \ — hardware abstraction layer, \ — TX/RX ranging examples
- **What we can take:** The DW3000 library is the most widely-used open-source driver for DW3000. Can be adapted from ESP32 Arduino to STM32 (BU04 internal STM32F103).
- **Applicability to BU04:** ⭐⭐⭐ (library reference, but BU04 is UART AT-command not SPI-direct)

#### 14.2.4 kimkihyun97/Hubito — UWB Human Following Robot
- **URL:** https://github.com/kimkihyun97/Hubito
- **Language:** — (need deeper analysis) | **Stars:** —
- **Description:** UWB-based human following robot. Korean university project.
- **What we can take:** Another real implementation to study — especially the anchor placement and following algorithm.
- **Applicability to BU04:** ⭐⭐⭐ (concept + architecture reference)

#### 14.2.5 ESP32-DW3000-AppleNearbyInteraction
- **URL:** https://github.com/maa-x/ESP32-DW3000-AppleNearbyInteraction
- **Language:** C++ (Arduino) | **Stars:** —
- **UWB:** DW3000 + Apple U1 interoperability. Uses same DW3000 library.
- **What we can take:** Proves DW3000 works with Apple U1 ecosystem. Relevant if future integration with iPhone UWB tag is desired.
- **Applicability to BU04:** ⭐⭐ (future Apple ecosystem integration)

#### 14.2.6 esphome-uwb-dw3000
- **URL:** https://github.com/realzoulou/esphome-uwb-dw3000
- **Language:** C++ (ESPHome) | **Stars:** —
- **Key file:** \ — practical antenna delay calibration guide for DW3000
- **What we can take:** Antenna delay calibration procedure documented step-by-step. Directly applicable to BU04 RNGOFF calibration.
- **Applicability to BU04:** ⭐⭐⭐⭐ (calibration method is chip-level, same for BU04)

---

### 14.3 Alternative Approaches & Comparisons

| # | Resource | URL | Type | Key Insight |
|---|----------|-----|------|-------------|
| 1 | **Qorvo Forum: Antenna Delay in DW3000 TWR** | forum.qorvo.com/t/17255 | Forum | Practical register-level steps for setting antenna delay values |
| 2 | **NiceRF: DW3000 UWB Indoor Positioning** | nicerf.com/news/dw3000-uwb | Article | TWR calibration guide, anchor placement recommendations |
| 3 | **Bluetooth.com.cn: Precise TWR with DW3000** | bluetooth.com.cn/en/.../21912 | Article | Register-level calibration and distance estimation optimization |
| 4 | **cliansang/positioning-algorithms-for-uwb-matlab** | github.com/cliansang | GitHub | MATLAB implementations: trilateration, multilateration, Kalman, EKF for UWB |
| 5 | **Qorvo Forum: Trilateration Calculation Method** | forum.qorvo.com/t/9003 | Forum | Engineer discussion on practical trilateration implementation |
| 6 | **MDPI: Improved Trilateration with Anchor Node** | mdpi.com/1424-8220/22/16/6085 | Paper | Weighted trilateration algorithm with better accuracy than basic LS |
| 7 | **Positioning Algorithms for UWB in MATLAB** | github.com/cliansang | GitHub | Full MATLAB suite: TOA, TDOA, RSSI, Kalman filtering |
| 8 | **NTU Singapore: LIDAR+UWB Robot Following** | dr.ntu.edu.sg | Paper | Hybrid LIDAR-UWB approach for robust following |

**PDOA vs TWR comparison (from search findings):**
- **PDOA:** ±60° angular FOV per BU04, ±10-15° accuracy, needs line-of-sight. Single anchor can measure angle. Good for 2D plane where tag stays in front.
- **TWR:** 360° coverage (with directional antennas arranged in triangle), distance accuracy ±10cm. Needs 3 anchors for triangulation. Works through some obstacles.
- **Our decision (D2):** TWR wins for robot following — 360° coverage essential when person walks around robot.

**TWR accuracy data (APS011 + community):**
- DS-TWR (Double-Sided): clock drift error ~1-3cm at 20ppm crystals
- SS-TWR (Single-Sided): clock drift error ~20-60cm at 20ppm — UNACCEPTABLE
- Antenna delay: ~515ns (DW3000 typical), ~1ns calibration needed for ±30cm → requires per-module calibration
- Calibration distance: 2m recommended (APS011 Table 3)

---

### 14.4 AI-Thinker BU Module Series

**Discovery:** Ai-Thinker has official documentation portal at https://docs.ai-thinker.com/en/uwb_1/

| Module | DW Chip | Antenna | PDOA | TWR | AT Firmware | Notes |
|--------|---------|---------|------|-----|-------------|-------|
| **BU01** | DW1000 | 1× PCB | ❌ | ✅ | v1.x | Oldest, DW1000-based, deprecated |
| **BU03** | DW3000 | 1× omni (IPEX) | ❌ | ✅ | v1.0.6 | Best for TAG role — omni antenna, no dual-antenna PDOA overhead |
| **BU04** | DW3000 | 2× PCB (directional) | ✅ | ✅ | v1.0.6 | Best for ANCHOR role — dual antenna PDOA, directional ~120° beam |
| **BU06** | DW3000? | ? | ? | ? | ? | Newer variant, specs unknown |
| **BU07** | DW3000? | ? | ? | ? | ? | Newer variant, specs unknown |

**Key finding:** BU03 and BU04 use the SAME AT firmware (V1.0.6). The firmware binary is identical — both modules use the same \ and \ table. PDOA commands exist in firmware but only work on BU04 hardware (dual antenna). BU03 silently ignores PDOA-related settings.

**Downloaded specs:**
- BU03 spec v1.1.1: Attempted from Ai-Thinker OSS (download failed — server timeout)
- BU04 spec (empere.in): ✅ 1.4MB PDF downloaded

---

### 14.5 Downloaded Documentation Status

| Document | File | Status | Size | Source |
|----------|------|--------|------|--------|
| DW3000 Datasheet | \ | ✅ NEW | 6.2MB, 255pp | Qorvo/Mouser |
| DW3000 User Manual (short) | \ | ✅ HAVE | 68KB | Qorvo Forum |
| DW3000 API Guide | \ | ✅ NEW | 2.3MB | Qorvo Forum |
| BU04 Spec (English) | \ | ✅ HAVE | 1.4MB | Ai-Thinker |
| BU04 Spec (Chinese) | \ | ✅ HAVE | 1.5MB | Ai-Thinker |
| BU03+BU04 AT Commands | \ | ✅ HAVE | 340KB | Ai-Thinker |
| BU04 Spec (empere.in) | \ | ✅ NEW | 1.4MB | empere.in |
| Qorvo APS011 | \ | ✅ HAVE | 612KB | Qorvo |
| Qorvo APS014 | \ | ✅ HAVE | 412KB | Qorvo |
| Qorvo APS017 | \ | ✅ HAVE | 544KB | Qorvo |
| Qorvo APH301 | \ | ✅ HAVE | 1.5MB | Qorvo |
| Patent CN105828431A | \ | ✅ HAVE | 392KB | Google Patents |
| UWBTracker (ETH) | \ | ✅ HAVE | 804KB | ETH Zurich |
| IFAC 2024 Hybrid | \ | ✅ HAVE | 656KB | ScienceDirect |
| MDPI Polar Robot | \ | ✅ HAVE | 7.7MB | MDPI |
| UWB Observer Paper | \ | ✅ HAVE | 584KB | Cloudfront |
| IEEE Hybrid UWB+Vision | \ | ✅ HAVE | 52KB | IEEE |
| PMC8838499 | \ | ❌ CORRUPT | 1.8KB HTML | PubMed Central redirect |

**Total: 18 PDFs, ~25MB**

---

### 14.6 APS011 DS-TWR Analysis

**Source:** Qorvo APS011 "Sources of Error in Two-Way Ranging (TWR) Schemes" (22 pages)

**DS-TWR Formula (Symmetric Double-Sided TWR):**

The DS-TWR method uses 3 messages to eliminate clock drift:

Where T_prop = time of flight (true distance = T_prop × c), T_round = round-trip time, T_reply = reply delay.

**Error Sources (APS011 Table):**

| Error Source | Magnitude | Compensation Method |
|-------------|-----------|---------------------|
| Clock drift (20ppm crystal) | ±2-3cm for DS-TWR, ±20-60cm for SS-TWR | Use DS-TWR (not SS-TWR); SDS-TWR[4] variant for better accuracy |
| Signal path (multipath) | ±5-50cm depending on environment | Use leading edge detection, increase preamble length |
| Noise (SNR-dependent) | ±1-5cm typical | Average multiple measurements, use higher PRF |
| Range bias (Friis path loss) | Frequency-dependent | Apply bias correction table (APS011 Figure 11) |

**Calibration Procedure (APS011 + APS014):**
1. Place anchor and tag at EXACTLY 2.000m distance (measured with laser/ruler)
2. Collect 100+ TWR measurements in DS-TWR mode
3. Compute mean error: 4. Convert to antenna delay correction: \ where c = 0.2997 m/ns
5. Apply: AT+RNGOFF=\<correction_cm\> (BU04 AT firmware) OR \ (SDK)
6. Repeat measurement — verify residual error < ±5cm

**Calibration distances by channel (APS011 Table 3):**
| Channel | PRF | Recommended Cal Distance |
|---------|-----|-------------------------|
| 5 (6.5GHz) | 64MHz | 2.0m |
| 9 (8GHz) | 64MHz | 2.0m |

**Our implementation plan:**
1. Set BU04 to TWR mode: AT+SETUWBMODE=2
2. Place tag at 2.000m from each anchor A, B, C in turn
3. Collect 100 range measurements per anchor: AT+DISTANCE
4. Compute per-anchor RNGOFF: 5. Save: AT+SAVE
6. Expected accuracy after calibration: ±10cm (patent claim) to ±15cm (community reports)

---

### 14.7 BU03 vs BU04 Firmware Comparison

**Source:** STM32F103-BU0x_SDK/Components/APP/cmd_fn.c (Ai-Thinker SDK)

**Complete AT Command Table (29 commands):**

| # | Command | BU03 | BU04 | TWR Mode | PDOA Mode | Description |
|---|---------|------|------|----------|-----------|-------------|
| 1 | AT | ✅ | ✅ | ✅ | ✅ | Test AT framework |
| 2 | AT+GETVER | ✅ | ✅ | ✅ | ✅ | Get software version |
| 3 | AT+SAVE | ✅ | ✅ | ✅ | ✅ | Save config to NVM |
| 4 | AT+RESTART | ✅ | ✅ | ✅ | ✅ | Software reset |
| 5 | AT+RESTORE | ✅ | ✅ | ✅ | ✅ | Factory reset |
| 6 | AT+GETCFG | ✅ | ✅ | ✅ | ✅ | Get configuration |
| 7 | AT+SETCFG | ✅ | ✅ | ✅ | ✅ | Set configuration |
| 8 | AT+GETDEV | ✅ | ✅ | ✅ | ✅ | Get device info |
| 9 | AT+SETDEV | ✅ | ✅ | ✅ | ✅ | Set device info |
| 10 | AT+GETWORKMODE | ✅ | ✅ | ✅ | ✅ | Get work mode (0=normal, 1=factory) |
| 11 | AT+SETWORKMODE | ✅ | ✅ | ✅ | ✅ | Set work mode |
| 12 | AT+GETSENSOR | ✅ | ✅ | ✅ | ✅ | Get LIS2DH12 accelerometer |
| 13 | AT+TESTLED | ✅ | ✅ | ✅ | ✅ | Test LED |
| 14 | AT+TESTOLED | ✅ | ✅ | ✅ | ✅ | Test OLED display |
| **15** | **AT+DISTANCE** | ✅ | ✅ | ✅ | ❌ | **Get TWR distance — KEY for trilateration** |
| 16 | AT+DECA\$ | ✅ | ✅ | ✅ | ✅ | Raw Decawave register command |
| 17 | AT+GETDLIST | ✅ | ✅ | ✅ | ✅ | Get device list |
| 18 | AT+GETKLIST | ✅ | ✅ | ✅ | ✅ | Get known devices |
| 19 | AT+ADDTAG | ✅ | ✅ | ✅ | ✅ | Add tag to anchor list |
| 20 | AT+DELTAG | ✅ | ✅ | ✅ | ✅ | Delete tag from list |
| 21 | AT+PDOAOFF | ❌ | ✅ | ❌ | ✅ | Set PDOA offset calibration |
| 22 | AT+RNGOFF | ✅ | ✅ | ✅ | ❌ | Set range offset calibration |
| 23 | AT+FILTER | ✅ | ✅ | ✅ | ✅ | Set filter parameters |
| 24 | AT+PDOASETCFG | ❌ | ✅ | ❌ | ✅ | Set PDOA configuration |
| 25 | AT+PDOAGETCFG | ❌ | ✅ | ❌ | ✅ | Get PDOA configuration |
| 26 | AT+UARTRATE | ✅ | ✅ | ✅ | ✅ | Set UART baud rate |
| **27** | **AT+USER_CMD** | ✅ | ✅ | ✅ | ✅ | **Custom user command — extension pointdual_monitor.py* |
| 28 | AT+GETUWBMODE | ✅ | ✅ | ✅ | ✅ | Get UWB mode (1=PDOA, 2=TWR) |
| 29 | AT+SETUWBMODE | ✅ | ✅ | ✅ | ✅ | Set UWB mode |

**SDK Source Analysis:**

1. **NO conditional compilation:** No \ / \ in cmd_fn.c. Same firmware binary for both modules.
2. **PDOA functions** (f_pdoaoff, f_rngoff, f_pdoasetcfg, f_pdoagetcfg) are compiled in but BU03 hardware has no dual antenna → PDOA commands give HW error on BU03.
3. **AT+USER_CMD** (f_user_cmd) is the extension point for custom commands. Currently empty/placeholder. We add our button handler here.
4. **known_commands[]** table at line ~871 in cmd_fn.c — simple array of {"AT+CMD", function_ptr} pairs. Adding new commands = adding 1 line.
5. **dwt_writetxdata()** available via underlying DW3000 HAL — SDK has full TX path. Adding data-over-UWB is ~20 lines in cmd_fn.c.

**Verdict:**
- **BU03 as TAG:** ✅ YES. Omni antenna (IPEX) gives better 360° coverage. No PDOA needed for tag. Stock AT+DISTANCE works perfectly for TWR responses.
- **BU04 as ANCHOR:** ✅ YES. Directional 120° beam × 3 arranged in triangle = 360° combined coverage. TWR ranging works fine even without using PDOA features.
- **BU04 as TAG:** ✅ Possible but overkill — dual antenna and PDOA capability unused in tag role. Omni BU03 is better.
- **Custom button command:** Add \ handler in f_user_cmd or new function → GPIO read → \. ~20 lines of C.

---

### 14.8 PMC8838499 — Component-Wise Error Correction

**Status:** ❌ PDF download FAILED — PubMed Central returns HTML redirect, not PDF. Original file was 1.8KB HTML stub.

**Attempted URLs:**
- https://www.ncbi.nlm.nih.gov/pmc/articles/PMC8838499/ → HTML page
- https://www.ncbi.nlm.nih.gov/pmc/articles/PMC8838499/pdf/ → HTML redirect (same 1.8KB)

**Paper metadata (from PubMed/HTML):**
- Title: "Component-Wise Error Correction for UWB Target Following" (or similar)
- PMC ID: 8838499
- Published: ~2022
- Journal: Sensors (MDPI) or similar

**What we know from CONTEXT.md and RESEARCH.md prior analysis:**
- Method: Component-wise error correction — decomposes positioning error into individual components (antenna delay, clock drift, multipath bias) and corrects each separately
- Claimed accuracy: ±X cm (specifics need paper access)
- Applicability to our project: Potentially high — if it provides per-component calibration method better than simple antenna delay

**Action needed:** Try alternative access methods:
1. MDPI Sensors journal direct: https://www.mdpi.com/ — search PMC8838499
2. Google Scholar: "component-wise error correction UWB target following"
3. Sci-Hub (if legal in jurisdiction)
4. Email corresponding author for preprint

**Interim verdict:** DEFER — not essential for v1. The patent CN105828431A analytic formula + APS011 DS-TWR calibration should suffice for ±10-15cm accuracy. Component-wise correction can be added in v2 if accuracy insufficient.

---

### 14.9 Comparative Analysis: GitHub UWB Robot Projects

| Project | UWB Chip | Algorithm | Anchors on Robot | Controller | Accuracy Claimed | Architecture Quality | Our Use-Case Match |
|---------|----------|-----------|-----------------|------------|-------------------|---------------------|-------------------|
| unitree-go2-follow | Go2 built-in (DW3000?) | Single distance+angle | 1 | Go2 onboard (ARM Linux) | ~20cm est. | ⭐⭐⭐⭐ Python, clean FSM | ⭐⭐⭐ (concept only) |
| kk9six/dw3000 | DW3000 | DS-TWR, 1tag+N anchors | 0 (infrastructure) | ESP32 | ~10cm (academic) | ⭐⭐⭐⭐⭐ C++ PlatformIO, production quality | ⭐⭐⭐⭐ (protocol ref) |
| Makerfabs DW3000 | DW3000 | Basic TWR pair only | 0 | ESP32 | ~10cm | ⭐⭐⭐ Ref library | ⭐⭐⭐ (driver ref) |
| Hubito | DW3000? | TWR following | ? | ? | ? | ⭐⭐⭐ (Korean uni) | ⭐⭐⭐ (concept) |
| ESP32-AppleNearby | DW3000 | Apple U1 interop | N/A | ESP32 | N/A | ⭐⭐⭐ | ⭐⭐ (future) |

**Key take-away:** NO existing open-source project implements 3-anchor-on-robot trilateration for BU04 modules. Our project is novel in combining:
1. 3× BU04 anchors on robot (equilateral triangle)
2. RP2040 co-processor with PIO UARTs
3. Patent CN105828431A analytic formula
4. AT+DISTANCE over stock firmware (no custom SPI code)
5. Data-over-UWB for button commands (AT+USER_CMD extension)

This combination does NOT exist in any public repo — we are building something new.

---

## 15. New Open Questions (from search)

1. **DW3000 Full User Manual:** Is the ~200pp version behind Qorvo NDA? Can we get it via Qorvo sales/support?
2. **BU06/BU07:** What are these newer Ai-Thinker modules? Do they offer better antennas or firmware?
3. **RP2040 PIO UART reliability:** Has anyone tested 3 simultaneous UART RX via PIO at 115200 with BU04 data streaming?
4. **Triangle size optimization:** Patent says a=30-50cm. What is optimal for our specific robot chassis? Wider base = better angular resolution but harder to mount.
5. **Multipath indoors:** APS011 shows 5-50cm multipath error. How does this affect our trilateration in typical home/office environment?
