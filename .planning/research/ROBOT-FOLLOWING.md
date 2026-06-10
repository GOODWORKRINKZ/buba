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
