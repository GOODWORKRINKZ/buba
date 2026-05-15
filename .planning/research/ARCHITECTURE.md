# Architecture Research — UWB PDOA System

**Project:** BU04 UWB Positioning & Robot Following
**Researched:** 2026-05-15
**Confidence:** HIGH (v1 test bench) / MEDIUM (v2 robot following — hardware not yet integrated)

---

## System Components

### Test Bench (v1)

| Component | Hardware | Role |
|-----------|----------|------|
| **Firmware: Anchor** | ESP32-C3 + BU04 | Configures BU04 in PDOA mode; parses streaming measurement lines from BU04; emits CSV to PC |
| **Firmware: Tag** | ESP32-C3 + BU04 | Stays in user's pocket; BU04 transmits UWB frames autonomously; ESP32 does no data output |
| **PC Visualizer** | Python (pyserial + matplotlib) | Reads serial CSV, live polar plot, CSV log file |
| **Calibration** | Python CLI script | Collects raw PDOA samples at known positions; computes PDOAOFF + RNGOFF; no reflash needed |

#### What each component owns

**Anchor firmware** (`anchor1_pdoa` environment):
- BU04 configuration sequence (SETUWBMODE, PDOAOFF, RNGOFF, ADDTAG)
- `AT+USER_CMD=0` continuous stream parsing (`fieldVal()`)
- Moving-average filter on range
- CSV serialisation → `USB Serial`

**Tag firmware** (`tag` environment):
- 2-phase BU04 cold-start sequence (DW3000 initialisation workaround)
- No data output to PC; silently participates in UWB ranging when powered on

**PC Visualizer** (`tools/visualizer.py`):
- Serial port thread (daemon, `readline`)
- Thread-safe ring buffer (deque)
- FuncAnimation polar scatter with trail
- Append-mode CSV log

**Calibration tool** (`tools/calibrate.py`):
- Collects N samples at commanded distance and angle
- Computes mean offset (PDOAOFF candidate) and range bias (RNGOFF candidate)
- Prints `config.h` snippet ready to copy-paste

---

### Robot Following (v2)

Motor control lives in a **new build environment on the same anchor ESP32-C3** — not a separate microcontroller. This avoids extra hardware (the only available MCUs are the two ESP32-C3s already in use).

New build environment: `robot_follower`
Build flags: `-DROLE_ANCHOR1=1 -DPDOA_MODE=1 -DROBOT_MODE=1`

The PDOA parsing loop is unchanged; a new code block (guarded by `#ifdef ROBOT_MODE`) replaces the CSV-to-USB Serial output with a P-controller → H-bridge PWM output.

USB Serial is retained for debug telemetry in robot mode (same CSV format — the robot can be monitored from PC while it follows).

**Robot v2 components added:**

| Component | Hardware | Role |
|-----------|----------|------|
| **H-bridge driver** | L298N or DRV8833 | Converts PWM + direction GPIO from ESP32-C3 to motor current |
| **Left motor + Right motor** | DC gear motors | Differential drive platform |
| **Robot firmware** | Same ESP32-C3 anchor | P-controller consuming (range_m, angle_deg), outputting PWM |

The tag ESP32-C3 is unchanged in v2 — still just a BU04 powered on in the user's pocket.

---

## Data Flow

### v1: PDOA → PC

```
┌────────────────────────────────────────────────────────────────────────┐
│  BU04 (anchor)                                                         │
│  DW3000 PDOA engine                                                    │
│  Dual-antenna phase comparison → angle + distance                      │
│  STM32F103 formats → UART streaming:                                   │
│  "Tag_Addr:0001, Seq:42, Xcm:15, Ycm:230, Range:230, Angle:4\r\n"    │
└────────────────────┬───────────────────────────────────────────────────┘
                     │ UART1 115200 baud (GPIO2/3)
                     ▼
┌────────────────────────────────────────────────────────────────────────┐
│  ESP32-C3 (anchor firmware — loop())                                   │
│                                                                        │
│  bu04.available() → read char → accumulate line                        │
│  fieldVal("Range:") → range_cm   → /100 → range_m                     │
│  fieldVal("Angle:") → angle_deg  (signed, −90..+90)                   │
│  fieldVal("Tag_Addr:") → addr_hex                                      │
│                                                                        │
│  updateAvg(range_m)  ← 5-sample moving average (existing)             │
│                                                                        │
│  Serial.printf("PDOA,%s,%u,%.2f,%.1f,%.2f,%.2f,%lu\r\n",             │
│                 addr, seq, range_m, angle_deg, x_m, y_m, millis())    │
└────────────────────┬───────────────────────────────────────────────────┘
                     │ USB CDC 115200 baud (/dev/ttyACM0)
                     ▼
┌────────────────────────────────────────────────────────────────────────┐
│  Python visualizer (PC)                                                │
│                                                                        │
│  serial.Serial() → daemon reader thread → deque(maxlen=200)           │
│  FuncAnimation (100 ms interval) → polar scatter plot                 │
│  csv.writer → append PDOA rows to session log                         │
└────────────────────────────────────────────────────────────────────────┘
```

### v2: PDOA → Robot Controller

```
BU04 UART stream
        │
        ▼  (same PDOA parser as v1)
   (range_m, angle_deg)  ← ~5 Hz
        │
        ▼
   Moving-average filter (range: N=5, angle: N=3 recommended)
        │
        ├── [USB Serial] CSV debug → PC (optional monitor)
        │
        └── [#ifdef ROBOT_MODE]
              P-controller:
                angle_err  = angle_deg              (target = 0°)
                range_err  = range_m - FOLLOW_DIST  (target = ~0.5 m)

                turn  = K_ANGLE * angle_err         (±, positive = left)
                fwd   = K_RANGE * range_err         (positive = forward)

                left_pwm  = clamp(BASE + fwd + turn, 0, 255)
                right_pwm = clamp(BASE + fwd - turn, 0, 255)
              │
              ▼
           ledc PWM → H-bridge → Left motor / Right motor
```

**Control constants (start values to tune on bench):**

| Constant | Suggested start | Meaning |
|----------|-----------------|---------|
| `FOLLOW_DIST` | 0.50 m | Distance setpoint |
| `K_ANGLE` | 3.0 | Turn gain (PWM units per degree) |
| `K_RANGE` | 80.0 | Forward gain (PWM units per metre) |
| `BASE` | 0 | Added to both wheels (0 = stop when on-target) |

Dead-band: if `range_m < 0.3` → stop both motors (tag too close).
Safety: if no PDOA packet received for >500 ms → stop both motors (tag lost).

---

## Component Boundaries

```
┌──────────────────────────────────────────────────────────────────────────┐
│ config.h (shared constants)                                              │
│  PIN_BU04_TX/RX, BU04_BAUD, BU04_CHANNEL, BU04_RATE, BU04_GROUP        │
│  BU04_ID_ANCHOR1, BU04_ID_TAG                                            │
│  PDOA_OFFSET_DEG, RANGE_OFFSET_CM        ← calibration                  │
│  POLL_INTERVAL_MS, MOVING_AVG_SAMPLES, MAX_VALID_DIST_M                 │
│  [v2 adds] FOLLOW_DIST_M, K_ANGLE, K_RANGE, PIN_MOTOR_L1/L2/R1/R2      │
└──────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────┐    UART1     ┌────────────────────────────┐
│ BU04 anchor module       │ ──────────── │ anchor1_pdoa / robot       │
│  DW3000 PDOA engine      │   AT cmds    │  firmware (main.cpp)       │
│  STM32F103 AT interface  │ ◄──────────  │  configureBU04()           │
│                          │   UART data  │  sendAT(AT+PDOAOFF)        │
│                          │ ──────────►  │  sendAT(AT+RNGOFF)         │
│                          │              │  sendAT(AT+ADDTAG)         │
└──────────────────────────┘              │  loop(): fieldVal parser   │
                                          │  updateAvg()               │
┌──────────────────────────┐    UART1     │  → CSV Serial println      │
│ BU04 tag module          │ ──────────── │  [robot] P-controller      │
│  DW3000 UWB initiator    │   AT cmds    │  [robot] ledc PWM          │
│                          │ ◄──────────  └────────────┬───────────────┘
└──────────────────────────┘                           │ USB CDC
                                                        ▼
                                          ┌────────────────────────────┐
                                          │ PC Python tools/           │
                                          │  visualizer.py             │
                                          │  calibrate.py              │
                                          └────────────────────────────┘
```

**Boundary rule:** The ESP32-C3 is the only bridge between BU04's AT world and everything else. It must not be bypassed (no direct PC-to-BU04 serial path in production — only in `test_bu04` diagnostic environment).

---

## Calibration Workflow

Calibration must happen before accuracy validation. It has two sub-workflows:

### A. AT+PDOAOFF (angle offset)

Corrects systematic phase asymmetry between BU04's two antenna PCB traces.

```
1. Flash anchor1_pdoa with PDOA_OFFSET_DEG=0 in config.h
2. Place anchor on stable surface, antenna plane facing the tag straight ahead
3. Place tag at exactly 0° (equidistant from both antennas), ~1.5 m
4. Run tools/calibrate.py --mode angle --samples 50
   → Script collects 50 PDOA lines, computes mean angle
   → Prints: PDOA_OFFSET_DEG = -<mean_angle>
5. Update config.h: #define PDOA_OFFSET_DEG  <value>
6. Reflash (AT+PDOAOFF is sent in setup() after AT+SETUWBMODE=1)
7. Verify: at 0°, ±30°, ±60° — expect residual < ±15°
```

**Where the command goes in firmware:**

```
setup() → waitBU04Ready() → sendAT("AT+SETUWBMODE=1")
        → sendAT("AT+PDOAOFF=" + String(PDOA_OFFSET_DEG))  ← ADD THIS
        → sendAT("AT+RNGOFF="  + String(RANGE_OFFSET_CM))  ← ADD THIS
        → sendAT("AT+SAVE")
        → ... (ADDTAG auto-registration)
```

Both commands must be sent **after** `AT+SETUWBMODE=1` and **before** `AT+USER_CMD=0`.

### B. AT+RNGOFF (range offset)

Corrects fixed group delay through BU04's PCB traces and STM32 firmware.

```
1. Set up a 1.000 m reference distance with a tape measure
   (measure from centre of BU04 dual-antenna midpoint to tag)
2. Flash with RANGE_OFFSET_CM=0
3. Run tools/calibrate.py --mode range --samples 30 --expected 100
   → Computes mean_cm, prints: RANGE_OFFSET_CM = 100 - <mean_cm>
4. Update config.h, reflash
5. Verify at 1 m, 2 m, 3 m — expect residual ≤ ±10 cm
```

### C. Re-verification grid test

After both offsets are set, run a 3×3 grid:

```
       −1 m    0°    +1 m
  3 m:  [SW]  [S]   [SE]
  2 m:  [W]   [C]   [E]
  1 m:  [NW]  [N]   [NE]

For each of 9 points: collect 30 samples, log expected vs measured
Accept if: |Δrange| ≤ 10 cm, |Δangle| ≤ 15° in LOS
```

---

## Suggested Build Order

Dependencies create a strict ordering. Each step unlocks the next.

```
Step 1: Auto tag registration
  ─ Firmware: scan PDOA stream for new Tag_Addr; if not in registered list,
    pause USER_CMD, send AT+ADDTAG=<addr>, resume USER_CMD
  ─ Unblocks: all subsequent testing (removes mandatory reboot manual step)
  ─ Risk: BU04 returns ERR if tag addr format wrong; need AT指令 V1.0.6 §ADDTAG

Step 2: Send PDOAOFF + RNGOFF at startup
  ─ Firmware: add two sendAT() calls in setup(), after SETUWBMODE, before USER_CMD
  ─ Even with value=0, verifies commands are accepted and not rejected as ERR
  ─ Unblocks: calibration workflow

Step 3: Python visualizer (tools/visualizer.py)
  ─ Serial thread + deque + FuncAnimation polar scatter
  ─ Unblocks: visual sanity check during calibration; needed for Step 4

Step 4: Calibration procedure (tools/calibrate.py + empirical runs)
  ─ Run calibrate.py --mode angle, then --mode range
  ─ Update config.h PDOA_OFFSET_DEG and RANGE_OFFSET_CM
  ─ Reflash
  ─ Unblocks: accuracy validation

Step 5: Accuracy validation
  ─ 3×3 grid test, CSV log, post-analysis in Python (pandas optional)
  ─ Gate: must pass ±10 cm / ±15° before robot phase starts
  ─ Unblocks: v2 robot integration

Step 6: Robot build environment + H-bridge wiring
  ─ New platformio.ini env: robot_follower (-DROBOT_MODE=1)
  ─ Wire H-bridge to ESP32-C3 GPIO (4 GPIO: IN1/IN2 left, IN3/IN4 right)
  ─ Add motor PWM block to main.cpp (#ifdef ROBOT_MODE)
  ─ Start with P-controller, tune K_ANGLE / K_RANGE on bench

Step 7: Field test (following)
  ─ Walk away / circle robot, verify tracking
  ─ Add dead-band, safety timeout (no PDOA → stop)
```

---

## Integration Points

| Interface | Protocol | Rate | Owner | Notes |
|-----------|----------|------|-------|-------|
| BU04 ↔ ESP32 UART | AT commands (ASCII, CRLF) | 115200 baud | ESP32 drives; BU04 responds | UART1, GPIO2/3. BU04 streams unsolicited after AT+USER_CMD=0. |
| ESP32 ↔ PC USB | CSV lines (ASCII, CRLF) | 115200 baud (virtual CDC) | ESP32 writes; Python reads | `/dev/ttyACMx`. DTR-reset: open with `ser.setDTR(False)`. |
| ESP32 ↔ H-bridge | GPIO (direction) + ledc PWM | ~1 kHz PWM | ESP32 only | L298N: IN1/IN2/ENA per side. DRV8833: IN1/IN2 per side (PWM directly). |
| BU04 tag ↔ BU04 anchor | UWB radio (DW3000) | ~5 Hz PDOA | BU04 hardware | 6.8 Mbps, CH5 (6489.5 MHz). Air-side is invisible to firmware — BU04 handles it internally. |

**No cross-board wireless in PDOA mode.** ESP-NOW is only used in TWR mode (ANCHOR2 → ANCHOR1). PDOA mode has one anchor that does everything.

---

## Key Architectural Decisions

| Decision | Rationale |
|----------|-----------|
| Motor control on anchor ESP32-C3, not a separate MCU | No additional hardware available; latency from PDOA to motor is acceptable at 5 Hz; adding a second MCU + wireless link adds failure modes |
| Static calibration constants in config.h (not AT runtime) | Simplest path: PDOAOFF/RNGOFF sent at startup; recalibrate by reflashing; no EEPROM/NVS needed |
| Python tools live in `tools/` directory, not as part of firmware | Clear separation; PC tools can be changed without reflashing; Python is sufficient for 5 Hz data rate |
| Single main.cpp with compile-time role selection | Existing pattern; adding `#ifdef ROBOT_MODE` inside `#ifdef ROLE_ANCHOR1 && PDOA_MODE` extends cleanly without new files |
| PDOA instead of TWR for positioning | DW3000 PDOA gives angle + distance from one device; TWR with two anchors adds cable runs, ESP-NOW complexity, and synchronisation problems — overkill for a robot-following use case |
| 5-sample moving average on range, 3-sample on angle | Range benefits from more smoothing (cm-level noise); angle needs lower latency for responsive steering |

---

## Pitfall Cross-Reference

| Architectural choice | Linked pitfall |
|---------------------|----------------|
| Motor control on anchor ESP32 | ESP32-C3 has limited GPIO; verify enough free pins after BU04 UART is claimed (GPIO2/3 taken; USB uses GPIO18/19; leaves GPIO0,1,4,5,6,7,8,10 — enough for H-bridge) |
| PDOAOFF before USER_CMD | If sent after USER_CMD=0, BU04 may ignore the command (event loop is already running). Send order: SETUWBMODE → PDOAOFF → RNGOFF → SAVE → USER_CMD=0 |
| Auto ADDTAG scanning | BU04 PDOA stream must be parsed for unknown Tag_Addr only when USER_CMD=0 is active. Must pause stream (AT+USER_CMD=1?) or handle AT+ADDTAG async without interrupting the line reader |
| P-controller with 5 Hz PDOA | At 5 Hz there is 200 ms latency minimum. Keep K_ANGLE low to avoid oscillation on slow feedback. Consider reducing POLL_INTERVAL_MS if BU04 supports faster streaming |

---

*Sources: codebase map (.planning/codebase/ARCHITECTURE.md), Qorvo DW3000 product page, BU04 AT指令 V1.0.6 (referenced in config.h comments), STACK.md calibration procedure section.*
