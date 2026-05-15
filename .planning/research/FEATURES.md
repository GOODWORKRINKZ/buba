# Features Research — UWB Test Bench & Robot Following

**Project:** BU04 UWB Positioning & Robot Following  
**Hardware constraint:** AT-command-only BU04 (DW3000 behind STM32F103), no direct SPI  
**Researched:** 2026-05-15  
**Overall confidence:** HIGH (based on existing codebase, AT-command spec, UWB calibration practice)

---

## Table Stakes (must have — system is broken without these)

### TS-1: PDOA telemetry stream, stable at ≥ 5 Hz
**What:** Firmware emits `PDOA,addr,seq,range_m,angle_deg,x_m,y_m,ts_ms` over USB Serial 115200  
without crashing, losing packets, or triggering ESP32 WDT resets.  
**Why must-have:** If the data stream stops or corrupts, every downstream consumer (visualizer, logger,
robot) is blind. The stream is the only interface between hardware and software.  
**Complexity:** Low — existing in `src/main.cpp` `anchor1_pdoa` env, but needs smoke testing at 200 ms
poll rate for ≥ 60 s without reboot.  
**Status in codebase:** Exists. `g_seq` uint8 overflows at ~51 s — known, not critical for bench test.

---

### TS-2: Calibration offsets applied at firmware startup
**What:** `AT+PDOAOFF=<value>` (angle offset, units = internal PDOA ticks) and
`AT+RNGOFF=<value>` (range offset in cm) sent to BU04 every cold boot before measurements start.  
**Why must-have:** BU04 DW3000 PDOA angle has a systematic offset from antenna placement and PCB
geometry. Without correction the angle reading is typically 15–40° off from ground truth — immediately
fails the ±15° requirement. The constants exist in `config.h` but are **not currently sent** in
`main.cpp`.  
**Complexity:** Low — send two AT commands in `setup()` after `configureBU04()`.  
**Status in codebase:** `AT_PDOAOFF` / `AT_RNGOFF` macros defined in `config.h`. Initial values must be
determined empirically (see TS-6). Sending them is a one-liner.  
**Source:** Qorvo APS014 "Antenna Delay Calibration"; BU04 AT指令 V1.0.6 section 4.

---

### TS-3: Automatic tag registration after every anchor reboot
**What:** On startup, anchor queries `AT+GETDLIST` to discover broadcasting tags, picks the first
(or configured) tag long address, and issues `AT+ADDTAG=<LongAddr64>,<ShortAddr>,10,20,0`
automatically.  
**Why must-have:** Current firmware requires manual `AT+ADDTAG` entry every time the anchor reboots.
In a test session with multiple reboots (calibration iterations, firmware updates) this destroys
repeatability and wastes 5–10 min per session.  
**Complexity:** Medium — parse `AT+GETDLIST` response, extract LongAddr64, issue ADDTAG, confirm with
`AT+GETKLIST`. Must handle "no tags found yet, retry" with timeout.  
**Status in codebase:** NOT implemented. `AT_GETDLIST` / `AT_GETKLIST` macros exist but no parsing
logic.

---

### TS-4: Python real-time polar plot (distance + angle)
**What:** Python script reads CSV over serial port, renders live polar scatter plot where
r = range_m and θ = angle_deg. Updates at the full stream rate (≥5 Hz visible refresh).  
**Why must-have:** Without visualization you cannot see whether PDOA is tracking the tag correctly
or outputting noise. A polar plot is the natural representation for (range, angle) data — Cartesian
x/y is harder to read when verifying angular accuracy at fixed distances.  
**Complexity:** Low — `pyserial` + `matplotlib` polar axes with `FuncAnimation`. Pattern is standard;
the Makerfabs ESP32-UWB repo uses identical approach with UDP instead of serial.  
**Status in codebase:** NOT implemented. PROJECT.md marks as "Active".  
**Minimum viable plot:**
- Polar axes, range 0–3 m, angle −90° to +90°
- Current point highlighted, last N=50 points faded
- Text overlay: seq, range_m, angle_deg, update rate

---

### TS-5: CSV data logging (PC-side, session file per run)
**What:** Python logger writes every received PDOA line to
`logs/YYYYMMDD_HHMMSS.csv` with no buffering delay.  
**Why must-have:** The v1 accuracy requirement ("verify ±10 cm at 1–3 m, ±15° at 90° rotation")
cannot be verified in real time by eye. You need a recorded dataset to compute RMSE after the
grid test. Without logs, the test bench produces no evidence.  
**Complexity:** Low — open file in append mode in the same Python script as the visualizer.  
**Note:** Can be integrated into the visualizer script (--log flag) rather than a separate tool.

---

### TS-6: Calibration procedure — empirical PDOA offset determination
**What:** A defined 3-step procedure:
1. Place tag at 0° (dead ahead of antenna midpoint) at 1.5 m.
2. Record 200 angle readings, compute mean error vs 0°.
3. Write result as `PDOA_ANGLE_OFFSET_DEG` in `config.h`; convert to BU04 internal ticks and send
   via `AT+PDOAOFF`.  
Repeat with tag at −45°, 0°, +45° to verify linearity.  
**Why must-have:** PDOA offset is hardware-specific per module (antenna spacing, PCB layout). The
BU04 datasheet quotes typical ±20° uncalibrated error. Without a calibration step the ±15°
target is not achievable.  
**Complexity:** Medium — procedure design (Low), computing offset from logs (Low), validating
linearity (Medium — may require iterative adjustment if offset is nonlinear).  
**Status in codebase:** `PDOA_ANGLE_OFFSET_DEG = 0` and range constants defined in `config.h` as
placeholders.

---

### TS-7: Accuracy grid test with documented pass/fail verdict
**What:** Test tag at a 3×3 grid of positions: ranges {1.0, 1.5, 2.5} m × angles {−45°, 0°, +45°}.
Record 100 readings per position. Compute mean error and RMSE. Pass if all positions meet
±10 cm / ±15°.  
**Why must-have:** This is the explicit v1 goal. Without it the project has no completion criterion.  
**Complexity:** Medium — the measurement is Low, the fixture (known-position jig or tape measure) is
Medium, the analysis script (RMSE from CSV) is Low.

---

## Differentiators (nice to have — add value without being critical)

### D-1: Exponential moving average (EMA) filter on angle
**What:** Apply EMA (α ≈ 0.2) in firmware or Python to smooth PDOA angle noise.  
**Why valuable:** Raw PDOA angle from DW3000 has ≈ 5–8° RMS jitter per reading. EMA reduces jitter
for the robot following use case without adding latency bias like a longer window MA.  
**Complexity:** Low — one-liner in both firmware and Python.  
**Note:** 5-sample MA already exists in firmware for range. Angle needs the same or better.

---

### D-2: Polar plot error tolerance rings
**What:** Draw dashed circles at ±10 cm and sector arcs at ±15° on the polar plot to show
pass/fail zones visually during the grid test.  
**Why valuable:** Makes the test bench demonstration self-explanatory. A reviewer can instantly see
whether readings fall inside tolerance without computing numbers.  
**Complexity:** Low — matplotlib polar axis patches.

---

### D-3: Seq number continuity check + packet loss display
**What:** Python visualizer tracks sequence number delta. Displays packet loss %, warns if gap > 1.  
**Why valuable:** Distinguishes "PDOA angle noise" from "PDOA packet drops". Both look like jumps
in the plot. g_seq overflows at 255 (~51 s at 200 ms) so the check must handle wraparound.  
**Complexity:** Low.

---

### D-4: Angle histogram at fixed position
**What:** Accumulate N=500 angle readings at one position and display distribution (histogram or
boxplot) in the Python tool.  
**Why valuable:** Quantifies the noise floor at a specific pose rather than just RMSE across
positions. Useful for understanding whether error is systematic (offset) or random (noise).  
**Complexity:** Low.

---

### D-5: `AT+FILTER` enable (BU04 built-in Kalman filter)
**What:** Send `AT+FILTER=1` (if supported by BU04 PDOA mode firmware) at startup to enable
BU04's internal smoothing.  
**Why valuable:** Zero implementation cost if the AT command is accepted in PDOA mode. May reduce
angle jitter by 30–50% without PC-side processing.  
**Complexity:** Low to verify, outcome uncertain — requires testing whether `AT+FILTER` is
accepted in `SETUWBMODE=1` mode.  
**Confidence:** MEDIUM — AT命令 doc lists the command but doesn't confirm PDOA mode support.

---

### D-6 (v2): Dead zone configurable at runtime via Serial command
**What:** Python or serial terminal can send a command like `DEADZONE=0.4` to change the
robot's dead zone radius without reflashing.  
**Why valuable:** Finding the right dead zone for a particular robot/floor requires tuning. Runtime
config avoids the PlatformIO → flash → test cycle.  
**Complexity:** Low — parse one parameter from Serial in firmware.

---

### D-7 (v2): Motor PWM ramp (soft start)
**What:** Linearly ramp PWM from 0 to target over 100–200 ms when starting from stopped.  
**Why valuable:** Prevents wheel slip on smooth floors, reduces mechanical stress, eliminates the
"lurch" behavior that makes UWB-following robots feel unstable.  
**Complexity:** Low — track current PWM, step toward target each loop iteration.

---

## Anti-Features (deliberately NOT building)

| Anti-Feature | Why Avoid | What to Do Instead |
|---|---|---|
| Wi-Fi / UDP transport | Adds AP setup complexity, no benefit for wired test bench | USB Serial 115200 baud is sufficient; cable = power + data |
| Multiple simultaneous tags | Complicates firmware parsing, not needed to verify PDOA accuracy | Single-tag PDOA is the v1 test scope |
| TWR two-anchor triangulation | Requires ESP-NOW, two separate anchor positions, baseline measurement | PDOA with dual-antenna single anchor is the chosen approach |
| Web dashboard / browser UI | Three extra dependencies (Flask/MQTT/WebSocket) for zero accuracy benefit | matplotlib polar plot is sufficient for a test bench |
| OLED display on module | ESP32-C3 SuperMini has no display connector; adds wiring | PC Serial Monitor + Python visualizer is the display |
| Persistent tag pairing (NVM) | BU04 saves to NVM with AT+SAVE, but NVM-based pairing doesn't help if tag address changes | Auto-discovery via AT+GETDLIST at every boot is more robust |
| SLAM / map building | Far beyond scope; requires odometry, not possible with 1 anchor | Tag position is (r, θ) polar — that's the product |
| IMU fusion for orientation | No IMU on BU04 or ESP32-C3 SuperMini | PDOA angle is the sole orientation input; fuse only when/if HW is added |
| BLE / NFC pairing for tag | Tag is identified by UWB long address; separate wireless pairing is over-engineering | Long address is fixed per module, store in `config.h` as fallback |
| PID position controller (v2) | Overkill for "follow at fixed distance" behavior | Bang-bang with proportional speed scaling is sufficient for v2 |

---

## Feature Dependencies

```
TS-1 (PDOA stream)
  └── TS-4 (polar plot)         — visualizer reads stream
  └── TS-5 (CSV logging)        — logger records stream
  └── TS-7 (grid test)          — test consumes logged data

TS-2 (calibration offsets sent)
  └── TS-6 (empirical procedure) — must determine offset values first
       └── TS-5 (CSV logging)    — need logged data to compute mean error
       └── TS-4 (polar plot)     — useful to see offset visually while calibrating

TS-3 (auto tag registration)
  └── TS-1 (PDOA stream)        — stream only starts after tag is registered

TS-6 (offset values known)
  └── TS-2 (offsets applied)    — chicken-and-egg: start with offset=0, iterate

[v2 robot following prerequisites]
TS-7 (v1 accuracy verified)
  └── v2 firmware (PWM commands) — don't start v2 until PDOA meets spec
       └── D-6 (runtime deadzone tuning)
       └── D-7 (soft start ramp)
```

---

## MVP Recommendation

**v1 test bench MVP (minimum to complete the phase):**  
Prioritize in order:

1. **TS-3** — Auto tag registration (unblocks everything else; without it every reboot needs manual intervention)
2. **TS-2** — Send calibration offsets at startup (even with zero values; establishes the hook)
3. **TS-4 + TS-5** — Combined Python script: polar plot + CSV logger in one file
4. **TS-6** — Run empirical calibration to find PDOA offset values
5. **TS-7** — Execute grid test and record pass/fail result

**Defer to v2 (confirmed out of scope for v1):**  
- All D-6, D-7 features (robot-specific)  
- D-5 (`AT+FILTER`) — test it, but don't block on it

**Nice-to-have for v1 (build only if time allows):**  
- D-1 (EMA on angle) — 10 min to add  
- D-2 (tolerance rings on plot) — 20 min to add  
- D-3 (packet loss display) — 15 min to add  

---

## Sources

| Source | Confidence | Notes |
|---|---|---|
| `src/main.cpp` + `include/config.h` (this repo) | HIGH | Ground truth for current state |
| `.planning/PROJECT.md` (this repo) | HIGH | Active requirements and out-of-scope decisions |
| BU04 AT指令 V1.0.6 (embedded in code comments) | HIGH | AT command spec, PDOA mode behavior |
| Qorvo APS014 "Antenna Delay Calibration" (2024) | HIGH | Calibration methodology for DW3000-based products |
| Makerfabs ESP32-UWB repo (DW1000, similar pattern) | MEDIUM | Shows standard pyserial+matplotlib pattern; DW1000 ≠ DW3000 |
| UWB robot following community implementations | MEDIUM | Dead zone, speed scaling, loss-of-signal stop are universal patterns |
