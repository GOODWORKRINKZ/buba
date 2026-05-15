# Roadmap: BU04 UWB PDOA Test Bench

**Milestone:** UWB Test Bench v1
**Goal:** Verify that the BU04 PDOA anchor reliably measures distance (±10 cm) and angle (±15°) to a tag at 1–3 m on a physical test bench.
**Success:** 3×3 grid accuracy test passes — RMSE < 10 cm (range) and < 15° (angle) at all 9 grid positions, documented with CSV evidence.

---

## Phases

- [ ] **Phase 1: Firmware Fixes** — Fix the 2 critical boot bugs so PDOA streams reliably after every reboot
- [ ] **Phase 2: Python Visualizer** — Real-time polar plot and CSV logger so a human can watch measurements live
- [ ] **Phase 3: Calibration** — Empirically measure PDOA_OFFSET_DEG and RANGE_OFFSET_CM, write them into firmware
- [ ] **Phase 4: Accuracy Verification** — Formal 3×3 grid test with RMSE analysis, produce pass/fail verdict

---

## Phase Details

### Phase 1: Firmware Fixes

**Goal:** PDOA telemetry streams continuously after every power cycle — no manual AT commands required, calibration offsets applied.
**Depends on:** Nothing (first phase)
**Requirements:** FW-01, FW-02, FW-03, FW-04, FW-05

**Deliverables:**
- Modified `src/main.cpp` (anchor1_pdoa env): `AT+PDOAOFF` and `AT+RNGOFF` sent in `setup()` before `AT+USER_CMD=0`
- Auto tag registration: `AT+GETDLIST` poll loop in `setup()`, addr parsed, `AT+ADDTAG` issued, `AT+GETKLIST` confirms
- `AT+SAVE` called after successful registration so config survives tag power-cycle
- Startup Serial diagnostic block: BU04 firmware version, PDOA mode status, registered tag address

**Plans:**
- [ ] Plan 1.1: Calibration offset send — add `sendAT("AT+PDOAOFF=...")` and `sendAT("AT+RNGOFF=...")` after PDOA mode confirm, before USER_CMD
- [ ] Plan 1.2: Auto tag registration — implement `getdlist_and_register()` in setup(): poll AT+GETDLIST up to 5×, parse LongAddr64, call AT+ADDTAG, verify with AT+GETKLIST, call AT+SAVE
- [ ] Plan 1.3: Startup diagnostics — Serial.printf banner with BU04 version (AT+VERSION), PDOA status, registered addr, PDOA_OFFSET_DEG / RANGE_OFFSET_CM values
- [ ] Plan 1.4: Stream stability test — 60 s burn-in, verify ≥ 5 Hz CSV output with no WDT reset, log to Serial Monitor

**UAT:**
- [ ] Power-cycle anchor, open Serial Monitor — startup prints BU04 version, PDOA enabled, tag address (no manual AT input)
- [ ] Stream runs ≥ 60 s at ≥ 5 Hz — count CSV lines in terminal (`grep -c "^PDOA"`) → ≥ 300 lines
- [ ] Power-cycle tag while anchor runs — anchor re-registers tag automatically within 10 s
- [ ] Serial Monitor shows `PDOAOFF=<non-zero>` and `RNGOFF=<non-zero>` confirmation lines on boot (placeholder values ok; will be replaced after Phase 3 calibration)

**Plans:** TBD

---

### Phase 2: Python Visualizer

**Goal:** A Python script shows a live polar plot of range/angle and writes every measurement to a timestamped CSV file.
**Depends on:** Phase 1 (stable firmware stream)
**Requirements:** VIZ-01, VIZ-02, VIZ-03, VIZ-04, VIZ-05

**Deliverables:**
- `tools/visualizer.py`: pyserial daemon thread → deque(maxlen=200) → FuncAnimation polar scatter (`.set_offsets()`, not `ax.clear()`)
- Serial port opened with `dtr=False` to prevent ESP32-C3 reset on connect
- `logs/YYYYMMDD_HHMMSS.csv` created per session with columns `timestamp,range_m,angle_deg,seq`
- Tolerance rings on polar plot at target distances (1 m, 2 m, 3 m ± 10 cm)
- "LOST" overlay text when no packets received for > 1 s; clears on packet resume

**Plans:**
- [ ] Plan 2.1: Serial reader thread — daemon thread opens port with `dtr=False`, reads lines, parses `PDOA,...` CSV, pushes to `collections.deque(maxlen=200)`
- [ ] Plan 2.2: FuncAnimation polar plot — `fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})`, scatter artist, `FuncAnimation` at 100 ms interval calling `.set_offsets()` on artist
- [ ] Plan 2.3: CSV session logger — open `logs/YYYYMMDD_HHMMSS.csv` on start, `csv.writer` writes row per packet from reader thread; flush every 10 rows
- [ ] Plan 2.4: Tolerance rings + signal loss — draw dashed circles at [1.0, 2.0, 3.0] m with ±0.1 m shading; track `last_packet_time`, overlay red "LOST" text if gap > 1 s

**UAT:**
- [ ] `python tools/visualizer.py /dev/ttyACM0 115200` — polar plot window opens, dot moves as tag moves; no ESP32 reset on open
- [ ] Move tag to ~1 m, 0° — dot appears near 1 m ring; hold for 5 s → CSV log grows (check `wc -l logs/*.csv`)
- [ ] Unplug tag for 2 s → "LOST" appears on plot; reconnect → "LOST" disappears within 1 s
- [ ] After 30 s session: `head -3 logs/*.csv` shows header + rows; `tail -1` shows recent timestamp

**Plans:** TBD
**UI hint**: yes

---

### Phase 3: Calibration

**Goal:** PDOA_OFFSET_DEG and RANGE_OFFSET_CM are empirically measured values (not zeros), written into config.h and deployed to firmware.
**Depends on:** Phase 2 (CSV logger needed for sample collection)
**Requirements:** CAL-01, CAL-02, CAL-03

**Deliverables:**
- `tools/calibrate.py`: places tag at known pose, collects 100 samples, computes mean offset, prints recommended `AT+PDOAOFF` and `AT+RNGOFF` values
- `include/config.h` updated with measured `PDOA_OFFSET_DEG` and `RANGE_OFFSET_CM` (non-zero)
- `docs/calibration_procedure.md`: step-by-step physical setup, command sequence, how to re-run calibration

**Plans:**
- [ ] Plan 3.1: calibrate.py angle offset — place tag at 0°, 1 m; collect 100 samples; print `mean_angle_deg` → this is PDOA_OFFSET_DEG (send as `AT+PDOAOFF=<value>`)
- [ ] Plan 3.2: calibrate.py range offset — place tag at exactly 1.000 m (measured with tape); collect 100 samples; compute `1000 - mean_range_mm` → this is RANGE_OFFSET_CM (units: verify Q-3 from research)
- [ ] Plan 3.3: Write offsets to config.h — update `PDOA_OFFSET_DEG` and `RANGE_OFFSET_CM` in `include/config.h`, rebuild and flash `anchor1_pdoa`, confirm startup banner shows new values
- [ ] Plan 3.4: Calibration docs — write `docs/calibration_procedure.md`: physical jig description, step-by-step commands, expected terminal output, how to interpret results

**UAT:**
- [ ] `python tools/calibrate.py /dev/ttyACM0 --samples 100 --mode angle` — prints `Recommended PDOA_OFFSET_DEG: X.X` after 100 samples
- [ ] `python tools/calibrate.py /dev/ttyACM0 --samples 100 --mode range` — prints `Recommended RANGE_OFFSET_CM: X.X` after 100 samples
- [ ] After reflashing with measured offsets: tag at 0° reads angle within ±5° of 0°; tag at 1 m reads range within ±5 cm of 1.000 m (pre-accuracy-test sanity check)
- [ ] `docs/calibration_procedure.md` exists and describes the physical setup with a diagram or measurement sketch

**Plans:** TBD

---

### Phase 4: Accuracy Verification

**Goal:** Formal 3×3 grid test passes — RMSE < 10 cm and < 15° at all grid positions — with documented evidence.
**Depends on:** Phase 3 (calibrated firmware)
**Requirements:** ACC-01, ACC-02, ACC-03, ACC-04

**Deliverables:**
- 9 CSV log files (`logs/grid_Xm_Ydeg.csv`), 100 samples each
- RMSE analysis script or inline analysis in `tools/calibrate.py`
- `docs/accuracy_test.md`: grid diagram, per-cell RMSE table, pass/fail verdict per cell, overall verdict
- Physical setup documented: module heights, antenna orientation, distances to metal surfaces

**Plans:**
- [ ] Plan 4.1: Grid measurement collection — for each of 9 positions (1 m/2 m/3 m × 0°/+45°/−45°): place tag, run `python tools/visualizer.py --record --output logs/grid_Xm_Ydeg.csv --count 100`, record 100 samples
- [ ] Plan 4.2: RMSE analysis — script or notebook: load each CSV, compute RMSE(range) and RMSE(angle) vs ground truth, print pass/fail per cell (threshold: range < 0.10 m, angle < 15°)
- [ ] Plan 4.3: accuracy_test.md — write `docs/accuracy_test.md`: grid diagram (ASCII or image), table of per-cell RMSE, pass/fail per cell, overall verdict, date and BU04 unit ID
- [ ] Plan 4.4: Physical conditions doc — add section to accuracy_test.md: module mounting height, anchor antenna plane orientation, nearest metal object distances, LOS confirmation, tag height

**UAT:**
- [ ] All 9 `logs/grid_*.csv` files exist with ≥ 100 rows each (`wc -l logs/grid_*.csv`)
- [ ] RMSE script prints per-cell results: every cell shows `range_rmse < 0.10 m` and `angle_rmse < 15.0 deg` → overall PASS
- [ ] `docs/accuracy_test.md` contains: grid diagram, full RMSE table, explicit "PASS" or "FAIL" verdict
- [ ] Physical conditions section lists: anchor height (cm), nearest metal surface distance (cm), LOS confirmed Y/N

**Plans:** TBD

---

## Progress Table

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. Firmware Fixes | 0/4 | Not started | - |
| 2. Python Visualizer | 0/4 | Not started | - |
| 3. Calibration | 0/4 | Not started | - |
| 4. Accuracy Verification | 0/4 | Not started | - |

---

## v2 Phases (future milestone)

### Phase 5: Robot Following

**Goal:** Anchor ESP32-C3 drives two wheel motors via H-bridge: robot follows tag at ~0.5 m, turns toward angle error, stops on signal loss.
**Depends on:** Phase 4 passing accuracy test
**Requirements:** ROB-01, ROB-02, ROB-03, ROB-04, ROB-05, ROB-06, ROB-07
**Status:** Deferred — begins after Phase 4 passes accuracy test

**Key work (deferred):**
- New `robot_follower` PlatformIO env on same ESP32-C3
- `#ifdef ROBOT_MODE` P-controller: speed ∝ (range − 0.5 m), differential steering ∝ angle
- Dead zone ±10°, saturation guard ±60° → stop motors
- Ramp PWM on start/stop (D-7 from research)
- Start/stop button on ESP32-C3 GPIO
- Decide H-bridge model before starting (DRV8833 preferred over L298N — research Q-5)

---

## Traceability

| Requirement | Phase | Plans |
|-------------|-------|-------|
| FW-01 | 1 | 1.1 |
| FW-02 | 1 | 1.2 |
| FW-03 | 1 | 1.2 |
| FW-04 | 1 | 1.4 |
| FW-05 | 1 | 1.3 |
| VIZ-01 | 2 | 2.1 |
| VIZ-02 | 2 | 2.2 |
| VIZ-03 | 2 | 2.3 |
| VIZ-04 | 2 | 2.4 |
| VIZ-05 | 2 | 2.4 |
| CAL-01 | 3 | 3.1, 3.4 |
| CAL-02 | 3 | 3.2, 3.4 |
| CAL-03 | 3 | 3.1, 3.2 |
| ACC-01 | 4 | 4.1, 4.2 |
| ACC-02 | 4 | 4.1, 4.2 |
| ACC-03 | 4 | 4.3 |
| ACC-04 | 4 | 4.4 |

**Coverage:**
- v1 requirements: 17 total
- Mapped to phases: 17
- Unmapped: 0 ✓

---

*Roadmap created: 2026-05-15*
*Last updated: 2026-05-15 after initialization*
