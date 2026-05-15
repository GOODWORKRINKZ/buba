# Research Summary — BU04 UWB PDOA Test Bench

**Synthesized:** 2026-05-15  
**Sources:** STACK.md · FEATURES.md · ARCHITECTURE.md · PITFALLS.md  
**Overall confidence:** HIGH (v1 test bench) / MEDIUM (v2 robot — hardware not yet integrated)

---

## Recommended Stack

| Layer | Choice | Status |
|-------|--------|--------|
| Serial I/O | pyserial 3.5 | Installed |
| Visualization | matplotlib 3.10 polar + FuncAnimation | Installed |
| CSV logging | Python stdlib `csv` | No extra dep |
| Firmware | PlatformIO + Arduino on ESP32-C3 | Working |
| UWB module | Ai-Thinker BU04 (DW3000 + STM32F103 AT-bridge) | Purchased |

**PC-side pattern:** daemon reader thread → `deque(maxlen=200)` ring buffer → `FuncAnimation`
(100 ms interval). Use artist `.set_offsets()` instead of `ax.clear()` to avoid CPU spikes (P-6).
Open serial port with `dtr=False` to prevent ESP32-C3 reset on connect (P-3).

**Do NOT add:** Plotly Dash, pyqtgraph (defer to robot phase), Wi-Fi/UDP, web dashboard, SLAM, IMU fusion.

---

## Table Stakes Features

| ID | Feature | Status | Blocker? |
|----|---------|--------|----------|
| TS-1 | PDOA telemetry stream stable ≥ 5 Hz, no WDT crash | Exists, untested at duration | — |
| TS-2 | `AT+PDOAOFF` + `AT+RNGOFF` sent in `setup()` | **Bug: NOT sent** | Yes — offsets silently zero every boot |
| TS-3 | Auto tag registration via `AT+GETDLIST` after reboot | **Not implemented** | Yes — stream dead after every reboot |
| TS-4 | Python real-time polar plot (r = range, θ = angle) | Not implemented | — |
| TS-5 | CSV session log (`logs/YYYYMMDD_HHMMSS.csv`) | Not implemented | — |
| TS-6 | Empirical calibration procedure (angle & range offsets) | Constants are placeholder 0 | — |
| TS-7 | 3×3 grid accuracy test, pass/fail vs ±10 cm / ±15° | Not implemented | — |

**Nice-to-have (v1):** EMA angle filter (D-1), plot tolerance rings (D-2), seq continuity / packet-loss display (D-3).  
**Nice-to-have (v2):** Runtime dead-zone config (D-6), motor PWM ramp (D-7).

---

## Suggested Architecture

```
BU04 anchor (DW3000 PDOA)
  │ UART1 115200, GPIO 2/3
  ▼
ESP32-C3 anchor firmware  ←── config.h (PDOA_OFFSET_DEG, RANGE_OFFSET_CM, …)
  │ AT seq: SETUWBMODE → PDOAOFF → RNGOFF → ADDTAG → USER_CMD
  │ loop: fieldVal parser → 5-sample MA → CSV printf
  │ [v2] #ifdef ROBOT_MODE → P-controller → ledc PWM → H-bridge
  │ USB CDC /dev/ttyACM0
  ▼
Python tools/ (PC)
  ├── visualizer.py  — serial thread, deque, FuncAnimation polar scatter
  └── calibrate.py   — collect N samples at known pose, compute offsets
```

**v2 robot:** same ESP32-C3 anchor, new `robot_follower` build env. PDOA parser unchanged;
`#ifdef ROBOT_MODE` block adds P-controller output to H-bridge. Tag firmware unchanged.

**Boundary rule:** ESP32-C3 is the only BU04↔PC bridge. Direct PC→BU04 serial only in `test_bu04` env.

---

## Watch Out For (top 5 pitfalls)

1. **P-1 (CRITICAL) — Calibration offsets never sent.**  
   `AT+PDOAOFF` / `AT+RNGOFF` defined in `config.h` but absent from `setup()`. Every boot produces
   15–40° angle error and 5–15 cm range bias. Fix: send both after PDOA mode confirm, before `USER_CMD`.

2. **P-2 (CRITICAL) — Tag registration lost on every reboot.**  
   `AT+ADDTAG` not called automatically. After any firmware flash or power cycle, the PDOA CSV stream
   never resumes until a human types `AT+ADDTAG` in Serial Monitor. Fix: poll `AT+GETDLIST` in
   `setup()`, extract addr, issue `AT+ADDTAG`, confirm with `AT+GETKLIST`.

3. **P-3 (HIGH) — pyserial DTR asserts ESP32-C3 reset on port open.**  
   Opening the serial port reboots the anchor; first 15 s of data is lost and P-2 fires again.
   Fix: `ser.dtr = False` before `ser.open()`.

4. **P-4 (HIGH) — PDOA saturates silently at ±60°.**  
   Beyond ±60° boresight the DW3000 clamps angle to ±60 with no error flag. For robot following
   this causes a "death spiral". Fix: detect N consecutive ±60 readings → stop motors / raise flag.

5. **P-5 (MEDIUM) — BU04 answers "OK" before DW3000 is ready.**  
   STM32F103 AT parser is up in ~800 ms; DW3000 UWB chip needs 2–4 s more. Keep the existing
   `delay(3000)` post-ready; never remove it as an "optimization".

**P-6 (LOW):** `ax.clear()` in FuncAnimation kills CPU. Use `.set_offsets()` instead.

---

## Build Order

### Phase 0 — Firmware smoke test (foundation)
Validate TS-1: 60 s continuous PDOA stream, no WDT, correct CSV format.
Fix P-5 guard (document the 3 s delay). **Dependency for everything.**

### Phase 1 — Calibration & firmware fixes (unblocks accuracy work)
Implement TS-2 (send PDOAOFF/RNGOFF in `setup()`) and TS-3 (auto `AT+GETDLIST` + `AT+ADDTAG`).
Run TS-6 empirical offset procedure; write confirmed values to `config.h`.
Avoids P-1 and P-2. **Must complete before any accuracy measurement.**

### Phase 2 — Python visualizer + logger (enables test bench workflow)
Implement TS-4 (polar plot with artist updates, not `ax.clear()`) and TS-5 (CSV log).
Apply P-3 fix (`dtr=False`). Add D-2 tolerance rings and D-3 seq continuity check.
**Required for Phase 3.**

### Phase 3 — Accuracy grid test (v1 completion criterion)
Implement TS-7: 3×3 grid, 100 readings/position, RMSE script, documented pass/fail verdict.
Optionally add D-1 EMA filter if jitter is too high. **v1 done when this passes.**

### Phase 4 — Robot following (v2)
New `robot_follower` build env on same ESP32-C3. P-controller → ledc PWM → H-bridge.
Add P-4 saturation guard (±60° → stop). Tune K_ANGLE, K_RANGE, FOLLOW_DIST on bench first.
Add D-6 (runtime dead-zone config) and D-7 (PWM ramp) during this phase.

---

## Open Questions

| # | Question | Impact | How to resolve |
|---|---------|--------|---------------|
| Q-1 | What is `PDOA_OFFSET_DEG` for this specific BU04 unit? | HIGH — ±15° accuracy impossible without it | Run Phase 1 calibration procedure |
| Q-2 | Does `AT+FILTER=1` work in `SETUWBMODE=1` (PDOA) mode? | LOW–MEDIUM — free noise reduction if it works | Test in `test_bu04` env, check AT指令 V1.0.6 |
| Q-3 | Exact `AT+RNGOFF` unit — centimetres or decimetres? | MEDIUM — wrong unit = wrong correction | Send `AT+RNGOFF=10`, measure actual range change |
| Q-4 | Does the existing 5-sample MA on range need the same for angle? | LOW — EMA (D-1) is the preferred fix | Profile raw angle jitter in Phase 0 |
| Q-5 | H-bridge model for v2 (L298N vs DRV8833)? | LOW for v1 | Decide before Phase 4; DRV8833 preferred (lower voltage drop) |
