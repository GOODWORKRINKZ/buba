# Phase 3 → Phase 4 Handoff: Gray Area Resolutions

**Date:** 2026-06-10
**Audience:** Phase 4 (Calibration) + Phase 5 (Implementation) executor
**Status:** Recommendations provided; empirical validation needed in Phase 4

---

## G1: Triangle Side Length (a = ?)

**Question:** What should be the equilateral triangle side length for 3× BU04 anchors on robot?

**Research Evidence:**
- Patent CN105828431A: formula works for any a; a is in denominator (a²) — larger a reduces noise amplification
- ETH Zurich UWBTracker: 4 UWB on drone with rigid baseline ~25cm → 10cm accuracy at 4m
- Physical constraint: BU04 module is ~40×25mm, 3 modules need spacing for antenna isolation (≥5cm)

**PLANNER RECOMMENDATION:** **a = 40cm** as compromise.
- 40cm fits a ~50cm robot chassis (TurtleBot, custom RPi robot)
- Larger than ETH Zurich's 25cm → better angular resolution
- Patent formula: a² in denominator means 40cm gives 2.5× better noise rejection than 25cm
- If robot chassis is smaller, drop to 30cm with expected ~20% accuracy reduction

**OPEN for Phase 4:** Test TWR accuracy at both 30cm and 50cm baselines during calibration. Quantify accuracy-vs-size tradeoff. Final a determined by Phase 5 robot chassis.

---

## G2: RP2040 Poll Rate

**Question:** How fast should RP2040 poll 3× BU04 anchors for TWR distances?

**Research Evidence:**
- BU04 TWR cycle: ~100-200ms per single range (AT+DISTANCE latency, empirically from PDOA test bench)
- 3 anchors sequentially = 300-600ms → 1.6-3.3 Hz
- BUT: TWR can overlap if tag responds to all anchors — need to verify BU04 stock firmware behavior
- ETH Zurich: Active-Passive Ranging gets measurement rate independent of unit count → 10Hz+

**PLANNER RECOMMENDATION:** **Target 5Hz (200ms cycle).**
- Poll each anchor's UART in round-robin at 50ms intervals
- Collect d1,d2,d3 within a 200ms sliding window, solve formula once per window
- If anchors range in parallel (tag responds to all), effective rate could reach 10Hz
- RP2040 PIO at 115200 baud: 3× UART RX at 50ms poll is well within PIO bandwidth

**OPEN for Phase 4:** Measure actual AT+DISTANCE latency on BU04. Test if parallel-ranging works (3 anchors, 1 tag, all in TWR mode). Determine max achievable update rate empirically.

---

## G3: Output Protocol (ASCII vs Binary)

**Question:** RP2040→Robot output: ASCII "TAG:L,θ" or compact binary?

**Research Evidence:**
- ASCII format already designed: `TAG:150,25\r\n` (15 bytes), `TAG:LOST\r\n`, `TAG:NOISE\r\n`
- Single `scanf("%*3s%f,%f")` parse on robot side
- 15 bytes/packet × 5Hz = 75 bytes/sec at 115200 baud = 0.06% utilization — negligible

**PLANNER RECOMMENDATION:** **ASCII.** LOCKED.
- Human-debuggable with any serial terminal (critical for bring-up)
- Binary saves 11 bytes/packet — not worth losing debuggability for 0.05% bandwidth
- Robot-side parsing is trivial (single scanf)
- Only reconsider if robot MCU has extreme RAM constraints (<1KB)

**OPEN:** None. This decision is final unless robot hardware changes.

---

## G4: 2-of-3 Anchors Lost → Continue or Stop?

**Question:** If only 2 of 3 anchors have valid distance to tag, should robot continue or stop?

**Research Evidence:**
- Patent formula REQUIRES all 3 distances (closed-form analytic solution)
- With 2 distances: two possible intersection points (circles intersect at 2 points) — ±y ambiguity
- ETH Zurich: uses 4 receivers + IEKF to handle occlusion — but we have 3
- Anchor dropout causes: antenna null (BU04 ~120° beam), multipath, tag behind robot

**PLANNER RECOMMENDATION:** **STOP — output TAG:LOST.**
- Do NOT attempt 2-anchor estimation — ambiguity means robot could turn wrong direction
- 2-anchor fallback adds code complexity for degraded mode that could cause unsafe motion
- Better: stop and wait for tag to come back into view
- If anchor dropout is FREQUENT in Phase 5 testing → fix mounting/orientation, not software
- Fallback for Phase 6+: add 4th anchor (ETH approach) or use last-known-good with timeout

**OPEN for Phase 5:** Monitor anchor dropout rate during real-world testing. If >5% of cycles lose an anchor, consider hardware fix (antenna orientation) or 4th anchor.

---

## G5: Per-Module TWR Calibration Needed?

**Question:** Does each BU04 module need individual RNGOFF calibration, or is one calibration sufficient?

**Research Evidence:**
- APS011 §Table 3: calibration distance recommendations (2m for Ch5/9)
- APS011: antenna delay is DOMINANT error source (515ns typical ≈ 77m equivalent — mostly compensated internally, residual 0.5-1.0ns ≈ 15-30cm)
- APS014: step-by-step antenna delay calibration procedure
- DW3000 manufacturing variance: antenna delay can vary 0.5-1.0ns between modules → 15-30cm error
- Community reports (Qorvo forum, esphome-uwb-dw3000): per-module calibration essential for <±15cm

**PLANNER RECOMMENDATION:** **YES — per-module calibration required for ±10cm.**
- Procedure (per anchor):
  1. Place tag at exactly 2.000m from anchor (laser-measured)
  2. Collect 100 TWR ranges via AT+DISTANCE
  3. Compute mean offset: bias_cm = mean(measured_cm) − 200
  4. Apply: AT+RNGOFF=<round(bias_cm)>
  5. Verify: residual error < ±5cm after correction
- Do NOT assume BU04 modules are identical
- Phase 4 work: calibrate all 3 anchors (A, B, C) against the tag

**OPEN for Phase 4:** Verify AT+RNGOFF exists and works in BU04 stock firmware. If not, apply offset on RP2040 side (software correction). Test whether RNGOFF persists after AT+SAVE+power cycle.

---

## G6: Tag Battery Indicator + Auto-Sleep?

**Question:** Does tag need battery monitoring and auto-sleep for power saving?

**Research Evidence:**
- BU04 power in TAG mode (TWR responding): ~50mA continuous (estimated from datasheet)
- 500mAh LiPo → ~10h runtime — sufficient for testing sessions
- No sleep mode in stock AT firmware (BU03/BU04 AT command doc)
- BU04 may expose ADC pin on header — voltage divider → RP2040 ADC alternative

**PLANNER RECOMMENDATION:** **v1: NO battery indicator, NO auto-sleep.**
- 10h runtime sufficient for all Phase 4-5 testing
- Add physical power switch on tag (simplest, most reliable)
- v2 (Phase 6+ optional):
  - Add voltage divider to BU04 ADC pin → AT command query
  - RP2040 relays battery voltage to robot in TAG status packet
  - Auto-sleep requires custom STM32 firmware → out of scope for Phase 3-5

**OPEN for Phase 4:** Check if BU04 exposes battery voltage via any AT command. Check if ADC pin is accessible on BU04 header (pinout in datasheet).

---

## G7: Kalman Filter — RP2040 or Robot?

**Question:** Should Kalman filter run on RP2040 co-processor or on the robot's main controller?

**Research Evidence:**
- Patent CN105828431A: analytic formula is closed-form, no Kalman needed for ±10cm
- ETH Zurich UWBTracker: IEKF on drone's onboard computer, not on UWB modules
- kk9six/dw3000: IMU+UWB fusion runs on ESP32 (not on UWB modules)
- Kalman adds latency (~1-2ms on RP2040 Cortex-M0+) but smooths noise

**PLANNER RECOMMENDATION:** **Start WITHOUT Kalman. Add on ROBOT side if needed.**
- Phase 4-5: run raw analytic formula on RP2040 → output TAG:L,θ
- Robot receives (L,θ) and applies its own motion filter (already has odometry/IMU)
- Kalman on robot benefits from odometry fusion (wheel encoders + UWB = better than UWB alone)
- Only move Kalman to RP2040 if:
  - Robot cannot run Kalman (STM32 with tight loop)
  - Latency of sending raw distances to robot is too high
  - Robot needs pre-filtered position for safety-critical stop

**OPEN for Phase 5:** Test noise level of raw (L,θ) output during robot motion. If jitter >±15cm without Kalman, add simple moving average (N=5) on RP2040 as lightweight alternative before full Kalman.

---

## Summary: What Phase 4 Needs to Know

| Decision | Status | Phase 4 Action |
|----------|--------|---------------|
| G1: a=40cm | RECOMMENDED | Test 30cm & 50cm accuracy during calibration |
| G2: 5Hz target | RECOMMENDED | Measure actual TWR cycle time, test parallel ranging |
| G3: ASCII output | LOCKED | Implement TAG:L,θ protocol on RP2040 |
| G4: STOP on 2 anchors | RECOMMENDED | Monitor dropout rate, fix antenna mounting if needed |
| G5: Per-module cal | REQUIRED | Calibrate all 3 anchors at 2.000m, apply RNGOFF |
| G6: No battery v1 | RECOMMENDED | Add power switch, check ADC pin availability |
| G7: No Kalman v1 | RECOMMENDED | Test noise level; add moving average if jitter >15cm |
