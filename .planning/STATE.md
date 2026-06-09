# Project State

**Project:** BU04 UWB PDOA Test Bench
**Milestone:** UWB Test Bench v1
**Initialized:** 2026-05-15

---

## Current Position

**Phase:** Phase 2 — Context ready, ready for planning
**Next action:** Run `/gsd-plan-phase 2` to create detailed plan, then `/gsd-execute-phase 2` to build

---

## Phase Status

| Phase | Name | Status |
|-------|------|--------|
| 1 | Firmware Fixes (AT+PDOAOFF, auto AT+ADDTAG) | ✅ Completed — PDOA streaming verified, direct USB working |
| 2 | Python Visualizer (polar plot + CSV logger) | 🔵 Context ready — decisions locked, ready for planning |
| 3 | Calibration (empirical PDOA_OFFSET + RANGE_OFFSET) | ⬜ Not started |
| 4 | Accuracy Verification (3×3 grid RMSE test) | ⬜ Not started |

---

## Key Context for Planning Agents

- **Critical bugs in firmware:** AT+PDOAOFF/RNGOFF never sent; AT+ADDTAG manual-only → Phase 1 fixes both
- **PDOA saturation:** ±60° hard limit — angle unreliable outside that range
- **dtr=False:** mandatory in Python visualizer or ESP32-C3 reboots on serial connect
- **AT sequence:** SETUWBMODE → PDOAOFF → RNGOFF → ADDTAG → USER_CMD (order matters)
- **g_seq:** uint8, wraps at 256 (~51 s at 200 ms interval) — Python must handle mod-256

## Decisions Log

| Date | Decision | Context |
|------|----------|---------|
| 2026-05-15 | PDOA mode (not TWR) | Two BU04 with dual antennas each → angle from single anchor, no second anchor needed |
| 2026-05-15 | USB Serial → Python (pyserial + matplotlib) | Minimum complexity for test bench; already installed |
| 2026-05-15 | Robot following deferred to v2 | Must verify UWB accuracy first |
| 2026-06-08 | Tag init: RAM-only SETCFG retry loop | SETCFG→tag_start→while(1) on INIT FAILED; avoid SAVE loss; watchdog-reset-safe |
| 2026-06-08 | Cross-flash test: HW confirmed OK | Swapped anchor/tag firmware — both BU04 work, INIT FAILED is SW timing issue |
| 2026-06-08 | Cloned Ai-Thinker SDK for reference | STM32F103-BU0x_SDK in project root — aitcmd.lib is only DW3000 retry path |

---
*Last updated: 2026-06-08 — Phase 1 Wave 1 (code) complete. Wave 2 (HW burn-in) blocked on BU04 DW3000 intermittent init.*
