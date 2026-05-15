# Project State

**Project:** BU04 UWB PDOA Test Bench
**Milestone:** UWB Test Bench v1
**Initialized:** 2026-05-15

---

## Current Position

**Phase:** Not started — ready for Phase 1
**Next action:** Run `/gsd-plan-phase 1` to start planning Phase 1 (Firmware Fixes)

---

## Phase Status

| Phase | Name | Status |
|-------|------|--------|
| 1 | Firmware Fixes (AT+PDOAOFF, auto AT+ADDTAG) | ⬜ Not started |
| 2 | Python Visualizer (polar plot + CSV logger) | ⬜ Not started |
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

---
*Last updated: 2026-05-15 after initialization*
