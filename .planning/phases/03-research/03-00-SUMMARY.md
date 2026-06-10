# 03-00 Plan Execution Summary

**Plan:** 03-00 — Technical Data Search (RESEARCH FIRST)
**Date:** 2026-06-10
**Status:** ✅ Complete

---

## Search Execution Summary

| Metric | Value |
|--------|-------|
| Total queries executed | 15 (DuckDuckGo) + 2 (GitHub fetch) + 2 (Qorvo forum) |
| Query groups | 3 (A: DW3000 docs, B: UWB robot projects, C: Alternatives+calibration) |
| KEEP-worthy results | 22 |
| SKIP results | ~40+ (blogs no data, DW1000-only, paywalled, Reddit/Hackaday/YouTube) |
| New PDFs downloaded | 5 (DW3000 Datasheet 255pp, API Guide 2.3MB, BU04 spec x2, DW3000 User Manual short) |
| GitHub repos analyzed | 5 (unitree-go2, kk9six/dw3000, Makerfabs, Hubito, ESP32-AppleNearby) |

## 5 Search Items Status

| # | Item from CONTEXT.md | Status | Findings |
|---|----------------------|--------|----------|
| 1 | Download DW3000 User Manual (~200pp) | PARTIAL | Short 68KB version downloaded; full ~200pp likely requires Qorvo NDA. HTML version found at caramelfur.dev. Datasheet (255pp) downloaded successfully. API Guide (2.3MB) downloaded. |
| 2 | Fetch unitree-go2-follow-system | ✅ DONE | Cloned & analyzed. UWB+YOLOv8 hybrid, Python. Uses Go2 built-in UWB (not BU04). FSM pattern, follow controller params useful. Applicability: ⭐⭐⭐ |
| 3 | Read APS011 — DS-TWR formula | ✅ DONE | Full analysis in ROBOT-FOLLOWING.md §14.6. DS-TWR formula, 4 error sources documented, calibration procedure step-by-step, calibration distance table. |
| 4 | BU03 vs BU04 firmware comparison | ✅ DONE | Full 29-command AT table in §14.7. Same firmware binary for both. BU03=tag (omni), BU04=anchor (directional). AT+USER_CMD is extension point for custom button. |
| 5 | PMC8838499 paper analysis | PARTIAL | PDF download failed — PubMed Central returns HTML redirect. Paper metadata captured. Interim verdict: DEFER to v2. Not essential for v1 accuracy target. |

## New Findings That Changed Understanding

1. **No existing open-source project does 3-anchor-on-robot TWR trilateration with BU04.** Our architecture is novel.
2. **unitree-go2 uses single anchor-tag pair (not trilateration)** — different approach, simpler but less accurate.
3. **DW3000 DS-TWR is ±2-3cm with calibration, SS-TWR is ±20-60cm** — we MUST use DS-TWR (SDK supports it).
4. **BU03 and BU04 share identical firmware** — BU03 as tag with omni antenna is better than BU04 as tag with dual directional.
5. **Ai-Thinker has official docs portal** (docs.ai-thinker.com/en/uwb_1/) with BU series documentation.
6. **Qorvo forum has direct PDF downloads** of API Guide and other documents — no registration needed.
7. **PMC8838499 cannot be downloaded via PubMed Central** — HTML redirect only.

## New Open Questions

1. DW3000 Full User Manual behind Qorvo NDA?
2. BU06/BU07 newer modules — better specs?
3. RP2040 PIO UART reliability at 3× 115200?
4. Optimal triangle side length for our robot?
5. Multipath error in indoor environments?

## Documents Modified

| File | Change |
|------|--------|
| `.planning/research/ROBOT-FOLLOWING.md` | +327 lines (§14-15 appended) |
| `docs/datasheets/DW3000_Datasheet.pdf` | NEW — 6.2MB, 255pp |
| `docs/datasheets/DW3000_User_Manual.pdf` | NEW — 68KB short version |
| `docs/app-notes/DW3000_API_Guide.pdf` | NEW — 2.3MB |
| `docs/datasheets/BU04_spec_empere.pdf` | NEW — 1.4MB |
