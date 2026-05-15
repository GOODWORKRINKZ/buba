# Phase 1: Firmware Fixes — Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in 01-CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-05-15
**Phase:** 01-firmware-fixes
**Areas discussed:** Calibration offsets, Auto tag registration, AT+SAVE, Diagnostics, Timeout behavior

---

## Devices

| Option | Description | Selected |
|--------|-------------|----------|
| anchor1_pdoa + tag | Main production mode | — |
| test_bu04 first | Diagnostic firmware on both modules before main flash | ✓ |
| Both | test_bu04 diagnostics + then anchor + tag | ✓ |

---

## Calibration Offsets (AT+PDOAOFF / AT+RNGOFF)

| Option | Description | Selected |
|--------|-------------|----------|
| 0 now, fill after calibration | Store as #define in config.h, send always | ✓ |
| Send only if != 0 | Conditional send | — |

**Note:** Always send even with value 0 — establishes correct AT sequence (SETUWBMODE → PDOAOFF → RNGOFF → USER_CMD).

---

## Auto Tag Registration (AT+ADDTAG)

| Option | Description | Selected |
|--------|-------------|----------|
| Manual via Serial Monitor | pass-through already exists | — |
| Auto via AT+GETDLIST in setup() | Implement getdlistAndRegister() | ✓ |

---

## Timeout If Tag Not Found

| Option | Description | Selected |
|--------|-------------|----------|
| Wait forever | Print "# Ожидание тега... [N] попытка" each iteration | ✓ |
| Timeout 30s, continue without tag | Proceed with warning | — |

---

## AT+SAVE After ADDTAG

| Option | Description | Selected |
|--------|-------------|----------|
| AT+SAVE — tag persists reboot | NVM write + NVIC_SystemReset on BU04 | ✓ |
| No AT+SAVE — RAM only | Must re-register on every anchor restart | — |

---

## Startup Diagnostics

| Option | Description | Selected |
|--------|-------------|----------|
| One-liner | # PDOA OK | Tag: XXXX | minimal | — |
| Verbose block (current style) | version, mode, tag addr, offset values | ✓ |
