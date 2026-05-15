# Phase 1: Firmware Fixes — Context

**Gathered:** 2026-05-15
**Status:** Ready for planning

<domain>
## Phase Boundary

Flash both ESP32-C3 devices (anchor in `anchor1_pdoa` mode, tag in `tag` mode), verify BU04 is in PDOA mode, implement automatic tag registration so anchor finds the tag without manual AT commands, and confirm continuous PDOA CSV telemetry in Serial Monitor.

**In scope:**
- `src/main.cpp` PDOA setup block: add `AT+PDOAOFF` / `AT+RNGOFF` before `AT+USER_CMD=0`
- Auto tag registration: `AT+GETDLIST` polling loop in `setup()` → parse Long Address → `AT+ADDTAG` → `AT+SAVE`
- Add `PDOA_OFFSET_DEG` and `RANGE_OFFSET_CM` constants to `include/config.h` (value = 0 until Phase 3 calibration)
- Serial startup diagnostics: BU04 version, PDOA mode status, registered tag address, PDOAOFF/RNGOFF values
- `test_bu04` diagnostic firmware test on both modules before main firmware flash
- Stream stability: 60 s burn-in, verify ≥ 5 Hz

**Out of scope:**
- Python visualizer (Phase 2)
- Actual calibration values (Phase 3)
- Robot motor control (Phase 5)
- TWR mode changes

</domain>

<decisions>
## Implementation Decisions

### Calibration Offset Storage
- **D-01:** Add `PDOA_OFFSET_DEG 0` and `RANGE_OFFSET_CM 0` as `#define` constants in `include/config.h` — same pattern as `BASELINE_M`, `POLL_INTERVAL_MS`
- **D-02:** `AT+PDOAOFF=0` and `AT+RNGOFF=0` are **always** sent at setup, even with zero values. Constants will be replaced after Phase 3 calibration. No conditional send.
- **D-03:** AT command sequence order (critical — from research): `AT+SETUWBMODE=1` → `AT+PDOAOFF=<val>` → `AT+RNGOFF=<val>` → `AT+USER_CMD=0`. Sending offsets after `AT+USER_CMD=0` is a no-op.

### Auto Tag Registration
- **D-04:** Implement `getdlist_and_register()` function called from `setup()` after PDOA mode confirmed. Loops `AT+GETDLIST` every 2 s indefinitely, printing `# Ожидание тега... [N] попытка` each attempt.
- **D-05:** On tag found: parse `LongAddr64` from `AT+GETDLIST` response, call `AT+ADDTAG=<addr>,<short>,1,64,0`, verify with `AT+GETKLIST`, then call `AT+SAVE` (writes NVM + triggers NVIC_SystemReset on BU04).
- **D-06:** After `AT+SAVE` the BU04 reboots — anchor firmware calls `waitBU04ReadyForever()` to wait for re-init before continuing. Tag is now persisted in BU04 NVM and survives further BU04 power cycles.
- **D-07:** If tag is already registered (Dlist count > 0 from `AT+GETDLIST`), skip registration and proceed directly — idempotent on re-flash.

### Timeout Behavior
- **D-08:** Wait forever — anchor blocks in `getdlist_and_register()` until tag appears. Every iteration prints `# Ожидание тега... [N] попытка` to Serial so operator knows system is alive. No watchdog trigger (WDT reset timer should be extended / disabled for setup phase or fed each iteration).

### Startup Diagnostics
- **D-09:** Keep existing verbose block style (matches current `# ======` header pattern). Add to the block: BU04 version (`AT+GETVER`), PDOA mode confirmed, registered tag short addr + long addr, `PDOA_OFFSET_DEG` value, `RANGE_OFFSET_CM` value. Print `# Начало стриминга PDOA...` as the last line before `loop()`.

### AT+SAVE and Reset
- **D-10:** Use `AT+SAVE` after `AT+ADDTAG` — tag persists across BU04 power cycle / reboot. Accept the NVIC_SystemReset as part of the flow; `waitBU04ReadyForever()` handles the wait.

### Naming (following code conventions)
- New function: `getdlistAndRegister()` (camelCase verb)
- New constants: `PDOA_OFFSET_DEG`, `RANGE_OFFSET_CM` (ALL_CAPS in config.h)
- New static variables: `g_` prefix if needed at file scope

### Test Sequence
- **D-11:** Flash `test_bu04` to each ESP32-C3 first, verify AT communication works (AT → OK), then flash production firmware (`anchor1_pdoa` and `tag`).

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Source Files
- `src/main.cpp` — full firmware; PDOA setup block starts at line ~409 (`#if defined(PDOA_MODE)`)
- `include/config.h` — all `#define` constants; add `PDOA_OFFSET_DEG` and `RANGE_OFFSET_CM` here
- `src/test_bu04.cpp` — diagnostic firmware (test_bu04 env)

### Build System
- `platformio.ini` — environments: `anchor1_pdoa` (ROLE_ANCHOR1=1 + PDOA_MODE=1), `tag` (ROLE_TAG=1), `test_bu04` (ROLE_TEST=1)

### Planning Docs
- `.planning/PROJECT.md` — project context and core value
- `.planning/REQUIREMENTS.md` — FW-01..FW-05 requirements for this phase
- `.planning/codebase/CONCERNS.md` — known issues: manual ADDTAG, missing PDOAOFF send, g_seq overflow
- `.planning/codebase/CONVENTIONS.md` — naming rules: `g_` prefix, camelCase functions, ALL_CAPS defines
- `.planning/research/PITFALLS.md` — P-1 (PDOAOFF not sent), P-2 (ADDTAG lost), P-5 (3s DW3000 init delay must stay), P-9 (AT+GETDLIST chicken-and-egg)

### Hardware Reference
- `BU03_BU04_AT_command_cn_V1.0.6.pdf` — AT command reference (in repo root)

</canonical_refs>

<deferred>
## Deferred Ideas

- Python auto-detection of serial port (Phase 2)
- Watchdog configuration for setup phase (to investigate — currently setup can take >10s if tag is slow)
- `AT+FILTER=1` in PDOA mode (needs empirical test — Phase 2 or 3)
- Multiple tag support (explicitly out of scope v1)

</deferred>
