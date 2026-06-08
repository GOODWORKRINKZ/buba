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

---

## 🔬 2026-06-08 — Hardware Verification & INIT FAILED Debugging

### Cross-flash test: prove HW vs SW issue

**Method:** swap firmware between devices to isolate root cause.

| Test | ACM2 (тег HW) | ACM3/ACM0 (якорь HW) | Verdict |
|------|--------------|----------------------|---------|
| anchor1_pdoa on both | PDOAOFF=OK ✅ | PDOAOFF=OK ✅ | Оба BU04 живые |
| tag on ACM0 (якорь HW) | — | Phase 1 stuck (INIT FAILED) | Проблема в прошивке `tag` |
| tag on ACM2 (тег HW) | Phase 1 stuck | — | Проблема в прошивке `tag` |

**Conclusion:** Железо OK на 100%. INIT FAILED возникает из-за того что `AT+SETCFG` вызывает `node_start()`/`tag_start()` которые при провале `dwt_initialise()` уходят в `while(1){}` намертво. AT-команды (включая SAVE) не обрабатываются.

### SDK Analysis (Ai-Thinker STM32F103-BU0x_SDK)

**Repo cloned:** `STM32F103-BU0x_SDK/` in project root.

**Key findings from source:**
1. `ds_twr_sts_sdc_resp.c` (PDOA anchor): `if (dwt_initialise() == DWT_ERROR) { _dbg_printf("INIT FAILED"); while(1){}; }` — dead loop
2. `ds_twr_sts_sdc_init.c` (PDOA tag): same `while(1){}` on INIT FAILED
3. `main.c` nt_task flow: `nodeAddr == 0xFFFF` → AT command loop (aitcmd.lib with DW3000 retry) → then twr_pdoa_mode check
4. `f_setcfg()` in `cmd_fn.c`: calls `node_start()`/`tag_start()` immediately after setting config in RAM
5. AT mode with `aitcmd.lib` IS the only path with DW3000 retry logic

**Aitcmd.lib property:** when `nodeAddr == 0xFFFF` and `workmode == 0`, BU04 stays in AT command loop. `aitcmd.lib` internally retries `reset_DWIC + dwt_initialise`. This is the ONLY recovery path from INIT FAILED.

### Tag firmware fix iterations

| Iteration | Strategy | Result |
|-----------|----------|--------|
| Original | Phase1: SETCFG(role=1) + SETUWBMODE + SAVE → reboot → Phase2: SETCFG(role=0) + SAVE | ❌ Phase1 stuck (SETCFG→node_start→while(1)) |
| Fix v1 | Detect INIT FAILED in GETCFG, skip Phase1 if UWBMODE already 1 | ❌ Phase2 stuck (SETCFG→tag_start→while(1)) |
| Fix v2 | Phase1: SETUWBMODE+SAVE only (no SETCFG) → Phase2: SETCFG+SAVE | ❌ Phase2 still stuck |
| **Fix v3 (current)** | Phase1: SETUWBMODE+SAVE → **Phase2: SETCFG WITHOUT SAVE (RAM only)** → verify GETCFG → if OK then SAVE | ⏳ Retries, waiting for DW3000 to come up |

**Key insight for Fix v3:** sending SETCFG without SAVE means:
- If DW3000 works → SETCFG applies in RAM → GETCFG shows new config → we SAVE
- If DW3000 fails → tag_start() → while(1){} → watchdog reset (~3s) → BU04 back to AT mode → loop retries

### Current state (2026-06-08 16:00 MSK)

- `src/main.cpp`: tag `configureBU04()` rewritten with RAM-only SETCFG retry loop
- `include/config.h`: PDOA_OFFSET_DEG, RANGE_OFFSET_CM, TAG_SHORT_ADDR constants added
- **Anchor firmware (anchor1_pdoa):** ✅ compiles, waits for tag
- **Tag firmware (tag):** ✅ compiles, loops retrying SETCFG → INIT FAILED → wait 5s → retry
- **Hardware:** confirmed working (cross-flash test), DW3000 init is intermittent
- **Next:** need DW3000 to init successfully once → tag config will be saved → streaming starts

### Notes on DW3000 INIT FAILED

From SDK `deca_device.c`:
- `dwt_initialise()` checks device ID: expects `0xDECA0312` (PDOA) or `0xDECA0302`
- SPI must be ≤ 7 MHz during init
- DW3000 needs ~5ms after reset to reach IDLE_RC state
- PLL lock failure, PGF calibration failure can also cause INIT FAILED
- Aitcmd.lib has internal retry loop — exact mechanism unknown (pre-compiled library)
