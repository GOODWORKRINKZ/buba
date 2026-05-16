# Phase 1: Firmware Fixes — Research

**Researched:** 2026-05-15
**Domain:** ESP32-C3 / BU04 AT-command firmware (C++/Arduino, PlatformIO)
**Confidence:** HIGH — all findings drawn from direct codebase analysis; response formats are ASSUMED (see Assumptions Log)

---

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

- **D-01:** Add `PDOA_OFFSET_DEG 0` and `RANGE_OFFSET_CM 0` as `#define` constants in `include/config.h` — same pattern as `BASELINE_M`, `POLL_INTERVAL_MS`
- **D-02:** `AT+PDOAOFF=0` and `AT+RNGOFF=0` are **always** sent at setup, even with zero values. No conditional send.
- **D-03:** AT command sequence order (critical): `AT+SETUWBMODE=1` → `AT+PDOAOFF=<val>` → `AT+RNGOFF=<val>` → `AT+USER_CMD=0`. Sending offsets after `AT+USER_CMD=0` is a no-op.
- **D-04:** Implement `getdlistAndRegister()` called from `setup()` after PDOA mode confirmed. Loops `AT+GETDLIST` every 2 s indefinitely, printing `# Ожидание тега... [N] попытка` each attempt.
- **D-05:** On tag found: parse `LongAddr64` from `AT+GETDLIST` response, call `AT+ADDTAG=<addr>,<short>,1,64,0`, verify with `AT+GETKLIST`, then call `AT+SAVE`.
- **D-06:** After `AT+SAVE` the BU04 reboots — call `waitBU04ReadyForever()` to wait for re-init before continuing.
- **D-07:** If tag already registered (check KList via `AT+GETKLIST` or `AT+GETCFG`), skip registration — idempotent on re-flash.
- **D-08:** Wait forever — feed WDT each iteration (via `delay(2000)` which calls `vTaskDelay()`, sufficient).
- **D-09:** Startup diagnostics: `AT+GETVER`, PDOA mode confirmed, registered tag short addr + long addr, `PDOA_OFFSET_DEG` value, `RANGE_OFFSET_CM` value. Print `# Начало стриминга PDOA...` as last line.
- **D-10:** Use `AT+SAVE` after `AT+ADDTAG`; accept NVIC_SystemReset via `waitBU04ReadyForever()`.
- **D-11:** Flash `test_bu04` to each ESP32-C3 first, then production firmware.

### the agent's Discretion

- Short address value for `AT+ADDTAG` (D-05 says `<short>` but no specific value locked)
- Whether to add `#define AT_ADDTAG` macro in config.h or construct inline
- `delay(3000)` after `waitBU04ReadyForever()` inside `getdlistAndRegister()` (to match DW3000 init window)
- Exact diagnostic print format (must follow existing `# ====` block style per D-09)

### Deferred Ideas (OUT OF SCOPE)

- Python auto-detection of serial port (Phase 2)
- Watchdog explicit configuration (delay-based feeding is sufficient — see Q5)
- `AT+FILTER=1` in PDOA mode (Phase 2/3)
- Multiple tag support (out of scope v1)
</user_constraints>

---

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| FW-01 | Firmware sends `AT+PDOAOFF` and `AT+RNGOFF` at every start, before streaming | Plan 1.1; existing macros `AT_PDOAOFF`/`AT_RNGOFF` in config.h; insert before existing `AT_USER_CMD_JSON` send |
| FW-02 | Firmware auto-registers tag after reboot: `AT+GETDLIST` → parse Long Address → `AT+ADDTAG` without manual input | Plan 1.2; `getdlistAndRegister()` new function; macros `AT_GETDLIST`/`AT_GETKLIST` exist |
| FW-03 | Tag config saved in BU04 NVM (`AT+SAVE`) to survive power loss | Plan 1.2; `AT_SAVE` macro exists; `waitBU04ReadyForever()` exists for post-SAVE reboot wait |
| FW-04 | Stable telemetry ≥ 5 Hz continuous ≥ 60 s without WDT reset | Plan 1.4; PDOA loop already in `loop()`, WDT fed by `delay()` in `getdlistAndRegister()` |
| FW-05 | Serial Monitor startup diagnostics: BU04 version, PDOA status, registered tag address | Plan 1.3; `AT+GETVER` already called in PDOA block; expand with tag addr and offset values |
</phase_requirements>

---

## Executive Summary

Four targeted changes to `src/main.cpp` and `include/config.h` deliver the phase goal. The current firmware prints a manual `AT+ADDTAG` hint and sends no offset commands — both are bugs. All infrastructure needed (AT macros, `sendAT()`, `waitBU04ReadyForever()`, the PDOA setup block) already exists in the codebase. The work is insertion and a new function, not a rewrite.

**Primary recommendation:** Insert the three new blocks in order within the existing `#if defined(PDOA_MODE)` setup block at `src/main.cpp` line ~409: (1) offset sends, (2) `getdlistAndRegister()` call, (3) expanded diagnostics. Define two new `#define` constants in `include/config.h`.

**Biggest risk:** `AT+GETDLIST` response format is not directly verifiable from the codebase — the BU04 AT command PDF is in the repo root (`BU03_BU04_AT_command_cn_V1.0.6.pdf`) and must be consulted or tested via `test_bu04` before writing the parser. If parsing fails silently, `getdlistAndRegister()` will loop forever with no error message.

---

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| AT command sequencing (PDOAOFF→RNGOFF→ADDTAG→USER_CMD) | ESP32 `setup()` firmware | BU04 STM32 NVM | ESP32 controls the order; BU04 persists to NVM |
| Tag discovery (AT+GETDLIST polling) | ESP32 `setup()` firmware | — | Blocking loop in setup, before loop() is entered |
| Tag registration (AT+ADDTAG + AT+SAVE) | ESP32 `setup()` firmware | BU04 STM32 NVM | ESP32 sends; BU04 persists tag in NVM, reboots |
| PDOA telemetry streaming | BU04 hardware/firmware | ESP32 `loop()` passthrough | BU04 emits autonomously once tag registered; ESP32 parses |
| Startup diagnostics | ESP32 `setup()` Serial.printf | — | All display on USB CDC Serial |
| Stream stability (60 s burn-in) | Hardware-in-loop test | — | Cannot verify in CI |

---

## Validation Architecture

### Test Framework

| Property | Value |
|----------|-------|
| Framework | PlatformIO build system (no Arduino unit test framework in use) |
| Config file | `platformio.ini` |
| Quick compile check | `pio run -e anchor1_pdoa` |
| Full compile all envs | `pio run -e anchor1_pdoa -e tag -e test_bu04` |
| Static analysis | `pio check -e anchor1_pdoa` |

### Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Automated Command | Hardware Needed? |
|--------|----------|-----------|-------------------|-----------------|
| FW-01 | `AT+PDOAOFF` and `AT+RNGOFF` sends present in code | Compile + code review | `pio run -e anchor1_pdoa` | ❌ |
| FW-01 | Offset sends are before `AT_USER_CMD_JSON` in execution order | Code review / grep | `grep -n "AT_PDOAOFF\|AT_RNGOFF\|AT_USER_CMD" src/main.cpp` | ❌ |
| FW-02 | `getdlistAndRegister()` function compiled without errors | Compile | `pio run -e anchor1_pdoa` | ❌ |
| FW-02 | Function called from PDOA setup block before `AT_USER_CMD_JSON` | Code review | manual | ❌ |
| FW-03 | `AT+SAVE` called inside `getdlistAndRegister()` | Code review | manual | ❌ |
| FW-03 | `waitBU04ReadyForever()` called after `AT+SAVE` | Code review | manual | ❌ |
| FW-04 | 60 s burn-in ≥ 5 Hz no WDT reset | Hardware burn-in | manual Serial Monitor | ✅ required |
| FW-05 | `AT+GETVER` response printed in diagnostics | Compile + hardware | `pio run -e anchor1_pdoa` + serial | ✅ required |
| FW-05 | `PDOA_OFFSET_DEG` and `RANGE_OFFSET_CM` printed | Compile + code review | `pio run -e anchor1_pdoa` | ❌ |

### Wave 0 Gaps

- [ ] No automated unit tests — this project has no test framework; all functional validation is hardware-in-loop
- [ ] `pio check` static analysis config (if desired — optional for this phase)
- [ ] Manual test checklist in PLAN.md (verify AT+GETDLIST raw response via test_bu04 before writing parser)

### Sampling Rate

- **Per task commit:** `pio run -e anchor1_pdoa -e tag -e test_bu04` (compile check)
- **Per wave merge:** Full compile all envs + code review of AT command order
- **Phase gate:** Hardware verification — flash both devices, 60 s burn-in, ≥ 5 Hz CSV in Serial Monitor

---

## AT Command Reference

### AT+GETVER — BU04 Version Query

**Command:** `AT+GETVER`  
**Macro:** `AT_GETVER "AT+GETVER"` (defined in `include/config.h`)  
**Response format (verified):** `"getver software:V1.0.0,hardware:V1.0.0"` [VERIFIED: config.h comment + main.cpp usage]  
**Timeout:** 500 ms is sufficient (used in existing PDOA block at line ~472)  
**Already used:** Yes, in the existing PDOA diagnostics block (`sendAT("AT+GETVER", 500)`)

```cpp
String ver = sendAT(AT_GETVER, 500);
ver.trim();
Serial.println("# " + ver);
// Output: # getver software:V1.0.0,hardware:V1.0.0
```

---

### AT+PDOAOFF / AT+RNGOFF — Calibration Offset Commands

**Commands:** `AT+PDOAOFF=<value>`, `AT+RNGOFF=<value>`  
**Macros:** `AT_PDOAOFF "AT+PDOAOFF"`, `AT_RNGOFF "AT+RNGOFF"` (defined in `include/config.h`)  
**Note:** macros store only the base command — value must be appended with `"=" + String(val)`  
**Value units:**
- `AT+PDOAOFF`: internal PDOA ticks (not degrees directly). For Phase 1 value = 0. [VERIFIED: PITFALLS.md "units = internal PDOA ticks"]
- `AT+RNGOFF`: range offset in cm. For Phase 1 value = 0. [VERIFIED: PITFALLS.md "range offset in cm"]

**Persistence:** Both values appear in `AT+GETCFG` PDOA output as `pdoaOffset:N rngOffset:N` — they are saved to NVM with `AT+SAVE`. [VERIFIED: config.h AT+GETCFG comment]

**Call pattern:**
```cpp
// Use existing macros + String concatenation (same pattern as other AT commands)
sendAT(String(AT_PDOAOFF) + "=" + String(PDOA_OFFSET_DEG), 500);
sendAT(String(AT_RNGOFF)  + "=" + String(RANGE_OFFSET_CM), 500);
```

**Verification:** `sendAT(AT_GETCFG, 1000)` should show `pdoaOffset:0 rngOffset:0` after sending.

---

### AT+GETDLIST — Tag Discovery List

**Command:** `AT+GETDLIST`  
**Macro:** `AT_GETDLIST "AT+GETDLIST"` (defined in `include/config.h`)  
**Purpose:** Returns list of UWB tags that are actively broadcasting within range  
**Timeout:** 2000 ms recommended (matches D-04 "every 2 s") [ASSUMED]

**Response format** [ASSUMED — verify with test_bu04 before implementing parser]:
```
getdlist cnt:N
LongAddr:XXXXXXXXXXXXXXXX ShortAddr:YYYY
OK
```
Where:
- `N` = number of discovered tags (0 if no tags in range)
- `XXXXXXXXXXXXXXXX` = 16 hex chars = 64-bit IEEE EUI-64 address
- `YYYY` = 4 hex chars = 16-bit short address

**Evidence for this format:**
- PITFALLS.md P-2 says "extract addr and shortaddr" from GETDLIST → suggests both fields present [VERIFIED: PITFALLS.md]
- AT+GETCFG PDOA shows `Dlist:N` field → consistent with `cnt:N` pattern [VERIFIED: config.h comment]
- Pattern consistent with `getver`/`getcfg` response key naming style [VERIFIED: config.h]

**Confidence:** LOW-MEDIUM — exact field names unconfirmed. **Must verify with test_bu04 before coding parser.**

**Parse pattern (same as existing PDOA field parser):**
```cpp
// Parse count from response
int cntIdx = dlist.indexOf("cnt:");
int cnt = 0;
if (cntIdx >= 0) cnt = dlist.substring(cntIdx + 4).toInt();

// Parse LongAddr
String longAddr = "";
int laIdx = dlist.indexOf("LongAddr:");
if (laIdx >= 0) {
    laIdx += 9;
    int end = dlist.indexOf(' ', laIdx);
    longAddr = (end >= 0) ? dlist.substring(laIdx, end) : dlist.substring(laIdx);
    longAddr.trim();
}
```

---

### AT+ADDTAG — Register Tag with Anchor

**Command:** `AT+ADDTAG=LongAddr64,ShortAddr,MinRate,MaxRate,Mode`  
**No macro defined** — construct inline or add `#define AT_ADDTAG "AT+ADDTAG"` to config.h  
**Parameters** [VERIFIED: config.h comment + CONTEXT D-05]:

| Parameter | Value | Meaning |
|-----------|-------|---------|
| LongAddr64 | from GETDLIST | IEEE EUI-64 address of tag (16 hex chars) |
| ShortAddr | `0x0001` | 16-bit short address to assign (we choose this) |
| MinRate | `1` | Minimum update rate in Hz |
| MaxRate | `64` | Maximum update rate in Hz (BU04 hardware max = 64) [VERIFIED: CONCERNS.md + README] |
| Mode | `0` | Normal mode |

**Call pattern:**
```cpp
// Recommend adding to config.h: #define TAG_SHORT_ADDR 0x0001
String addCmd = "AT+ADDTAG=" + longAddr + "," + String(TAG_SHORT_ADDR) + ",1,64,0";
String resp = sendAT(addCmd, 1000);
bool ok = resp.indexOf("OK") >= 0;
```

**After ADDTAG:** verify with `AT+GETKLIST`, then send `AT+SAVE`.

---

### AT+GETKLIST — Registered Tag List

**Command:** `AT+GETKLIST`  
**Macro:** `AT_GETKLIST "AT+GETKLIST"` (defined in `include/config.h`)  
**Purpose:** Returns list of tags that have been registered via `AT+ADDTAG` and persisted  
**Timeout:** 1000 ms [ASSUMED]

**Response format** [ASSUMED — verify with test_bu04]:
```
getklist cnt:N
LongAddr:XXXXXXXXXXXXXXXX ShortAddr:YYYY
OK
```
Or possibly checked via AT+GETCFG: `KList:N` field shows registered count.

**Idempotency check (D-07):** Call AT+GETKLIST at start of `getdlistAndRegister()`. If `cnt:` > 0, tag is already registered → return immediately without ADDTAG+SAVE.

```cpp
// Check if already registered (D-07)
String kl = sendAT(AT_GETKLIST, 1000);
// Also check AT+GETCFG KList:N as fallback
String cfg = sendAT(AT_GETCFG, 1000);
int klistCount = 0;
int ki = cfg.indexOf("KList:");
if (ki >= 0) klistCount = cfg.substring(ki + 6).toInt();
if (klistCount > 0) {
    Serial.println("# Тег уже зарегистрирован (KList > 0) — пропуск ADDTAG");
    return;
}
```

---

## Codebase Integration Points

### `sendAT()` — AT Command Send/Receive [VERIFIED: main.cpp lines ~109-121]

```cpp
// Signature:
static String sendAT(const String &cmd, uint32_t timeoutMs = 1000);

// Returns: raw String containing everything BU04 sent, including echo and response
// Breaks on: "OK" found, "ERR" found, or timeout
// BU04 sends "ERR" not "ERROR" — verified in code comment and CONCERNS.md
// Default timeout: 1000 ms

// Success check:
bool ok = resp.indexOf("OK") >= 0;
// Failure check:
bool err = resp.indexOf("ERR") >= 0;
// Timeout (no response): returns empty or partial string — no "OK" and no "ERR"
```

**No return value for error** — callers must inspect the returned String. Pattern is consistent throughout the file.

---

### `waitBU04ReadyForever()` — Already Exists [VERIFIED: main.cpp lines ~99-107]

```cpp
// Signature:
static void waitBU04ReadyForever(const char *msg);

// Behavior: prints msg, loops calling waitBU04Ready() (up to 15 s each),
// prints "# BU04 не отвечает — передёрните питание ТОЛЬКО BU04 (не ESP32)"
// if not ready, retries forever.
// Returns ONLY when BU04 responds "OK" to "AT".

// Use after AT+SAVE in getdlistAndRegister():
waitBU04ReadyForever("# BU04 перезагружается после сохранения тега...");
delay(3000);  // wait for DW3000 init (same as setup() top)
flushBU04();
```

**CRITICAL:** After `waitBU04ReadyForever()` returns, add `delay(3000)` and `flushBU04()` before proceeding — BU04 answers "AT" before DW3000 finishes initializing (P-5). The existing setup() already does this at the top.

---

### WDT Feeding in `getdlistAndRegister()` [VERIFIED: analysis of delay() behavior]

No explicit `esp_task_wdt_reset()` needed. The `delay(2000)` call in the polling loop calls `vTaskDelay()`, which yields the scheduler and feeds the TWDT for the current task. This is the same pattern used in all existing `waitBU04Ready()` loops.

The deferred item "Watchdog configuration for setup phase" is **resolved**: `delay(2000)` is sufficient.

---

### PDOA Setup Block Location [VERIFIED: main.cpp analysis + CONTEXT.md]

`src/main.cpp`, inside `setup()`, inside `#if defined(ROLE_ANCHOR1)` block:

```
Line ~409:  #if defined(PDOA_MODE)
Line ~411:      String uwbm = sendAT(AT_GETUWBMODE, 600);
Line ~413:      bool inPdoa = uwbm.indexOf("1") >= 0;
Line ~415:      if (!inPdoa) {
                    // ... configure PDOA: optional SETCFG + SETUWBMODE=1 + SAVE + waitBU04ReadyForever
Line ~506:      } else {
Line ~507:          Serial.println("# BU04 уже в режиме PDOA");
Line ~508:      }
Line ~510:      sendAT(AT_USER_CMD_JSON, 500);   // ← INSERT: offsets + getdlistAndRegister() BEFORE this line
Line ~514:      { ver print }
Line ~518:      { cfg print }
Line ~521:      Serial.println("# Добавьте тег: ...");  // ← DELETE this line
Line ~522:  }
Line ~523:  #else  // TWR mode
```

**Insertion point for Plans 1.1 + 1.2:** Between line ~508 (closing brace of if/else PDOA check) and line ~510 (`sendAT(AT_USER_CMD_JSON)`).

---

### Existing AT Command Macros in `config.h` [VERIFIED: config.h full read]

| Macro | String Value | Note |
|-------|-------------|------|
| `AT_TEST` | `"AT"` | ping |
| `AT_SAVE` | `"AT+SAVE"` | NVM write + NVIC_SystemReset |
| `AT_GETVER` | `"AT+GETVER"` | response: `"getver software:V1.0.0,hardware:V1.0.0"` |
| `AT_RESTART` | `"AT+RESTART"` | soft reset |
| `AT_RESTORE` | `"AT+RESTORE"` | factory reset |
| `AT_GETCFG` | `"AT+GETCFG"` | PDOA format: `"getcfg Dlist:N KList:N Net:XXXX AncID:N..."` |
| `AT_DISTANCE` | `"AT+DISTANCE"` | TWR only |
| `AT_GETUWBMODE` | `"AT+GETUWBMODE"` | returns `"1"` if in PDOA |
| `AT_SETUWBMODE_TWR` | `"AT+SETUWBMODE=0"` | RAM only, no response |
| `AT_SETUWBMODE_PDOA` | `"AT+SETUWBMODE=1"` | RAM only, no response |
| `AT_DECA` | `"AT+DECA$"` | PDOA auth |
| `AT_GETDLIST` | `"AT+GETDLIST"` | discovery list |
| `AT_GETKLIST` | `"AT+GETKLIST"` | registered list |
| `AT_PDOAOFF` | `"AT+PDOAOFF"` | angle correction (append `=VALUE`) |
| `AT_RNGOFF` | `"AT+RNGOFF"` | range correction (append `=VALUE`) |
| `AT_FILTER` | `"AT+FILTER"` | Kalman filter |
| `AT_USER_CMD_JSON` | `"AT+USER_CMD=0"` | JSON output format |
| `AT_USER_CMD_HEX` | `"AT+USER_CMD=1"` | HEX output format |
| `AT_PDOAGETCFG` | `"AT+PDOAGETCFG"` | read PDOA params |

**Not defined as macros (need inline string or new define):**
- `AT+ADDTAG=...` — comment exists, no `#define AT_ADDTAG`
- `AT+DELTAG=...` — comment exists, no macro

**New macros needed (add to config.h):**
```cpp
// Add after existing AT_RNGOFF line:
#define TAG_SHORT_ADDR     0x0001        // ShortAddr for AT+ADDTAG (первый и единственный тег)
#define PDOA_OFFSET_DEG    0             // коррекция угла PDOA (заменить после Phase 3)
#define RANGE_OFFSET_CM    0             // коррекция расстояния PDOA (заменить после Phase 3)
```

---

## Implementation Details per Plan

### Plan 1.1: Calibration Offset Send (FW-01)

**What changes:**
1. `include/config.h`: add two `#define` constants
2. `src/main.cpp`: add two `sendAT` calls in PDOA setup block

**config.h additions** (after `ANCHOR2_STALE_MS` or after existing AT command section):
```cpp
// ----- Калибровочные смещения PDOA (заменить после Phase 3) ----------------
// Источник: BU03/BU04 AT指令 V1.0.6, команды AT+PDOAOFF и AT+RNGOFF
#define PDOA_OFFSET_DEG    0             // смещение угла PDOA (внутренние тики BU04)
#define RANGE_OFFSET_CM    0             // смещение дистанции (см)
```

**main.cpp insertion** — between closing `}` of PDOA mode check (line ~508) and `sendAT(AT_USER_CMD_JSON)` (line ~510):
```cpp
// ── Калибровочные смещения (FW-01) — отправляем всегда, даже при нулевых значениях ──
// Источник: BU03/BU04 AT指令 V1.0.6, раздел 4 (PDOA команды)
// AT+PDOAOFF должен быть отправлен ДО AT+USER_CMD=0 (иначе — no-op)
sendAT(String(AT_PDOAOFF) + "=" + String(PDOA_OFFSET_DEG), 500);
sendAT(String(AT_RNGOFF)  + "=" + String(RANGE_OFFSET_CM), 500);
Serial.printf("# PDOA offset: %d  Range offset: %d cm\n",
              PDOA_OFFSET_DEG, RANGE_OFFSET_CM);
```

**Verification:** After sending, call `sendAT(AT_GETCFG, 1000)` — response should show `pdoaOffset:0 rngOffset:0`.

---

### Plan 1.2: Auto Tag Registration — `getdlistAndRegister()` (FW-02, FW-03)

**New function** — add above `setup()` in `src/main.cpp`, inside `#if defined(ROLE_ANCHOR1)` section or as a file-scope static:

```cpp
// ── Автоматическая регистрация тега (FW-02/FW-03) ───────────────────────────
// Вызывается из setup() после подтверждения режима PDOA.
// Ждёт тег бесконечно (D-08), регистрирует его через AT+ADDTAG,
// сохраняет в NVM через AT+SAVE (D-10), ждёт перезагрузку BU04 (D-06).
// Идемпотентна: если тег уже зарегистрирован (D-07) — возвращается сразу.
static void getdlistAndRegister() {
    // D-07: проверяем, есть ли уже зарегистрированный тег
    {
        String cfg = sendAT(AT_GETCFG, 1000);
        int ki = cfg.indexOf("KList:");
        if (ki >= 0 && cfg.substring(ki + 6).toInt() > 0) {
            // Тег уже в KList — пропускаем регистрацию
            String kl = sendAT(AT_GETKLIST, 1000);
            kl.trim();
            Serial.println("# Тег уже зарегистрирован: " + kl);
            return;
        }
    }

    // D-04: цикл ожидания тега — 2 с между попытками, бесконечно
    int attempt = 0;
    while (true) {
        attempt++;
        Serial.printf("# Ожидание тега... [%d] попытка\n", attempt);

        String dlist = sendAT(AT_GETDLIST, 2000);

        // Парсим LongAddr из ответа AT+GETDLIST
        // ВНИМАНИЕ: формат ответа подтвердить через test_bu04 перед финальной реализацией!
        // Ожидаемый формат: "getdlist cnt:N LongAddr:XXXXXXXXXXXXXXXX ShortAddr:YYYY\r\nOK"
        String longAddr;
        int laIdx = dlist.indexOf("LongAddr:");
        if (laIdx >= 0) {
            laIdx += 9;
            // Читаем до пробела или конца строки
            int endIdx = dlist.indexOf(' ', laIdx);
            if (endIdx < 0) endIdx = dlist.indexOf('\r', laIdx);
            if (endIdx < 0) endIdx = dlist.indexOf('\n', laIdx);
            longAddr = (endIdx >= 0)
                ? dlist.substring(laIdx, endIdx)
                : dlist.substring(laIdx);
            longAddr.trim();
        }

        if (longAddr.length() >= 16) {
            // D-05: тег найден — добавляем
            Serial.printf("# Тег найден: LongAddr=%s\n", longAddr.c_str());
            String addCmd = "AT+ADDTAG=" + longAddr + "," +
                            String(TAG_SHORT_ADDR) + ",1,64,0";
            String addResp = sendAT(addCmd, 1000);
            if (addResp.indexOf("OK") < 0) {
                Serial.println("# AT+ADDTAG ERR: " + addResp);
                delay(2000);  // кормим WDT (D-08), повторяем
                continue;
            }
            Serial.println("# AT+ADDTAG OK");

            // Проверка через AT+GETKLIST (D-05)
            String kl = sendAT(AT_GETKLIST, 1000);
            if (kl.indexOf("OK") >= 0) {
                Serial.println("# GETKLIST: " + kl);
            }

            // D-10: AT+SAVE → NVM + NVIC_SystemReset BU04
            sendAT(AT_SAVE, 1000);  // BU04 ответит OK, затем перезагрузится

            // D-06: ждём перезагрузку BU04
            waitBU04ReadyForever("# BU04 перезагружается после сохранения тега...");
            // P-5: ждём полную инициализацию DW3000 (такой же delay как в начале setup())
            Serial.println("# Ждём инициализацию BU04 после SAVE (3 с)…");
            delay(3000);
            flushBU04();
            return;
        }

        // Тег не найден — ждём 2 с, кормим WDT (D-08)
        delay(2000);
    }
}
```

**Insertion point in main.cpp PDOA block:** After Plan 1.1 offset sends, before `sendAT(AT_USER_CMD_JSON)`:
```cpp
// ── Автоматическая регистрация тега (FW-02/FW-03) ──
getdlistAndRegister();
```

**Critical note on LongAddr parsing:** The regex/parse above assumes `"LongAddr:"` as the field name. This MUST be verified with `test_bu04` by sending `AT+GETDLIST` manually and reading the hex dump. If the actual field name differs (e.g., `"longaddr:"`, `"addr:"`, etc.), update accordingly.

---

### Plan 1.3: Startup Diagnostics (FW-05)

**Location:** Replace the existing minimal diagnostics block (after `sendAT(AT_USER_CMD_JSON)`, approximately line ~514–521 in current main.cpp).

**Current code to replace:**
```cpp
{
    String ver = sendAT("AT+GETVER", 500);
    ver.trim();
    Serial.println("# " + ver);
    String cfg = sendAT(AT_GETCFG, 1000);
    cfg.trim();
    Serial.println("# " + cfg);
}
Serial.println("# Добавьте тег: AT+ADDTAG=<LongAddr64>,<ShortAddr>,1,64,0");
```

**New diagnostics block** (D-09 — follow existing `# ====` style):
```cpp
// ── Диагностика при старте (FW-05) ──────────────────────────────────────────
{
    // BU04 версия (D-09)
    String ver = sendAT(AT_GETVER, 500);
    ver.trim();
    Serial.println("# " + ver);

    // PDOA статус (D-09)
    String cfg = sendAT(AT_GETCFG, 1000);
    cfg.trim();
    Serial.println("# " + cfg);
    Serial.println("# Режим PDOA подтверждён");

    // Зарегистрированный тег (D-09)
    String kl = sendAT(AT_GETKLIST, 1000);
    kl.trim();
    if (kl.indexOf("OK") >= 0) {
        Serial.println("# Зарегистрированный тег: " + kl);
    } else {
        Serial.println("# Зарегистрированный тег: не найден");
    }

    // Калибровочные смещения (D-09)
    Serial.printf("# PDOA_OFFSET_DEG=%d  RANGE_OFFSET_CM=%d\n",
                  PDOA_OFFSET_DEG, RANGE_OFFSET_CM);

    // Последняя строка перед loop() (D-09)
    Serial.println("# ========================================");
    Serial.println("# Начало стриминга PDOA...");
}
```

---

### Plan 1.4: Stream Stability Test (FW-04)

**What this plan verifies:** ≥ 5 Hz continuous PDOA CSV output for ≥ 60 s without WDT reset.

**Cannot be verified without hardware.** Verification procedure:

1. Flash `test_bu04` to both ESP32-C3 devices. Verify `AT → OK` on both. (D-11)
2. Flash `anchor1_pdoa` to anchor ESP32-C3.
3. Flash `tag` to tag ESP32-C3.
4. Open Serial Monitor (anchor port, 115200).
5. Observe startup banner: `# BU04 готов в режиме PDOA`, offset lines, `getdlistAndRegister()` waiting messages, eventual `# Начало стриминга PDOA...`
6. Count `PDOA,...` CSV lines per second in Serial Monitor for 60 s.
7. **Pass:** ≥ 5 lines/sec (= 1 line per ≤200 ms), no `Guru Meditation Error` or reset banner.
8. **Fail indicators:** WDT reset (restart banner reappears), fewer than 5 CSV lines/sec, `PDOA,...` lines stop.

**What CAN be verified by compile:**
- `g_seq` is `uint8_t` — compile check confirms it. (overflow at 256 ~= 51 s is known and acceptable per STATE.md)
- POLL_INTERVAL_MS=200 → 5 Hz max rate — confirmed in config.h
- No `delay()` added to the PDOA `loop()` branch that would throttle below 5 Hz

**Compile-time check (no hardware):**
```bash
pio run -e anchor1_pdoa -e tag -e test_bu04
```
Confirms: no syntax errors, all new macros resolve, `getdlistAndRegister()` compiles.

---

## Common Pitfalls

### Pitfall 1: AT+GETDLIST Response Format Unknown
**What goes wrong:** Parser writes to wrong field name → `longAddr.length()` never reaches 16 → infinite loop printing "# Ожидание тега..."  
**Why it happens:** Actual BU04 AT command response not directly readable from codebase; PDF needed  
**How to avoid:** Step 0 of Plan 1.2 = use `test_bu04` to manually send `AT+GETDLIST` and capture hex dump BEFORE writing the parser. Confirm field names from raw response.  
**Warning signs:** getdlistAndRegister() never exits, `# Ожидание тега...` loops even with tag powered on

### Pitfall 2: AT+PDOAOFF Sent After AT+USER_CMD=0 (Current Bug, P-1)
**What goes wrong:** Offset is silently ignored — BU04 applies default offsets (usually non-zero)  
**Why it happens:** Current code sends USER_CMD before anything; this sequence was never enforced  
**How to avoid:** Strictly follow D-03 order: PDOAOFF → RNGOFF → USER_CMD. The insertion point is explicitly BEFORE `sendAT(AT_USER_CMD_JSON)` at line ~510.  
**Warning signs:** `AT+GETCFG` shows `pdoaOffset:0` but readings have systematic angle bias

### Pitfall 3: No delay(3000) After Post-SAVE Reboot (P-5)
**What goes wrong:** `waitBU04ReadyForever()` returns when BU04 answers "OK", but DW3000 takes 2–4 s more to initialize. Commands issued immediately fail silently or return stale values.  
**How to avoid:** Always `delay(3000); flushBU04();` after any `waitBU04ReadyForever()` call in `getdlistAndRegister()`.

### Pitfall 4: AT+SAVE Timeout Too Short
**What goes wrong:** `sendAT(AT_SAVE, 500)` — BU04 responds "OK" then immediately reboots within 500 ms. If the timeout expires before "OK" arrives, the String returned has no "OK". This doesn't matter since we call `waitBU04ReadyForever()` next anyway.  
**How to avoid:** No action needed for correctness, but use 1000 ms for clarity: `sendAT(AT_SAVE, 1000)`.

### Pitfall 5: PDOA_OFFSET_DEG Name vs. AT Command Units
**What goes wrong:** Macro named `PDOA_OFFSET_DEG` but `AT+PDOAOFF` takes internal PDOA ticks (not degrees). For Phase 1 (value=0) this doesn't matter. For Phase 3 calibration, the conversion factor must be determined.  
**How to avoid:** Document in config.h comment that the value unit for AT+PDOAOFF is ticks, not degrees. Keep the `_DEG` suffix as a human-readable hint for the target value, but add a comment clarifying this.

### Pitfall 6: AT+ADDTAG After PDOA is Already Streaming (Non-Issue Here)
**What goes wrong:** In some BU04 firmware versions, `AT+ADDTAG` is only accepted before `AT+USER_CMD=0`. Sending it after starts streaming may fail.  
**How to avoid:** `getdlistAndRegister()` is called BEFORE `sendAT(AT_USER_CMD_JSON)` per D-03 order — no action needed, already handled by the correct sequence.

---

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | `AT+GETDLIST` response format: `"getdlist cnt:N LongAddr:XXXXXXXX ShortAddr:YYYY\r\nOK"` | AT Command Reference | Parser fails silently → infinite getdlistAndRegister loop |
| A2 | `AT+GETKLIST` response format contains registered tag LongAddr and ShortAddr | AT Command Reference | Cannot verify registration, cannot print tag addr in diagnostics |
| A3 | `AT+PDOAOFF` and `AT+RNGOFF` values persist to NVM via `AT+SAVE` (shown as `pdoaOffset:N` in GETCFG) | Plan 1.1 | If they don't persist, offsets reset to factory default after each BU04 reboot |
| A4 | `delay(2000)` in getdlistAndRegister() polling loop is sufficient WDT feeding (no explicit `esp_task_wdt_reset()` needed) | Plan 1.2 | If Arduino task WDT subscription is stricter, WDT fires during long tag-wait → ESP32 resets |
| A5 | BU04 assigns ShortAddr automatically and returns it in AT+GETDLIST; if not, `TAG_SHORT_ADDR=0x0001` is a valid choice | Plan 1.2 | If `0x0001` is rejected by BU04, ADDTAG returns ERR |

**Mitigation for A1:** Use `test_bu04` environment to send raw `AT+GETDLIST` and capture hex dump. This should be Wave 0 step in PLAN.md before writing the parser.

---

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| PlatformIO CLI | Build + flash | ✓ (last used in terminal) | unknown | — |
| ESP32-C3 anchor device | Plans 1.2, 1.3, 1.4 | ✓ (hardware exists) | — | — |
| ESP32-C3 tag device | Plans 1.2, 1.4 | ✓ (hardware exists) | — | — |
| BU04 module (anchor) | All plans | ✓ (hardware exists) | — | — |
| BU04 module (tag) | Plans 1.2, 1.4 | ✓ (hardware exists) | — | — |
| `BU03_BU04_AT_command_cn_V1.0.6.pdf` | A1 resolution | ✓ (in repo root) | V1.0.6 | test_bu04 empirical |

**Missing dependencies with no fallback:** None — all required hardware is available.

**Missing dependencies with fallback:**
- AT+GETDLIST format (A1): primary = read PDF; fallback = test_bu04 empirical hex dump

---

## Security Domain

This phase has no network surface, no user input parsing from untrusted sources, and no authentication changes. AT commands are sent over a local UART to a trusted hardware module. OWASP Top 10 and ASVS categories V2, V3, V4, V6 do not apply. V5 (input validation) is partially relevant:

| Concern | Mitigation |
|---------|------------|
| `longAddr.length() >= 16` guard in ADDTAG call | Prevents sending malformed ADDTAG if GETDLIST parser extracts garbage |
| ESPNOW_MAGIC in existing EspNowPkt | Not modified in this phase |

---

## Sources

### Primary (HIGH confidence — verified from codebase)
- `src/main.cpp` — all function signatures, sendAT(), waitBU04ReadyForever(), PDOA block location, PDOA output format
- `include/config.h` — all AT command macros, their string values, response format comments
- `.planning/codebase/CONCERNS.md` — known issues: manual ADDTAG, missing PDOAOFF send, BU04 behavior
- `.planning/codebase/CONVENTIONS.md` — naming rules verified against actual code
- `.planning/phases/01-firmware-fixes/01-CONTEXT.md` — all locked decisions

### Secondary (MEDIUM confidence — cross-referenced)
- `.planning/research/PITFALLS.md` — P-1 (PDOAOFF send order), P-2 (GETDLIST usage pattern confirming it returns LongAddr), P-5 (3s DW3000 delay)
- `.planning/research/FEATURES.md` — TS-2 (PDOAOFF units = internal ticks), TS-3 (auto-registration design pattern)
- `src/test_bu04.cpp` — sendCmd() pattern, AT+SAVE timeout observed as 500ms

### Tertiary (LOW confidence — assumed, needs verification)
- AT+GETDLIST response format (A1) — inferred from field naming patterns and PITFALLS P-2 description
- AT+GETKLIST response format (A2) — inferred by analogy with GETDLIST

---

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH — C++/Arduino on ESP32-C3, PlatformIO, all verified in codebase
- Function signatures: HIGH — read directly from main.cpp
- AT command macros: HIGH — read directly from config.h
- PDOA block location: HIGH — cross-referenced with CONTEXT.md
- AT command response formats: LOW — inferred; must verify with test_bu04 or AT command PDF

**Research date:** 2026-05-15
**Valid until:** This is a closed, non-networked firmware project with locked hardware. Research is stable indefinitely, except A1/A2 which must be verified before implementation.

---

## RESEARCH COMPLETE

**Phase:** 1 - Firmware Fixes
**Confidence:** HIGH (codebase), LOW (AT response format A1/A2 — flag for Wave 0)

### Key Findings

1. **All infrastructure exists:** `sendAT()`, `waitBU04ReadyForever()`, all AT macros (`AT_GETDLIST`, `AT_GETKLIST`, `AT_PDOAOFF`, `AT_RNGOFF`), PDOA setup block — no new patterns needed
2. **Insertion point is precise:** Three blocks insert BEFORE `sendAT(AT_USER_CMD_JSON)` at line ~510, ensuring D-03 AT command order is honored
3. **`getdlistAndRegister()` design is fully specified:** idempotency check via GETCFG KList, polling loop with `delay(2000)` WDT feeding, ADDTAG+SAVE+waitBU04ReadyForever+delay(3000) on success
4. **Critical unknown — A1:** AT+GETDLIST response format must be verified before writing parser. `test_bu04` environment makes this a 5-minute empirical check. Plan Wave 0 must include this step.
5. **New config.h items:** `PDOA_OFFSET_DEG 0`, `RANGE_OFFSET_CM 0`, `TAG_SHORT_ADDR 0x0001` — all follow existing `#define` conventions

### File Created

`.planning/phases/01-firmware-fixes/01-RESEARCH.md`

### Confidence Assessment

| Area | Level | Reason |
|------|-------|--------|
| sendAT() and existing function signatures | HIGH | Read directly from main.cpp |
| AT command macros and their values | HIGH | Read directly from config.h |
| PDOA setup block structure and insertion points | HIGH | Full main.cpp read + CONTEXT.md line reference |
| AT+GETDLIST / AT+GETKLIST response formats | LOW | Inferred; PDF in repo or test_bu04 needed |
| WDT behavior with delay() | MEDIUM | Arduino ESP32 framework behavior; no explicit test |

### Open Questions

1. **AT+GETDLIST exact response format** — Read `BU03_BU04_AT_command_cn_V1.0.6.pdf` (in repo root) OR send `AT+GETDLIST` via test_bu04 and capture hex dump before writing parser
2. **Does AT+GETDLIST include ShortAddr?** — If yes, use it in ADDTAG. If no, use `TAG_SHORT_ADDR=0x0001`.
3. **AT+PDOAOFF unit conversion for Phase 3** — Note in config.h that `PDOA_OFFSET_DEG` is passed directly as-is to AT+PDOAOFF; the conversion factor (ticks/degree) must be measured in Phase 3.

### Ready for Planning

Research complete. Planner can now create PLAN.md files. Wave 0 MUST include: verify AT+GETDLIST format via test_bu04 before implementing the LongAddr parser in getdlistAndRegister().
