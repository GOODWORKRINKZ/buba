---
phase: 01-firmware-fixes
plan: 01
type: execute
wave: 1
depends_on: []
files_modified:
  - include/config.h
  - src/main.cpp
autonomous: true
requirements:
  - FW-01
  - FW-02
  - FW-03
  - FW-04
  - FW-05

must_haves:
  truths:
    - "Power-cycle anchor → Serial Monitor shows BU04 version, PDOA enabled, registered tag addr — no manual AT input required"
    - "AT+PDOAOFF=0 and AT+RNGOFF=0 are logged on every boot"
    - "Tag is auto-registered on first boot; subsequent boots skip registration (idempotent)"
    - "AT+SAVE is called after AT+ADDTAG; BU04 re-init is awaited before stream starts"
    - "PDOA CSV lines arrive at ≥ 5 Hz for ≥ 60 s without WDT reset"
  artifacts:
    - path: "include/config.h"
      provides: "PDOA_OFFSET_DEG, RANGE_OFFSET_CM, TAG_SHORT_ADDR constants"
      contains: "PDOA_OFFSET_DEG"
    - path: "src/main.cpp"
      provides: "getdlistAndRegister() function + offset sends + diagnostics banner"
      exports: ["getdlistAndRegister"]
  key_links:
    - from: "setup() PDOA branch"
      to: "sendAT(AT_PDOAOFF=...)"
      via: "inline call after PDOA mode confirmed, before AT_USER_CMD_JSON"
      pattern: "AT\\+PDOAOFF="
    - from: "setup() PDOA branch"
      to: "getdlistAndRegister()"
      via: "function call after offset sends"
      pattern: "getdlistAndRegister"
    - from: "getdlistAndRegister()"
      to: "AT+SAVE"
      via: "sendAT(AT_SAVE) after AT+ADDTAG"
      pattern: "AT_SAVE"
    - from: "setup() diagnostics"
      to: "AT+GETVER"
      via: "sendAT before streaming loop"
      pattern: "AT\\+GETVER"
---

<objective>
Implement all five firmware fixes for the BU04 PDOA anchor so that PDOA telemetry streams
continuously on every power cycle — no manual AT commands required.

Purpose: Currently the anchor requires manual AT+ADDTAG input each boot and sends no calibration
offsets. These fixes make the device fully autonomous.

Output:
- `include/config.h` gains PDOA_OFFSET_DEG, RANGE_OFFSET_CM, TAG_SHORT_ADDR constants
- `src/main.cpp` gains: offset sends in correct AT sequence (D-03), getdlistAndRegister()
  function (D-04/D-05/D-06/D-07/D-08), and startup diagnostics banner (D-09)
- Compile-clean build for env:anchor1_pdoa
</objective>

<execution_context>
@~/.copilot/get-shit-done/workflows/execute-plan.md
@~/.copilot/get-shit-done/templates/summary.md
</execution_context>

<context>
@/home/ros2/buba/include/config.h
@/home/ros2/buba/src/main.cpp
@/home/ros2/buba/platformio.ini

<interfaces>
<!-- Key types and patterns extracted from existing codebase. -->

From src/main.cpp — existing helper functions available to all tasks:

```cpp
// Sends AT command, waits for OK/ERR or timeout. Returns raw String response.
static String sendAT(const String &cmd, uint32_t timeoutMs = 1000);

// Waits forever until BU04 responds OK to "AT" ping. Feeds loop via delay().
static void waitBU04ReadyForever(const char *msg);

// Discards all pending bytes in BU04 UART RX buffer.
static void flushBU04();
```

From include/config.h — existing AT command macros relevant to PDOA:

```cpp
#define AT_GETVER      "AT+GETVER"
#define AT_GETCFG      "AT+GETCFG"
#define AT_GETDLIST    "AT+GETDLIST"
#define AT_GETKLIST    "AT+GETKLIST"
#define AT_PDOAOFF     "AT+PDOAOFF"   // base macro — do NOT use alone; append "=<val>"
#define AT_RNGOFF      "AT+RNGOFF"    // base macro — do NOT use alone; append "=<val>"
#define AT_SAVE        "AT+SAVE"
#define AT_USER_CMD_JSON "AT+USER_CMD=0"
```

From src/main.cpp — exact insertion point (line ~469–474):

```cpp
        } else {
            Serial.println("# BU04 уже в режиме PDOA");
        }
        // ← INSERT: offset sends + getdlistAndRegister() call HERE
        // Устанавливаем JSON-вывод
        sendAT(AT_USER_CMD_JSON, 500);
        Serial.println("# Формат вывода: JSON");

        // Печатаем информацию об устройстве  ← REPLACE this block with diagnostics banner
        {
            String ver = sendAT("AT+GETVER", 500);
            ...
        }
        Serial.println("# Добавьте тег: AT+ADDTAG=<LongAddr64>,<ShortAddr>,1,64,0"); // ← DELETE
```

AT+GETDLIST response format (ASSUMED — verified via test_bu04 first, see Note below):
  In PDOA mode AT+GETCFG returns: "getcfg Dlist:N KList:N Net:XXXX AncID:N Rate:N ..."
  AT+GETDLIST returns lines like:
    "getdlist TagNum:N"
    "LongAddr64:XXXXXXXXXXXXXXXX ShortAddr:XXXX"   (one per tag in range)
    "OK"
  AT+GETKLIST returns lines like:
    "getklist TagNum:N"
    "LongAddr64:XXXXXXXXXXXXXXXX ShortAddr:XXXX"
    "OK"
  AT+ADDTAG=<LongAddr64>,<ShortAddr>,<MinRate>,<MaxRate>,<Mode>
    Example: AT+ADDTAG=AABBCCDDEEFF0011,0001,1,64,0
</interfaces>
</context>

---

## Threat Model

| Threat ID | Category | Component | Disposition | Mitigation |
|-----------|----------|-----------|-------------|------------|
| T-01-01 | Denial of Service | `getdlistAndRegister()` infinite loop | mitigate | Feed WDT via `delay(2000)` each iteration (per D-08); calls `vTaskDelay()` internally which resets TWDT |
| T-01-02 | Tampering | Wrong tag registered on re-flash | mitigate | Check KList > 0 before calling AT+ADDTAG (per D-07); idempotent guard prevents duplicate registration |
| T-01-03 | Tampering | AT command sent after AT+USER_CMD=0 is a no-op | mitigate | Enforce order: PDOAOFF → RNGOFF → USER_CMD (per D-03); executor must not reorder |
| T-01-04 | Availability | AT+SAVE triggers BU04 reboot; firmware proceeds without waiting | mitigate | Call `waitBU04ReadyForever()` + `delay(3000)` + `flushBU04()` after AT+SAVE (per D-06/D-10) |
| T-01-05 | Information Disclosure | Tag long address printed to Serial (64-bit UWB addr) | accept | Test-bench environment; no public exposure; matches existing codebase debug-print convention |

---

## Wave Structure

| Wave | Plans | Description | Autonomous |
|------|-------|-------------|------------|
| 1 | Plan 1.1, 1.2, 1.3 | All code changes (config.h + main.cpp) — independent by concern, same wave | yes |
| 2 | Plan 1.4 | Hardware-in-loop burn-in — requires flashed hardware | checkpoint |

Plans 1.1, 1.2, 1.3 share `src/main.cpp`. Execute them **sequentially in the order listed** (1.1 → 1.2 → 1.3) because each edits the same file. Plan 1.4 is hardware-only and runs after all code changes are flashed.

---

<tasks>

<!-- ═══════════════════════════════════════════════════════════
     PLAN 1.1 — Calibration offset constants + send (FW-01, D-01, D-02, D-03)
     ═══════════════════════════════════════════════════════════ -->

<task type="auto">
  <name>Task 1.1 — Add PDOA_OFFSET_DEG, RANGE_OFFSET_CM, TAG_SHORT_ADDR to config.h (D-01)</name>
  <files>include/config.h</files>
  <action>
In `include/config.h`, add three new `#define` constants in the existing PDOA section (after the
`AT_PDOAGETCFG` line at the bottom of the AT command macros), following the exact style of the
existing `BASELINE_M` and `POLL_INTERVAL_MS` definitions — same comment format, same alignment.

Add the following block (after the `AT_PDOAGETCFG` definition):

```
// ----- PDOA калибровка (только для режима PDOA) -------------
// Поправка угла (градусы) — вычтите измеренную систематическую ошибку.
// 0 = калибровка отключена. Диапазон: −90 … +90.
#define PDOA_OFFSET_DEG   0

// Поправка расстояния (сантиметры) — вычтите смещение от эталона.
// 0 = калибровка отключена. Диапазон: −200 … +200.
#define RANGE_OFFSET_CM   0

// Короткий адрес тега (16-бит) — используется при AT+ADDTAG.
// Значение 0x0001 — стандартный адрес первого тега.
#define TAG_SHORT_ADDR    0x0001
```

Do NOT modify any existing lines. Append only.
  </action>
  <verify>
    <automated>cd /home/ros2/buba && grep -c "PDOA_OFFSET_DEG" include/config.h</automated>
  </verify>
  <done>
`grep -c "PDOA_OFFSET_DEG" include/config.h` returns 1.
`grep -c "RANGE_OFFSET_CM" include/config.h` returns 1.
`grep -c "TAG_SHORT_ADDR" include/config.h` returns 1.
  </done>
</task>

<task type="auto">
  <name>Task 1.2 — Send AT+PDOAOFF and AT+RNGOFF in correct sequence (FW-01, D-02, D-03)</name>
  <files>src/main.cpp</files>
  <action>
In `src/main.cpp`, locate the exact comment and code block beginning at:

```cpp
        } else {
            Serial.println("# BU04 уже в режиме PDOA");
        }

        // Устанавливаем JSON-вывод
        sendAT(AT_USER_CMD_JSON, 500);
```

Insert the following block **between** the closing `}` of the PDOA mode check and the
`// Устанавливаем JSON-вывод` comment. The result must read:

```cpp
        } else {
            Serial.println("# BU04 уже в режиме PDOA");
        }

        // Калибровочные поправки PDOA (D-02, D-03):
        // AT+PDOAOFF и AT+RNGOFF ДОЛЖНЫ отправляться ДО AT+USER_CMD=0.
        // После AT+USER_CMD=0 эти команды — no-op!
        {
            String offResp = sendAT(String(AT_PDOAOFF) + "=" + String(PDOA_OFFSET_DEG), 500);
            Serial.printf("# PDOAOFF=%d → %s\n", PDOA_OFFSET_DEG,
                          offResp.indexOf("OK") >= 0 ? "OK" : offResp.c_str());
            String rngResp = sendAT(String(AT_RNGOFF) + "=" + String(RANGE_OFFSET_CM), 500);
            Serial.printf("# RNGOFF=%d → %s\n", RANGE_OFFSET_CM,
                          rngResp.indexOf("OK") >= 0 ? "OK" : rngResp.c_str());
        }

        // Устанавливаем JSON-вывод
        sendAT(AT_USER_CMD_JSON, 500);
```

Critical: the offset sends come BEFORE `sendAT(AT_USER_CMD_JSON, 500)`. Do not swap order.

Note: `String(AT_PDOAOFF) + "=" + String(PDOA_OFFSET_DEG)` constructs `"AT+PDOAOFF=0"` at
runtime. `AT_PDOAOFF` is the base macro `"AT+PDOAOFF"` — it must NOT be used alone as a command.
  </action>
  <verify>
    <automated>cd /home/ros2/buba && pio run -e anchor1_pdoa 2>&1 | tail -5</automated>
  </verify>
  <done>
`pio run -e anchor1_pdoa` exits 0 with `[SUCCESS]`.
`grep -n "AT_PDOAOFF" src/main.cpp` shows the new line before the `AT_USER_CMD_JSON` line.
  </done>
</task>

<!-- ═══════════════════════════════════════════════════════════
     PLAN 1.2 — Auto tag registration: getdlistAndRegister() (FW-02, FW-03, D-04…D-08)
     ═══════════════════════════════════════════════════════════ -->

<task type="auto">
  <name>Task 1.3 — Implement getdlistAndRegister() and call from setup() (FW-02, FW-03, D-04–D-08)</name>
  <files>src/main.cpp</files>
  <action>
**Step A — Declare the function.**

Add the function `getdlistAndRegister()` as a `static void` in `src/main.cpp`, immediately BEFORE
the `setup()` function definition (`void setup() {`). Place it right above `void setup()`.

The function implements the following logic (per decisions D-04 through D-08):

```cpp
// Регистрирует тег в BU04 через AT+GETDLIST / AT+ADDTAG / AT+SAVE.
// Идемпотентна: если KList > 0 — тег уже зарегистрирован, ничего не делает (D-07).
// Ждёт бесконечно, кормит WDT через delay(2000) (D-08).
static void getdlistAndRegister() {
    // Проверяем, не зарегистрирован ли тег уже (idempotent guard, D-07)
    {
        String klist = sendAT(AT_GETKLIST, 1000);
        // "getklist TagNum:N" → если N > 0, тег уже есть
        int idx = klist.indexOf("TagNum:");
        if (idx >= 0) {
            int n = klist.substring(idx + 7).toInt();
            if (n > 0) {
                Serial.printf("# Тег уже зарегистрирован (KList=%d), пропускаем AT+ADDTAG\n", n);
                return;
            }
        }
    }

    // Ждём появления тега в эфире (D-04)
    Serial.println("# Ожидание тега (AT+GETDLIST)...");
    String longAddr;
    for (int attempt = 1; ; attempt++) {
        Serial.printf("# Ожидание тега... [%d] попытка\n", attempt);
        String dlist = sendAT(AT_GETDLIST, 1500);

        // Парсим LongAddr64 из ответа (формат: "LongAddr64:XXXXXXXXXXXXXXXX ...")
        int la = dlist.indexOf("LongAddr64:");
        if (la >= 0) {
            // Извлекаем 16 hex-символов после "LongAddr64:"
            String rest = dlist.substring(la + 11);
            rest.trim();
            // Адрес — до первого пробела или конца строки
            int sp = rest.indexOf(' ');
            longAddr = (sp >= 0) ? rest.substring(0, sp) : rest.substring(0, 16);
            longAddr.trim();
            if (longAddr.length() >= 8) break;  // адрес найден
        }
        delay(2000);  // кормим WDT (D-08); vTaskDelay внутри сбрасывает TWDT
    }
    Serial.printf("# Тег обнаружен: LongAddr64=%s\n", longAddr.c_str());

    // Регистрируем тег (D-05)
    // AT+ADDTAG=<LongAddr64>,<ShortAddr>,<MinRate>,<MaxRate>,<Mode>
    // MinRate=1, MaxRate=64, Mode=0 — стандартные параметры PDOA
    char addtagCmd[64];
    snprintf(addtagCmd, sizeof(addtagCmd), "AT+ADDTAG=%s,%04X,1,64,0",
             longAddr.c_str(), (unsigned)TAG_SHORT_ADDR);
    String addResp = sendAT(String(addtagCmd), 2000);
    if (addResp.indexOf("OK") < 0) {
        Serial.printf("# AT+ADDTAG ОШИБКА: %s\n", addResp.c_str());
    } else {
        Serial.printf("# AT+ADDTAG OK (тег %04X)\n", (unsigned)TAG_SHORT_ADDR);
    }

    // Верификация (D-05)
    String klist2 = sendAT(AT_GETKLIST, 1000);
    Serial.printf("# GETKLIST: %s\n", klist2.c_str());

    // Сохраняем в NVM и ждём перезагрузки BU04 (D-05, D-06, D-10)
    Serial.println("# AT+SAVE → BU04 перезагружается...");
    sendAT(AT_SAVE, 500);  // BU04 уходит на NVIC_SystemReset сразу
    delay(500);
    waitBU04ReadyForever("# Ждём перезагрузку BU04 после AT+SAVE...");
    delay(3000);
    flushBU04();
    Serial.println("# BU04 готов после AT+SAVE");
}
```

**Step B — Call the function from setup().**

Locate the block immediately AFTER the newly-added offset sends block (from Task 1.2) and BEFORE
`sendAT(AT_USER_CMD_JSON, 500)`:

```cpp
        // Калибровочные поправки PDOA (D-02, D-03):
        ...
        }

        // Устанавливаем JSON-вывод
        sendAT(AT_USER_CMD_JSON, 500);
```

Insert one line between the offset block's closing `}` and the `// Устанавливаем JSON-вывод` comment:

```cpp
        // Регистрация тега (D-04..D-08): ждём тег, добавляем, сохраняем в NVM
        getdlistAndRegister();
```

Final sequence in setup() must be:
1. `sendAT(AT_PDOAOFF=...)` / `sendAT(AT_RNGOFF=...)` — offsets
2. `getdlistAndRegister()` — tag registration
3. `sendAT(AT_USER_CMD_JSON, 500)` — enable streaming

**Step C — Verify `TAG_SHORT_ADDR` is in scope.**

`TAG_SHORT_ADDR` is defined in `include/config.h` which is already included via `#include "config.h"`.
No additional include is needed.
  </action>
  <verify>
    <automated>cd /home/ros2/buba && pio run -e anchor1_pdoa 2>&1 | tail -5</automated>
  </verify>
  <done>
`pio run -e anchor1_pdoa` exits 0.
`grep -n "getdlistAndRegister" src/main.cpp` shows both definition (before setup) and call (inside setup).
`grep -n "AT_SAVE" src/main.cpp` shows AT_SAVE used inside `getdlistAndRegister`.
  </done>
</task>

<!-- ═══════════════════════════════════════════════════════════
     PLAN 1.3 — Startup diagnostics banner (FW-05, D-09)
     ═══════════════════════════════════════════════════════════ -->

<task type="auto">
  <name>Task 1.4 — Replace info block with startup diagnostics banner (FW-05, D-09)</name>
  <files>src/main.cpp</files>
  <action>
In `src/main.cpp`, find and replace the existing "Печатаем информацию об устройстве" block plus
the stale "Добавьте тег" print that follow `getdlistAndRegister()` and `sendAT(AT_USER_CMD_JSON)`:

**Old block to remove** (starting at the `// Печатаем информацию об устройстве` comment):

```cpp
        // Печатаем информацию об устройстве
        {
            String ver = sendAT("AT+GETVER", 500);
            ver.trim();
            Serial.println("# " + ver);
            String cfg = sendAT(AT_GETCFG, 1000);
            cfg.trim();
            // AT+GETCFG в PDOA: getcfg Dlist:N KList:N Net:XXXX AncID:N Rate:N ...
            Serial.println("# " + cfg);
        }

        Serial.println("# Добавьте тег: AT+ADDTAG=<LongAddr64>,<ShortAddr>,1,64,0");
```

**Replace with** the full diagnostics banner (D-09):

```cpp
        // Диагностический баннер при старте (D-09, FW-05)
        {
            // 1. Версия прошивки BU04
            String ver = sendAT(AT_GETVER, 600);
            ver.trim();
            Serial.printf("# BU04 версия: %s\n", ver.c_str());

            // 2. PDOA статус + зарегистрированный тег
            String cfg = sendAT(AT_GETCFG, 1000);
            cfg.trim();
            // AT+GETCFG в PDOA: "getcfg Dlist:N KList:N Net:XXXX AncID:N Rate:N ..."
            Serial.printf("# GETCFG: %s\n", cfg.c_str());

            // Извлекаем короткий и длинный адрес зарегистрированного тега из KList
            String klist = sendAT(AT_GETKLIST, 1000);
            klist.trim();
            {
                int la = klist.indexOf("LongAddr64:");
                int sa = klist.indexOf("ShortAddr:");
                String longA = (la >= 0) ? klist.substring(la + 11, la + 27) : "—";
                String shortA = "—";
                if (sa >= 0) {
                    String rest = klist.substring(sa + 10);
                    int sp = rest.indexOf(' ');
                    shortA = (sp >= 0) ? rest.substring(0, sp) : rest.substring(0, 4);
                }
                longA.trim();  shortA.trim();
                Serial.printf("# Тег: Short=%s  Long=%s\n",
                              shortA.c_str(), longA.c_str());
            }

            // 3. Калибровочные константы
            Serial.printf("# PDOAOFF=%d RNGOFF=%d\n",
                          PDOA_OFFSET_DEG, RANGE_OFFSET_CM);
        }
        Serial.println("# Начало стриминга PDOA...");
```

The final line `Serial.println("# Начало стриминга PDOA...")` must be the last print before
the closing `}` of the PDOA setup block (i.e., immediately before `#else // не PDOA`).

After this change, `pio run -e anchor1_pdoa` must still exit 0.
  </action>
  <verify>
    <automated>cd /home/ros2/buba && pio run -e anchor1_pdoa 2>&1 | tail -5</automated>
  </verify>
  <done>
`pio run -e anchor1_pdoa` exits 0.
`grep -n "Начало стриминга PDOA" src/main.cpp` returns a match.
`grep -n "Добавьте тег" src/main.cpp` returns no match (old line removed).
`grep -n "PDOAOFF=" src/main.cpp` returns the printf line showing both constants.
  </done>
</task>

<!-- ═══════════════════════════════════════════════════════════
     PLAN 1.4 — Stream stability + UAT (FW-04, hardware-in-loop)
     ═══════════════════════════════════════════════════════════ -->

<task type="checkpoint:human-verify" gate="blocking">
  <name>Task 1.5 — Flash anchor1_pdoa and run 60 s burn-in (FW-04)</name>
  <what-built>
Flash the compiled `anchor1_pdoa` firmware to the ESP32-C3 anchor board. Power-cycle the anchor,
open Serial Monitor, observe autonomous startup and streaming. Capture 60 s of CSV output and
verify ≥ 5 Hz continuous stream without WDT reset.
  </what-built>
  <how-to-verify>
**Step 1 — Flash test_bu04 first (D-11)**

Flash the diagnostic firmware to confirm AT+GETDLIST response format before flashing production:

```bash
pio run -e test_bu04 -t upload
```

Open Serial Monitor. Manually type `AT+GETDLIST` and observe the raw response. Confirm that:
- Response contains `LongAddr64:` field
- Long address is a 16-character hex string
- Record the actual format in the "Notes" section below

**Step 2 — Flash production firmware**

```bash
cd /home/ros2/buba
pio run -e anchor1_pdoa -t upload
pio device monitor -b 115200
```

**Step 3 — Power-cycle anchor (unplug/replug USB)**

Expected Serial Monitor output (no manual input):
```
# BU04 версия: getver software:V1.0.x,hardware:V1.0.x
# GETCFG: getcfg Dlist:1 KList:0 ...
# PDOAOFF=0 RNGOFF=0 → OK
# PDOAOFF=0 RNGOFF=0 → OK
# Ожидание тега (AT+GETDLIST)...
# Ожидание тега... [1] попытка
# Тег обнаружен: LongAddr64=XXXXXXXXXXXXXXXX
# AT+ADDTAG OK (тег 0001)
# BU04 готов після AT+SAVE
# BU04 версия: ...
# Тег: Short=0001  Long=XXXXXXXXXXXXXXXX
# PDOAOFF=0 RNGOFF=0
# Начало стриминга PDOA...
PDOA,0001,1,1.23,5,0.12,1.21,12345
PDOA,0001,2,1.24,5,0.12,1.22,12545
...
```

**Step 4 — 60 s burn-in (FW-04)**

Capture Serial Monitor output to a file, then count PDOA lines:

```bash
# In a second terminal — capture for 65 seconds
timeout 65 pio device monitor -b 115200 > /tmp/pdoa_log.txt 2>&1

# Count PDOA CSV lines
grep -c "^PDOA" /tmp/pdoa_log.txt
```

Pass criteria: result ≥ 300 lines (≥ 5 Hz × 60 s).

**Step 5 — Re-flash and power-cycle (idempotent check)**

Re-flash with `pio run -e anchor1_pdoa -t upload`, power-cycle. Confirm:
- Boot prints `# Тег уже зарегистрирован (KList=1), пропускаем AT+ADDTAG`
- Streaming starts without waiting for tag

**Step 6 — Power-cycle tag while anchor runs**

With anchor streaming, unplug/replug the tag. The anchor should continue streaming and
re-establish within 10 s (BU04 handles this internally once tag is registered in NVM).
  </how-to-verify>
  <resume-signal>
Type "approved" if all 6 steps pass. Or describe the issue (e.g., "WDT reset at step 4",
"LongAddr64 format differs: ...").
  </resume-signal>
</task>

</tasks>

---

## Phase Gate (UAT)

The phase is complete when ALL of the following pass:

| # | Check | Method | Pass |
|---|-------|--------|------|
| U-1 | Power-cycle anchor → startup prints BU04 version, PDOA enabled, tag address — no manual AT input | Serial Monitor observation | Prints without user typing any AT command |
| U-2 | `PDOAOFF=0 RNGOFF=0` visible in startup log | Serial Monitor observation | Exact strings present |
| U-3 | 60 s burn-in at ≥ 5 Hz | `grep -c "^PDOA" /tmp/pdoa_log.txt` | ≥ 300 |
| U-4 | No WDT reset during 60 s | Serial Monitor — no `Guru Meditation` / `Task watchdog` | No such lines in capture |
| U-5 | Re-flash idempotent: KList=1 on second boot, no AT+ADDTAG retry | Serial Monitor after re-flash | `пропускаем AT+ADDTAG` printed |
| U-6 | Power-cycle tag — anchor re-establishes within 10 s | Tag unplug/replug test | PDOA CSV resumes ≤ 10 s after tag power-on |

---

## Notes

### AT+GETDLIST response format (⚠ ASSUMED — must verify via test_bu04)

The plan assumes the following format based on AT command documentation V1.0.6 and
existing `AT+GETCFG` PDOA response patterns in `main.cpp`:

```
getdlist TagNum:1
LongAddr64:AABBCCDDEEFF0011 ShortAddr:0001
OK
```

**Before flashing production firmware (Task 1.5 Step 1):** flash `test_bu04` and manually send
`AT+GETDLIST` with a tag powered on. Record the actual response format.

**If the actual format differs**, the `indexOf("LongAddr64:")` parsing in `getdlistAndRegister()`
must be adjusted to match. Common variants to watch for:
- Lower-case key: `longAddr64:` → update `indexOf` needle
- No space before `ShortAddr`: `LongAddr64:XXXX\nShortAddr:XXXX` (newline-separated)
- Colon without space: `LongAddr64:XXXX,ShortAddr:XXXX` (comma-separated)
- Address length ≠ 16 chars: adjust the substring length check `>= 8` to `>= actual_len / 2`

### Commit messages

```
feat(01-firmware-fixes): add PDOA_OFFSET_DEG, RANGE_OFFSET_CM, TAG_SHORT_ADDR to config.h
feat(01-firmware-fixes): send AT+PDOAOFF/RNGOFF before AT+USER_CMD (D-02, D-03)
feat(01-firmware-fixes): implement getdlistAndRegister() with NVM save (D-04..D-08)
feat(01-firmware-fixes): replace info block with startup diagnostics banner (D-09)
```

### WDT safety

`delay(2000)` inside `getdlistAndRegister()` loop calls `vTaskDelay()` internally on ESP32
Arduino framework, which resets the task watchdog timer. No explicit `esp_task_wdt_reset()`
call is required.

### Offset send construction

`AT_PDOAOFF` macro is `"AT+PDOAOFF"` (without `=`). Construct the full command as:
```cpp
String(AT_PDOAOFF) + "=" + String(PDOA_OFFSET_DEG)
```
This avoids hardcoding the string twice and keeps the value in sync with the `#define`.

---

<verification>
Full phase verification — run after Task 1.4 flash steps complete:

```bash
# 1. Compile check (no hardware needed)
cd /home/ros2/buba && pio run -e anchor1_pdoa

# 2. Config.h constants present
grep "PDOA_OFFSET_DEG\|RANGE_OFFSET_CM\|TAG_SHORT_ADDR" include/config.h

# 3. Offset sends before USER_CMD (line order check)
grep -n "AT_PDOAOFF\|AT_USER_CMD_JSON\|getdlistAndRegister" src/main.cpp

# 4. AT+SAVE inside getdlistAndRegister
grep -A 30 "getdlistAndRegister()" src/main.cpp | grep "AT_SAVE"

# 5. Diagnostics banner present
grep -n "Начало стриминга PDOA\|PDOAOFF=.*RNGOFF=" src/main.cpp

# 6. 60 s burn-in
grep -c "^PDOA" /tmp/pdoa_log.txt  # expect ≥ 300
```
</verification>

<success_criteria>
- `pio run -e anchor1_pdoa` exits 0 (no compile errors)
- All five requirements FW-01 through FW-05 verified against UAT table above
- 60 s burn-in produces ≥ 300 PDOA CSV lines with no WDT reset
- Second boot after re-flash prints "пропускаем AT+ADDTAG" (idempotent registration)
</success_criteria>

<output>
After completing all tasks and UAT, create:
`.planning/phases/01-firmware-fixes/01-firmware-fixes-01-SUMMARY.md`

Include:
- What was implemented (function signatures, AT command sequence, constants added)
- Actual AT+GETDLIST response format observed during test_bu04 step
- Burn-in result (line count, Hz)
- Any offset values changed from 0 (if calibration performed)
- Known issues or deferred items
</output>
