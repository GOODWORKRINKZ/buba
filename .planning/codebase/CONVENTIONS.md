---
last_updated: 2026-05-15
focus: quality
---

# Code Conventions

## Language Standard

- **C++ with Arduino framework** targeting ESP32-C3 (ESP-IDF / arduino-esp32)
- PlatformIO build system; `platformio.ini` is the single source of truth for build config
- No external libraries — BU04 is controlled entirely via AT commands over UART
- `#pragma once` used in headers (`include/config.h`)
- `__attribute__((packed))` on structs shared over the wire (e.g., `EspNowPkt` in `src/main.cpp`)

## Naming

**Global variables:** `g_` prefix, snake_case
```cpp
static uint8_t  g_seq     = 0;
static float    g_avgBuf[MOVING_AVG_SAMPLES] = {};
static float    g_d2_m    = -1.0f;
static uint32_t g_d2_age  = 0;
```
(see `src/main.cpp`, lines ~55–62)

**Functions:** camelCase verbs
```cpp
flushBU04()        waitBU04Ready()    waitBU04ReadyForever()
sendAT()           parseDistance()    updateAvg()
configureBU04()    initESPNow()       handlePkt()
isMacDefault()
```
All file-scope helpers are declared `static`.

**Constants / macros:** ALL_CAPS `#define` in `include/config.h`
```cpp
#define PIN_BU04_TX     2
#define BU04_BAUD       115200
#define AT_TEST         "AT"
#define POLL_INTERVAL_MS  200
#define MAX_VALID_DIST_M  50.0f
```

**Types / structs:** PascalCase
```cpp
struct __attribute__((packed)) EspNowPkt { ... };   // src/main.cpp
```

**Local variables:** camelCase (`wantId`, `wantRole`, `t0`, `deadline`, `inPdoa`)

**Build-flag identifiers:** SCREAMING_SNAKE_CASE
```
ROLE_ANCHOR1   ROLE_ANCHOR2   ROLE_TAG   ROLE_TEST   PDOA_MODE
```

## Code Style

- **Indentation:** 4 spaces (no tabs)
- **Brace style:** K&R — opening brace on the same line as control statement
- **Trailing `f` suffix** on all float literals: `0.30f`, `-1.0f`, `50.0f`
- **`static` by default** for all file-scope functions and variables; nothing leaks to global linkage
- **Line length:** ~80–100 characters; longer lines appear in complex `printf` calls
- Section dividers use `// ══…══` or `// ──…──` to separate logical blocks within a file
- ASCII art used for hardware wiring and telemetry flow diagrams (see `include/config.h` and `src/main.cpp` header)

## Patterns Used

**Role-based compilation (single-file multi-target):**
All roles compile from `src/main.cpp`; role code is gated with `#ifdef`:
```cpp
#if defined(ROLE_ANCHOR1)
    // anchor-only code
#elif defined(ROLE_ANCHOR2)
    // ...
#elif defined(ROLE_TAG)
    // ...
#endif
```
`ROLE_TEST` environment uses `build_src_filter` to swap `main.cpp` for `test_bu04.cpp`.

**AT command send/receive:**
```cpp
static String sendAT(const String &cmd, uint32_t timeoutMs = 1000) {
    flushBU04();
    bu04.println(cmd);
    String resp;
    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        while (bu04.available()) resp += (char)bu04.read();
        if (resp.indexOf("OK")  >= 0) break;
        if (resp.indexOf("ERR") >= 0) break;
        delay(10);
    }
    return resp;
}
```
(`src/main.cpp`) — same pattern mirrored in `src/test_bu04.cpp` as `sendCmd()`.

**Burst AT commands (queue before awaiting):**
For sequences that trigger BU04 reboots, multiple commands are written to the UART buffer
before waiting for any response, exploiting the STM32F103 UART RX FIFO:
```cpp
bu04.print("AT+SETCFG=...\r\n");
bu04.print("AT+SETUWBMODE=1\r\n");
bu04.print("AT+SAVE\r\n");
// then wait for BU04 to reboot
```

**Polling loop (`loop()`):**
All measurement logic runs in a tight `loop()` with `delay(POLL_INTERVAL_MS)` throttling.
No RTOS tasks or interrupts are used for the main measurement path.

**Moving average:**
Ring-buffer implementation in `updateAvg()` (`src/main.cpp`) controlled by
`MOVING_AVG_SAMPLES` (default 5) defined in `include/config.h`.

**Magic-byte packet validation:**
ESP-NOW packets carry `ESPNOW_MAGIC = 0xBB04`; packets from other devices are discarded.

## Error Handling

- **Invalid distance** returns `-1.0f`; callers check `d < 0` before using the value
- **AT command failures** detected by `resp.indexOf("ERR") >= 0`; BU04 sends `"ERR"`, not `"ERROR"`
- **Timeout without response** returns the partial (possibly empty) `String`; caller decides
- **BU04 not ready:** `waitBU04Ready()` retries up to 30×500 ms; returns `false` on timeout.
  `waitBU04ReadyForever()` loops indefinitely printing human-readable prompts to Serial.
- **Unconfigured MAC** detected by `isMacDefault()` (all `0xFF` bytes); prints warning but continues
- **Missing role define** causes a compile-time `#error` at the top of `src/main.cpp`:
  ```cpp
  #if !defined(ROLE_ANCHOR1) && !defined(ROLE_ANCHOR2) && !defined(ROLE_TAG)
  #  error "Задайте ROLE_ANCHOR1=1, ROLE_ANCHOR2=1 или ROLE_TAG=1 в build_flags"
  #endif
  ```

## Preprocessor Usage

- All pin numbers, baud rates, UWB parameters, and AT-command strings are `#define` in `include/config.h`
- Build roles injected via `build_flags` in `platformio.ini` (e.g., `-DROLE_ANCHOR1=1`)
- `PDOA_MODE` can be added alongside `ROLE_ANCHOR1` for the single-anchor angle-measurement mode
- `${env.build_flags}` inheritance used in `platformio.ini` to avoid repeating common flags
- `ARDUINO_USB_CDC_ON_BOOT=1` and `ARDUINO_USB_MODE=1` required for USB serial on ESP32-C3

## Documentation

- **Language:** Russian (Cyrillic) throughout — all inline comments, `Serial.println()` messages, and section headers
- **File headers:** C-style block comments `/* … */` describing purpose, roles, AT-command sources, and formulas
- **Inline comments:** `//` style, often right-aligned to describe register/pin names or protocol details
- **Source attribution:** referenced as `"Источник: BU03/BU04 AT指令 V1.0.6"` for AT commands,
  `"BU04 规格书 V1.0.0"` for hardware pinout
- **ASCII diagrams:** used for wiring tables (in `include/config.h`) and system topology (in `README.md` and `src/main.cpp`)
- **No JSDoc/Doxygen** — function-level docs are written as prose comments above the function body
