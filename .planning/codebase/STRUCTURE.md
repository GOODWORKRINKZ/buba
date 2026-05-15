---
last_updated: 2026-05-15
focus: arch
---

# Project Structure

## Directory Layout

```
buba/                          # Project root (ESP32-C3 + BU04 UWB firmware)
├── platformio.ini             # Build system — all environments, shared settings
├── README.md                  # Setup guide, wiring diagrams, quick-start
├── include/
│   └── config.h               # All constants, pin defs, AT command strings
└── src/
    ├── main.cpp               # All 4 production roles in one file (#ifdef guards)
    └── test_bu04.cpp          # Diagnostic tool — UART pass-through + AT sequence
```

No generated directories (`.pio/` build cache is gitignored by PlatformIO default).
No subdirectories under `src/` — the entire firmware fits in two source files.

## Key Files

### `platformio.ini`
Defines all build environments and shared settings. Controls role selection via `-D` build flags. Also sets `monitor_filters` and `monitor_speed`.

Environments defined:
- `anchor1` → `-DROLE_ANCHOR1=1`
- `anchor1_pdoa` → `-DROLE_ANCHOR1=1 -DPDOA_MODE=1`
- `anchor2` → `-DROLE_ANCHOR2=1`
- `tag` → `-DROLE_TAG=1`
- `test_bu04` → `-DROLE_TEST=1` with `build_src_filter = -<main.cpp> +<test_bu04.cpp>`

`default_envs = anchor1, anchor2, tag` — running `pio run` without `-e` builds all three production TWR roles.

### `include/config.h`
Single header included by both source files. Contains:
- **Pin definitions:** `PIN_BU04_TX` (GPIO2), `PIN_BU04_RX` (GPIO3), `BU04_BAUD`
- **BU04 protocol constants:** `BU04_CHANNEL`, `BU04_RATE`, `BU04_GROUP`, device IDs
- **AT command string macros:** `AT_TEST`, `AT_SAVE`, `AT_GETCFG`, `AT_GETVER`, etc.
- **Geometry:** `BASELINE_M` (0.30 m — distance between anchors)
- **ESP-NOW:** `ANCHOR1_MAC` placeholder `{0xFF,...}` — must be filled before flashing ANCHOR2. `ESPNOW_MAGIC` (0xBB04).
- **Timing:** `POLL_INTERVAL_MS` (200 ms), `ANCHOR2_STALE_MS` (1000 ms)
- **Filtering:** `MOVING_AVG_SAMPLES` (5), `MAX_VALID_DIST_M` (50.0 m)
- **Wiring diagram** in comments (BU04 pin table referencing datasheet V1.0.0 table 5)

### `src/main.cpp`
~700 lines. All production firmware. Internal organisation:

1. **Header block** — role overview, UWB mode descriptions, CSV format documentation
2. **Includes & compile-time guards** — error if no ROLE defined
3. **UART handle** — `static HardwareSerial bu04(1)`
4. **Global state** — `g_seq`, `g_avgBuf[]`, `g_d2_m` / `g_d2_age` (ANCHOR1 only)
5. **BU04 helpers** — `flushBU04()`, `waitBU04Ready()`, `waitBU04ReadyForever()`, `sendAT()`, `parseDistance()`, `updateAvg()`, `waitBU04Dark()`, `configureBU04()`
6. **ESP-NOW block** — `#if ROLE_ANCHOR1 || ROLE_ANCHOR2`: `EspNowPkt` struct, `handlePkt()`, `onRecv()` (IDF4/5 compat wrapper), `onSend()`, `initESPNow()`
7. **`setup()`** — role-branched init: UART, BU04 config, ESP-NOW init, PDOA mode setup
8. **`loop()`** — role-branched measurement loop: PDOA streaming parser or TWR `sendAT(AT_DISTANCE)` + CSV output

### `src/test_bu04.cpp`
~100 lines. Diagnostic-only, excluded from production builds via `build_src_filter`. Compiled only for `env:test_bu04` (`-DROLE_TEST=1`).

Contains:
- `hexdump()` — prints raw bytes as hex + escaped ASCII for debugging UART responses
- `sendCmd()` — simplified AT sender (no `AT_DISTANCE`, just flush + wait for OK/ERR)
- `setup()` — automated 7-step sequence: `AT` → `AT+GETCFG` → `AT+SETCFG=0,1,1,1` → verify → `AT+SAVE` → wait 5s → verify after reboot
- `loop()` (implied) — UART pass-through: Serial Monitor → BU04 / BU04 → Serial Monitor, with auto-ping `AT` every 3 seconds

## Build Environments

| Environment | Build Flags | Source | Output |
|-------------|-------------|--------|--------|
| `anchor1` | `ROLE_ANCHOR1=1` | `main.cpp` | Main platform anchor firmware (TWR) |
| `anchor1_pdoa` | `ROLE_ANCHOR1=1`, `PDOA_MODE=1` | `main.cpp` | Single-anchor PDOA firmware |
| `anchor2` | `ROLE_ANCHOR2=1` | `main.cpp` | Slave anchor firmware (TWR) |
| `tag` | `ROLE_TAG=1` | `main.cpp` | User tag firmware (TWR) |
| `test_bu04` | `ROLE_TEST=1` | `test_bu04.cpp` only | UART diagnostic tool |

**Shared build settings (all environments):**
- Platform: `espressif32`, Board: `esp32-c3-devkitc-02`, Framework: `arduino`
- `-DSERIAL_BAUD=115200 -DCORE_DEBUG_LEVEL=5 -DARDUINO_USB_CDC_ON_BOOT=1 -DARDUINO_USB_MODE=1`
- Monitor: 115200 baud, filters: `esp32_exception_decoder`, `time`

**Upload commands:**
```bash
pio run -e anchor1      -t upload
pio run -e anchor2      -t upload
pio run -e tag          -t upload
pio run -e anchor1_pdoa -t upload
pio run -e test_bu04    -t upload --upload-port /dev/ttyACM0
pio device monitor -b 115200
```

## Naming Conventions

**Files:**
- `main.cpp` — primary firmware (all production roles)
- `test_bu04.cpp` — prefixed with `test_` for diagnostic/utility tools
- `config.h` — single flat config header, no submodule headers

**Functions:**
- `camelCase` for all functions: `waitBU04Ready()`, `parseDistance()`, `updateAvg()`, `configureBU04()`, `initESPNow()`
- Prefix `wait` for blocking-until-condition functions
- Prefix `handle` for event callbacks: `handlePkt()`
- Prefix `on` for ESP-NOW registered callbacks: `onRecv()`, `onSend()`

**Variables:**
- Module-static globals prefixed `g_`: `g_seq`, `g_avgBuf`, `g_d2_m`, `g_d2_age`
- Local variables: `camelCase`, short names acceptable (`d`, `r`, `now`, `t0`)
- AT command macros: `AT_` prefix + `UPPER_SNAKE_CASE`: `AT_GETCFG`, `AT_SAVE`
- Config macros: `UPPER_SNAKE_CASE`, grouped by subsystem: `BU04_*`, `PIN_*`, `ANCHOR*_MAC`, `ESPNOW_*`, `POLL_*`, `MOVING_*`

**Build flags / role identifiers:**
- `ROLE_ANCHOR1`, `ROLE_ANCHOR2`, `ROLE_TAG`, `ROLE_TEST` — always `=1` when defined

## Code Organization

### Compile-time role gating

All role-specific code in `src/main.cpp` is wrapped in `#if defined(ROLE_*)` / `#elif` / `#endif` blocks. The compiler sees only one role per build. Pattern:

```c
#if defined(ROLE_ANCHOR1)
    // anchor1-specific code
#elif defined(ROLE_ANCHOR2)
    // anchor2-specific code
#elif defined(ROLE_TAG)
    // tag-specific code
#endif
```

The sub-mode `PDOA_MODE` is nested inside `ROLE_ANCHOR1` blocks:

```c
#if defined(ROLE_ANCHOR1) && defined(PDOA_MODE)
    // PDOA-specific anchor code
#else
    // TWR anchor/tag code
#endif
```

### Compile-time guard for ESP-NOW inclusion

```c
#if defined(ROLE_ANCHOR1) || defined(ROLE_ANCHOR2)
#  include <WiFi.h>
#  include <esp_now.h>
#  include <esp_idf_version.h>
#endif
```

ESP-NOW headers not included for TAG builds (saves flash).

### IDF version compatibility shim

`onRecv()` callback signature differs between Arduino-ESP32 2.x (IDF4) and 3.x (IDF5). Handled with:

```c
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void onRecv(const esp_now_recv_info_t *info, ...) { handlePkt(info->src_addr, ...); }
#else
static void onRecv(const uint8_t *mac, ...) { handlePkt(mac, ...); }
#endif
```

### Section structure within `main.cpp`

Sections separated by `// ====...====` comment banners:
1. File-level docblock (roles, modes, CSV format)
2. Includes & compile guards
3. UART handle + global state declarations
4. BU04 AT command helpers
5. ESP-NOW (conditionally compiled)
6. `setup()` 
7. `loop()`

### `test_bu04.cpp` gating

Entire file wrapped in `#ifdef ROLE_TEST` / `#endif` so it compiles cleanly if accidentally included in non-test builds (though `build_src_filter` already prevents this).

## Where to Add New Code

**New AT command string:** Add `#define AT_NEWCMD "AT+..."` to `include/config.h` under the relevant section comment.

**New config constant:** Add `#define NEW_CONST value` to `include/config.h`.

**New role:** Add a new `[env:newrole]` section in `platformio.ini` with `-DROLE_NEWROLE=1`, then add `#elif defined(ROLE_NEWROLE)` branches in `setup()` and `loop()` in `src/main.cpp`.

**New diagnostic tool:** Add a new `.cpp` file under `src/`, create a corresponding `[env:test_xxx]` in `platformio.ini` with `build_src_filter = -<main.cpp> +<test_xxx.cpp>` and `-DROLE_TEST_XXX=1`.

**New telemetry field:** Update both the CSV format comment at the top of `src/main.cpp` and the `Serial.printf()` format string in `loop()`.

---

*Structure analysis: 2026-05-15*
