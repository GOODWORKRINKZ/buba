---
last_updated: 2026-05-15
focus: quality
---

# Testing

## Testing Approach

This is embedded firmware for ESP32-C3 + BU04 UWB. There is no unit-test
framework (no Google Test, Unity, or PlatformIO's native test runner). Testing
is hardware-in-loop: real hardware must be connected and a Serial Monitor used
to observe output.

Two levels of testing exist:

1. **UART diagnostics** (`test_bu04` environment) — verifies physical UART
   connectivity to the BU04 module, AT command round-trips, and NVM persistence
   across power cycles.
2. **Integration smoke test** (production environments) — flash each role and
   verify CSV telemetry lines appear on Serial Monitor at 115200 baud.

## Test Environment

Defined in `platformio.ini`:

```ini
[env:test_bu04]
build_flags =
    ${env.build_flags}
    -DROLE_TEST=1
build_src_filter =
    -<main.cpp>
    +<test_bu04.cpp>
```

Key behaviour:
- `main.cpp` is **excluded** from this build; only `test_bu04.cpp` is compiled
- `-DROLE_TEST=1` activates the `#ifdef ROLE_TEST` guard in `src/test_bu04.cpp`
- All common flags (USB CDC, baud, debug level) are inherited from `[env]`

## Test Files

**`src/test_bu04.cpp`** — the sole test file.

Structure:
- `hexdump(const String &s, const char *prefix)` — prints byte length, hex dump,
  and escaped ASCII representation of any BU04 response; primary inspection tool
- `sendCmd(const String &cmd, uint32_t ms)` — local AT command helper (flush → send → collect until OK/ERR/timeout)
- `setup()` — runs a fixed 7-step auto-configuration sequence once on boot
- `loop()` — runs pass-through terminal + periodic auto-ping indefinitely

## Test Patterns

### 1. Hexdump inspection

Every AT response is printed as raw bytes + decoded ASCII:
```
<< [6B] hex: 4F 4B 0D 0A  str:"OK\r\n"
```
This makes invisible characters (CR, LF, unexpected bytes) immediately visible
and is the primary tool for debugging BU04 framing issues.

### 2. Auto-configuration sequence (`setup()`)

Seven fixed steps execute on boot without user interaction:

| Step | Command | Purpose |
|------|---------|---------|
| 1 | `AT` | Verify UART link is alive |
| 2 | `AT+GETCFG` | Read current stored configuration |
| 3 | `AT+SETCFG=0,1,1,1` | Set id=0, role=anchor, ch=5, rate=6.8 Mbps |
| 4 | `AT+GETCFG` | Confirm RAM config changed |
| 5 | `AT+SAVE` | Persist to NVM; BU04 reboots |
| 6 | `AT` (after 5 s delay) | Verify BU04 came back online |
| 7 | `AT+GETCFG` | Confirm NVM config survived reboot |

After step 7 the test pauses for a manual power cycle of the BU04, then
instructs the user to send `AT+GETCFG` manually to verify persistence across
full power loss.

### 3. Pass-through terminal (`loop()`)

- **Serial → BU04:** lines typed in Serial Monitor are forwarded to BU04;
  the response is hexdumped. Allows ad-hoc AT command exploration.
- **BU04 → Serial (unsolicited):** any spontaneous output from BU04 is
  hexdumped with prefix `"<< "`.
- **Auto-ping:** `"AT"` is sent every 3 s; response is hexdumped with
  timestamp. Confirms the module stays alive under sustained operation.

## Coverage

### Covered by `test_bu04`

| Area | Covered |
|------|---------|
| UART physical link (GPIO2/GPIO3) | Yes |
| AT echo / connectivity (`AT` → `OK`) | Yes |
| Config read (`AT+GETCFG`) | Yes |
| Config write (`AT+SETCFG`) | Yes |
| NVM persistence (`AT+SAVE` + reboot) | Yes |
| NVM persistence across power cycle | Manual (user-guided) |
| Unsolicited BU04 output | Yes (pass-through) |

### Not covered

| Area | Notes |
|------|-------|
| Role logic (ANCHOR1/ANCHOR2/TAG) | Requires 3 devices and full TWR session |
| ESP-NOW packet exchange | Hardware-only, no mock |
| PDOA JSON parsing | Not exercised in test environment |
| Trilateration math (`x/y/angle`) | No unit tests; formula in `src/main.cpp` header comment |
| Moving average (`updateAvg`) | No unit tests |
| `parseDistance` edge cases | No unit tests |
| `configureBU04` 2-phase TAG strategy | Integration only |
| Stale `g_d2_age` expiry | Integration only |

## Hardware-in-Loop

Physical requirements for running the `test_bu04` environment:

- **ESP32-C3 SuperMini** flashed with `test_bu04` firmware
- **BU04 module** connected:
  - `GPIO2` (TX) → BU04 `PA3` / `USART2_RX` (pin 5)
  - `GPIO3` (RX) ← BU04 `PA2` / `USART2_TX` (pin 4)
  - Alternatively: `PA10`/`PA9` (USART1) on the same GPIO pins
  - 3V3 power supply capable of ≥ 500 mA peak
- **USB cable** to host PC for Serial Monitor at 115200 baud

Without the physical BU04 module the test produces only `<< (нет ответа)` on
every AT step — no software-only fallback exists.

## Running Tests

```bash
# Flash the diagnostic firmware
pio run -e test_bu04 -t upload --upload-port /dev/ttyACM0

# Open Serial Monitor to observe auto-sequence output
pio device monitor -b 115200

# Manual AT commands: type in Serial Monitor, Enter to send
# Example: AT+GETCFG   →  hexdump of current config printed
```

For production role validation:

```bash
pio run -e anchor1 -t upload     # flash ANCHOR1; check "PLATFORM,…" CSV lines
pio run -e anchor2 -t upload     # flash ANCHOR2; check "ANCHOR2,…" CSV lines
pio run -e tag     -t upload     # flash TAG; check "TAG,…" CSV lines
pio run -e anchor1_pdoa -t upload  # flash PDOA anchor; check "PDOA,…" CSV lines
pio device monitor -b 115200     # all roles output at 115200 baud
```
