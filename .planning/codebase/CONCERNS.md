---
last_updated: 2026-05-15
focus: concerns
---

# Technical Concerns

## Known Issues

**Broadcast MAC placeholder (`ANCHOR1_MAC`):**
- `include/config.h` line: `#define ANCHOR1_MAC {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}`
- This is the default broadcast MAC. ANCHOR2 will send ESP-NOW packets to all nearby
  devices until this is replaced with the real ANCHOR1 MAC.
- `src/main.cpp` `isMacDefault()` detects the all-FF case and prints a warning at boot,
  but continues running — measurements silently fail to reach ANCHOR1.

**TAG and ANCHOR1 share the same BU04 device ID:**
- `include/config.h`: `BU04_ID_TAG = 0` and `BU04_ID_ANCHOR1 = 0`
- This is required by the TWR protocol (the tag must have the same ID as anchor1 to
  participate in a ranging session), but it is non-obvious and can cause confusion during
  initial setup or when adding a second tag.

**PDOA: tag registration is manual-only:**
- After flashing `anchor1_pdoa`, the firmware prints:
  `# Добавьте тег: AT+ADDTAG=<LongAddr64>,<ShortAddr>,1,64,0`
- No automated tag discovery or registration. The user must: (1) open Serial Monitor,
  (2) type `AT+GETDLIST`, (3) copy the 64-bit address, (4) type `AT+ADDTAG=...` by hand.
- Tag address is not persisted in any config file; procedure must be repeated after
  BU04 power loss unless the user manually issues `AT+SAVE` with the tag registered.

**TWR y-coordinate sign ambiguity:**
- `src/main.cpp` trilateration: `y = sqrtf(d1*d1 - x*x)` — square root always returns
  positive y. Tag position on the opposite side of the ANCHOR1→ANCHOR2 axis is
  indistinguishable from the near side.
- Documented in `README.md` ("знак y не определён"), but no mitigation is implemented.

**`g_seq` wraps at 256:**
- `src/main.cpp`: `static uint8_t g_seq = 0;` — incremented every `POLL_INTERVAL_MS`.
  At 200 ms intervals this wraps every ~51 seconds with no warning or log message.
  Consumer software relying on monotonically increasing sequence numbers will
  see spurious resets.

---

## Configuration TODOs

**`ANCHOR1_MAC` — must be replaced before ANCHOR2 works:**
- File: `include/config.h`
- Procedure: flash `anchor1`, read MAC from Serial Monitor, paste into `config.h`,
  reflash `anchor2`.
- No build-time check; firmware compiles and uploads with the placeholder silently.

**`BASELINE_M` — must be physically measured:**
- File: `include/config.h`, default `0.30f` (30 cm)
- Used in PLATFORM trilateration (`src/main.cpp`).
  An incorrect value directly biases all x/y/angle outputs; no runtime sanity check.

**PDOA calibration values not configured:**
- `AT+PDOAOFF` (angle offset) and `AT+RNGOFF` (range offset) are defined in
  `include/config.h` as string constants but never sent during setup.
- Default BU04 factory values are used; uncalibrated setups will have systematic
  angle and range errors.

**`AT+ADDTAG` MaxRate hard-limited to 64:**
- `README.md` documents `MaxRate макс. = 64`. The firmware comment and README repeat
  this limit but don't enforce or validate it — a user issuing `AT+ADDTAG` with a higher
  rate will silently get clamped or fail inside BU04 firmware.

---

## Fragile Areas

**2-phase TAG configuration (`configureBU04`, role=0):**
- File: `src/main.cpp`, `configureBU04()` function
- Phase 1 sends `AT+SETCFG=id,1` + `AT+SETUWBMODE=1` + `AT+SAVE` to bootstrap the
  DW3000 via `node_start()` (which contains `reset_DWIC`).
- Phase 2 sends `AT+SETCFG=id,0` + `AT+SAVE` while BU04 is in PDOA-anchor mode so
  `tag_start()` runs with an already-initialized DW3000.
- If phase 2's `waitBU04Dark(20000)` times out (BU04 doesn't reboot within 20 s), the
  firmware falls back to a manual power-cycle prompt — the user must physically disconnect
  BU04 power. No automated recovery.
- State detection relies on string matching `AT+GETCFG` output (`"ID:65535"`, `"Role:1,"`);
  any BU04 firmware change to that string format breaks the state machine.

**PDOA `Tag_Addr:` line parser:**
- File: `src/main.cpp`, PDOA loop block
- Uses manual `indexOf` / `substring` rather than `sscanf` (commented: "sscanf не
  доступен надёжно на Arduino"). Any change to the BU04 PDOA output format
  (spacing, field order) silently produces zero values rather than a parse error.

**ANCHOR config: manual power-cycle required for cold ANCHOR:**
- File: `src/main.cpp`, `configureBU04()` ANCHOR branch
- After sending `AT+SETCFG` + `AT+SAVE`, firmware waits 12 s then expects the user
  to manually power-cycle ONLY the BU04 module. This is an interactive step embedded
  inside `setup()`. If the user cycles the whole board (ESP32 + BU04), the 60 s
  startup wait can fail or produce stale config.

**`waitBU04Dark()` timing race:**
- File: `src/main.cpp`
- Polls `bu04.available()` with a 250 ms + 100 ms inner loop. If the BU04 UART ISR
  delivers a byte exactly between the `available()` check and the next poll iteration,
  `alive = true` and the function misses the reboot window, logging "не перезагрузился!"
  even though the reboot did occur.

**`test_bu04.cpp` duplicates pin definitions:**
- File: `src/test_bu04.cpp` lines 14–15: `#define BU04_TX_PIN 2` and `#define BU04_RX_PIN 3`
- These values duplicate `PIN_BU04_TX` / `PIN_BU04_RX` in `include/config.h` and are
  not sourced from it. If config.h pins change, the test binary will use wrong pins
  and silently fail without a compile error.

---

## Hardware Quirks

**`AT+SETCFG` for role=0 (TAG) causes INIT FAILED on cold boot:**
- Root cause (documented in `src/main.cpp` comment block before `configureBU04()`):
  `f_setcfg` for role=0 calls `tag_start()` without `reset_DWIC`. On a cold BU04,
  the DW3000 has not been initialized, so `dwt_initialise()` fails permanently.
- Workaround: the 2-phase strategy (see Fragile Areas). No single-command equivalent exists.

**`AT+SETUWBMODE` writes to RAM only, no response, no reset:**
- File: `include/config.h` comment: "AT+SETUWBMODE → RAM только (twr_pdoa_mode), нет ответа, нет сброса"
- A `sendAT(AT_SETUWBMODE_TWR)` call returns an empty string. Code must always follow
  `AT+SETUWBMODE` with `AT+SAVE` to persist the change; missing the `AT+SAVE` leaves
  BU04 in the new mode only until the next power cycle.

**`AT+SETWORKMODE=1` (AT-only mode) is incompatible with `AT+SETCFG`:**
- File: `src/main.cpp` comment: "at_cmd_recv (workmode=1) не понимает AT+SETCFG — другой протокол"
- In workmode=1, AT commands route through `at_cmd_recv()` in `aitcmd.lib`, not
  through the normal `cmd_fn.c` handler. `AT+SETCFG` is not implemented in that path.

**`AT+GETCFG` returns different formats in TWR vs PDOA mode:**
- TWR format:  `getcfg ID:X, Role:X, CH:X, Rate:X, Group:X`
- PDOA format: `getcfg Dlist:N KList:N Net:XXXX AncID:N Rate:N Filter:N UserCmd:N pdoaOffset:N rngOffset:N`
- `configureBU04()` checks for `"ID:"` and `"Role:"` which only appear in TWR format;
  parsing will fail silently if BU04 is already in PDOA mode when an ANCHOR role is
  being configured.

**`AT+GETSENSOR` is BU03-only:**
- File: `include/config.h` comment: "AT+GETSENSOR поддерживает только BU03, BU04 — НЕ поддерживает!"
- Any attempt to use sensor data from BU04 will fail. BU04 has no accelerometer.

**BU04 responds to AT before NVM/DW3000 init completes:**
- File: `src/main.cpp` `setup()`: "BU04 отвечает на AT раньше, чем заканчивает читать конфиг из flash"
- Mitigated by an extra `delay(3000)` after `waitBU04Ready()`, but this is a fixed
  magic delay — if BU04 firmware changes initialization timing, the delay may be
  too short (silent config corruption risk) or too long (wasted time).

**3.3 V power rail peak current 500 mA:**
- `include/config.h` pin table: "3V3 (500 мА пик!)"
- ESP32-C3 SuperMini 3V3 LDO may be marginal for this load. UWB ranging bursts
  can trigger brown-out resets not caught by `waitBU04Ready()`.

---

## Timing Dependencies

All delays are magic numbers with no reference to hardware specification sheets.

| Location in `src/main.cpp` | Delay | Purpose |
|---|---|---|
| `setup()` | `delay(500)` | Serial + UART boot settle |
| `setup()` | `delay(3000)` | BU04 NVM + DW3000 full init after AT OK |
| `configureBU04()` ANCHOR branch | `delay(12000)` | node_start() + SAVE processing + NVIC_SystemReset |
| `configureBU04()` ANCHOR branch | `delay(8000)` | node_start() + DW3000 init after manual power cycle |
| `configureBU04()` TAG phase 1 | `delay(2000)` | Settle after phase-1 reboot |
| `configureBU04()` TAG phase 2 | `delay(1000)` | Time for SETCFG + tag_start() entry before dark-wait |
| `configureBU04()` TAG phase 2 | `delay(3000)` | BU04 boot settle after phase-2 reboot |
| `waitBU04Ready()` inner loop | `delay(500)` per ping | AT ping cadence (up to 30 attempts = 15 s total) |
| `waitBU04Dark()` inner loop | `delay(250)` + `delay(100)` | Reboot detection cadence |
| `test_bu04.cpp` setup | `delay(2000)` | BU04 boot wait after UART init |
| `test_bu04.cpp` setup | `delay(5000)` | Post-SAVE reboot wait |

Risk: delays are sensitive to BU04 firmware version. A future BU04 firmware update
that changes `node_start()` boot time will silently cause configuration to read stale
NVM values.

---

## Security

**ESP-NOW magic byte is 16-bit only:**
- File: `include/config.h`: `#define ESPNOW_MAGIC 0xBB04`
- File: `src/main.cpp` `handlePkt()`: packet accepted if `p.magic == ESPNOW_MAGIC`
- A 16-bit magic value in a broadcast WLAN environment provides trivial collision
  probability (~1/65536). Any ESP-NOW device that happens to send a 6-byte payload
  beginning with `0xBB04` will inject a false distance measurement into ANCHOR1.

**No ESP-NOW encryption:**
- File: `src/main.cpp` `initESPNow()`: `peer.encrypt = false`
- Distance data is transmitted in plaintext. An attacker with a nearby ESP32 can
  replay or forge `EspNowPkt` packets to spoof robot/anchor position.

**Default broadcast MAC (`ANCHOR1_MAC`):**
- When `ANCHOR1_MAC` is left at the all-FF default, `esp_now_add_peer()` registers a
  broadcast peer. `esp_now_send()` with a broadcast address transmits the measurement
  distance to all ESP-NOW devices in range, not just ANCHOR1.

**No UWB-layer data integrity check:**
- AT+DISTANCE responses are trusted as-is. A malicious or malfunctioning BU04 UART
  output with a crafted `distance: X` string (e.g., injected via UART) would pass
  `parseDistance()` and be accepted as a valid measurement.

---

## Performance

**Moving average only — no Kalman filter at application level:**
- File: `include/config.h`: `#define MOVING_AVG_SAMPLES 5`
- BU04 firmware has a configurable Kalman filter accessible via `AT+SETDEV`
  (kalmanEn, Q, R parameters), but this is never configured in `configureBU04()`.
  The application-level window of 5 samples at 200 ms = 1-second latency before
  a stable reading.

**5 Hz measurement rate:**
- File: `include/config.h`: `#define POLL_INTERVAL_MS 200`
- Adequate for slow-moving robots; insufficient for fast-moving users or high-speed
  vehicle tracking.

**ANCHOR2 stale data threshold is fixed:**
- File: `include/config.h`: `#define ANCHOR2_STALE_MS 1000`
- If ESP-NOW latency spikes (interference, channel congestion), ANCHOR2 data is silently
  dropped from PLATFORM calculations without any log message or fallback.

**Hard ceiling on valid distance:**
- File: `include/config.h`: `#define MAX_VALID_DIST_M 50.0f`
- Distances above 50 m return -1.0f from `parseDistance()` with no warning printed.
  BU04 UWB range is documented as ~100 m line-of-sight; valid measurements above 50 m
  are silently discarded.

**PDOA polling is time-slot based, not event-driven:**
- File: `src/main.cpp` PDOA loop
- Reads UART for `POLL_INTERVAL_MS - 10` ms per cycle. Only the last `Tag_Addr:` line
  in the window is kept; intermediate measurements are discarded. Burst arrival from
  multiple tags is not handled.

**No error rate monitoring:**
- `AT+DISTANCE` errors (resp with no `distance:` substring) increment nothing. Long-term
  connectivity issues with BU04 are invisible unless the user watches the Serial Monitor.

---

## Missing Features / TODOs

**No `AT+ADDTAG` automation in PDOA mode:**
- The firmware prints a reminder to add the tag manually but provides no automated
  registration flow. Users unfamiliar with UWB addressing will be blocked at this step.

**No PDOA calibration automation (`AT+PDOAOFF`, `AT+RNGOFF`, `AT+FILTER`):**
- These calibration AT commands are defined in `include/config.h` but never called.
  Without calibration, systematic angle and range biases are unmitigated.

**No second-tag support:**
- TWR topology supports one tag at a time (BU04_ID_TAG = 0). Adding a second tag
  requires a custom ID assignment and significant changes to `configureBU04()` and
  the ESP-NOW packet structure.

**No unit tests for trilateration math:**
- The `x = (d1² − d2² + B²) / (2·B)`, `y = sqrtf(d1² − x²)`, `ang = atan2f(y, x)`
  computations in `src/main.cpp` have no automated test coverage.
  Degenerate cases (d2f < 0, B = 0, d1 = d2 = 0) are handled only partially (`d2f >= 0`
  guard for y) with no warning output.

**No persistent configuration validation:**
- After flashing, the BU04 role/ID/channel are verified at each boot inside
  `configureBU04()`. However, `BU04_GROUP` (defined as `1` in `include/config.h`) is
  never written — `AT+SETCFG` in `configureBU04()` sends only 4 parameters
  (`id,role,ch,rate`) omitting the group field. The Group field is left at factory
  default (0), which may prevent anchors from ranging if multiple UWB networks share
  the same channel.

**No logging to persistent storage:**
- Telemetry goes to USB Serial only. Power loss loses all measurement history.
  No SD card, SPIFFS, or network sink is implemented.

---

## Maintainability

**Single `main.cpp` for three roles:**
- `src/main.cpp` contains `ROLE_ANCHOR1`, `ROLE_ANCHOR2`, and `ROLE_TAG` logic gated
  by `#ifdef`. The file is ~750 lines and growing. Adding a fourth role (e.g., a
  gateway node) will increase branching complexity significantly.

**AT command strings duplicated between `config.h` and inline `bu04.print()` calls:**
- `include/config.h` defines `AT_SAVE`, `AT_GETCFG`, `AT_SETUWBMODE_*` etc. as macros.
- `src/main.cpp` `configureBU04()` constructs `AT+SETCFG=...` and `AT+SAVE` inline via
  `bu04.print()` string literals — not using the config.h macros. A change to the
  AT+SETCFG format would need updates in both places.

**Russian-language comments and Chinese-language source documentation:**
- All comments in `src/main.cpp`, `include/config.h`, `platformio.ini`, and `README.md`
  are in Russian. The referenced AT command specification (`BU03_BU04_AT指令_V1.0.6.pdf`)
  is in Chinese. International contributors cannot follow either document set.

**Internal BU04 SDK knowledge embedded in comments:**
- `src/main.cpp` `configureBU04()` comments reference internal BU04 firmware symbols:
  `f_setcfg`, `tag_start()`, `node_start()`, `reset_DWIC`, `ds_twr_sts_sdc_responder`,
  `ds_twr_sts_sdc_initiator`, `nt_task`, `at_cmd_recv`, `aitcmd.lib`.
- This knowledge is reverse-engineered from behavior and is not officially documented.
  A BU04 firmware update may silently invalidate the 2-phase workaround without warning.

**`test_bu04.cpp` not covered by any CI/build check:**
- Only compiled when `-DROLE_TEST=1` is set (env `test_bu04`). There is no automated
  test run; the diagnostic tool exists only as a manual upload target.

---

## Portability

**Hardcoded to ESP32-C3 (`esp32-c3-devkitc-02`):**
- File: `platformio.ini`: `board = esp32-c3-devkitc-02`
- All UART pin assignments (`GPIO 2`, `GPIO 3`) and USB CDC flags
  (`ARDUINO_USB_CDC_ON_BOOT=1`, `ARDUINO_USB_MODE=1`) are ESP32-C3-specific.
  Porting to ESP32, ESP32-S3, or other variants requires changes to both
  `platformio.ini` and `include/config.h`.

**PlatformIO-only build system:**
- No `CMakeLists.txt`, no Arduino IDE `.ino` wrapper. The project cannot be opened
  in Arduino IDE without restructuring.

**ESP-NOW IDF version compatibility shim:**
- File: `src/main.cpp`: `#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)` guards
  two different `onRecv` callback signatures. Only IDF 4.x (Arduino-ESP32 2.x) and
  IDF 5.x (Arduino-ESP32 3.x) are considered; IDF 5.1+ API changes are not validated.

**BU04 UART fallback not auto-detected:**
- `include/config.h` documents a fallback UART1 (PA9=TX, PA10=RX) if USART2 doesn't
  respond. `src/main.cpp` comment in `setup()` even says "Запускаем UART к BU04
  (USART1: PA9=TX→GPIO3, PA10=RX←GPIO2)" — the comment mismatches the code (which
  uses UART1 the ESP32 peripheral, not USART1 of BU04). There is no runtime fallback
  or detection; the user must manually rewire.

**Single channel and rate:**
- `include/config.h`: `BU04_RATE = 1` (6.8 Mbps is the only supported rate).
  CH9 (7987.2 MHz) is defined but `BU04_CHANNEL = 1` (CH5) is hardcoded. No runtime
  channel switching or channel conflict detection.
