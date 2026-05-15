# Pitfalls Research — UWB PDOA System

**Project:** BU04 UWB Positioning & Robot Following  
**Hardware:** Ai-Thinker BU04 (DW3000 + STM32F103 AT-bridge) × 2, ESP32-C3 × 2  
**Mode:** PDOA single-anchor  
**Researched:** 2026-05-15  
**Source basis:** Existing codebase (`main.cpp`, `config.h`), BU04 AT指令 V1.0.6, Qorvo DW3000 datasheet / APS014, community reports on ESP32-C3 USB CDC, matplotlib FuncAnimation behavior

---

## Critical Pitfalls (will definitely bite you)

### P-1: Calibration offsets silently absent at every cold boot

**Risk:** `AT+PDOAOFF` and `AT+RNGOFF` are defined in `config.h` but **never sent** in `setup()`.
The DW3000 PDOA phase measurement has a systematic offset caused by PCB trace length differences
between the two antenna paths and antenna coupling geometry. Typical uncalibrated error on BU04 is
**15–40° angular offset** and **5–15 cm range bias**. The system appears to work (non-zero readings
arrive) but every angle reading is wrong by a fixed amount that varies unit-to-unit.

**Warning signs:**
- Tag at dead-ahead (0°) reads a consistent nonzero angle, e.g. +22° regardless of distance.
- `AT+GETCFG` in PDOA mode shows `pdoaOffset:0 rngOffset:0` after reboot even though you set them interactively earlier.

**Prevention:**
Send both commands in `setup()` immediately after the PDOA mode confirmation block, before
issuing `AT+USER_CMD=0` / `AT_USER_CMD_JSON`:

```cpp
// After PDOA mode confirmed, before JSON output is enabled:
sendAT("AT+PDOAOFF=" + String(PDOA_ANGLE_OFFSET), 500);
sendAT("AT+RNGOFF="  + String(PDOA_RANGE_OFFSET_CM), 500);
```

The initial offset values must be determined empirically (place tag at 0°, 1.5 m; adjust
`PDOA_ANGLE_OFFSET` until reported angle = 0). The offsets **survive reboot only if
`AT+SAVE` is issued after setting them**, but sending them every startup avoids depending on NVM.

**Phase:** Phase 1 (calibration procedure) and every phase thereafter.

---

### P-2: Tag registration lost after every anchor reboot

**Risk:** `AT+ADDTAG=<LongAddr64>,<ShortAddr>,1,64,0` must be issued manually each time the
anchor power-cycles. The current firmware does **not** call `AT+GETDLIST` and does **not**
auto-register the tag. If the anchor reboots (firmware update, power loss, crash) the
PDOA output stream stops permanently until a human types `AT+ADDTAG` in Serial Monitor.
During a calibration session with many reboots, this wastes 5–10 minutes per session and
introduces inconsistency (different Short Address, different filter parameters).

**Warning signs:**
- Serial Monitor shows `# BU04 готов в режиме PDOA` but no `PDOA,...` CSV lines appear.
- `AT+GETKLIST` returns empty list.
- `AT+GETDLIST` returns one or more tags (they are ranging), but no ADDTAG was issued.

**Prevention:**
Implement auto-registration after PDOA mode confirmation:

```
1. Poll AT+GETDLIST every 500 ms for up to 10 s
2. On first response with a LongAddr64, extract addr and shortaddr
3. Issue AT+ADDTAG=<LongAddr64>,<ShortAddr>,1,64,0
4. Confirm with AT+GETKLIST
```

`AT+GETDLIST` only returns tags that are actively broadcasting (tag firmware must be running
and ranging). The anchor must already be in PDOA mode before GETDLIST returns any results.

**Phase:** Phase 1 (auto-registration feature, TS-3 in FEATURES.md).

---

### P-3: pyserial open resets ESP32-C3 via USB CDC DTR

**Risk:** On Linux (and Windows), opening a serial port with `pyserial` asserts the DTR line
by default. For USB CDC devices that map DTR to the reset/boot pin, this causes the ESP32-C3
to reset the moment the Python script opens the port. Consequences:
- Tag-side: ESP32-C3 (TAG role) reboots, BU04 reconfigures, measurements drop for 3–15 s.
- Anchor-side: if Python opens anchor's port, anchor reboots, tag registration lost (P-2 fires).

The ESP32-C3 SuperMini USB CDC implementation **does** trigger a reset on DTR assertion.
This is distinct from the auto-reset-for-flashing mechanism; it happens on every `serial.Serial()` open.

**Warning signs:**
- Anchor prints `# BU04 готов в режиме PDOA` again immediately after Python connects.
- First 10–15 seconds of data after Python start are missing or contain restart banners.
- `PDOA` CSV lines do not appear until ~15 s after Python connects.

**Prevention:**
Open the port with DTR disabled:

```python
ser = serial.Serial()
ser.port    = "/dev/ttyACM0"
ser.baudrate = 115200
ser.dtr     = False   # prevents ESP32-C3 reset on open
ser.open()
```

Alternatively use `exclusive=True` to hold the port open across script restarts (reduces
the number of open/close cycles during development).

**Phase:** Phase 2 (Python visualizer, TS-4 in FEATURES.md) and all PC-side phases.

---

### P-4: PDOA angle saturates hard at ±60° with no warning

**Risk:** The DW3000 PDOA algorithm is only accurate within **±60° of antenna boresight**.
Beyond that angle the phase difference wraps or saturates, and the BU04 reports the clamped
extremes (±60°) instead of the true angle. The robot sees a constant ±60° and turns at full
rate in one direction — a "death spiral" from which it never recovers while the tag is
to the side.

The saturation is **silent**: the JSON output still arrives at the same rate, the `Angle` field
just reads −60 or +60 permanently with no error flag.

**Warning signs:**
- Tag sweeps past anchor's side and robot sharply turns but never corrects.
- Logged angle column shows runs of exactly −60 or +60 for multiple seconds.

**Prevention:**
1. Mount anchor so its boresight faces the expected tag operating zone. For robot following,
   the anchor on the robot should face forward; the valid following cone is ±50° of forward.
2. Add a saturation-detection filter in Python/firmware:
   - If angle == ±60 for N consecutive readings (N ≥ 3), flag "tag out of PDOA range".
   - Stop robot or switch to a search routine rather than executing stale saturated commands.
3. Never use PDOA for omnidirectional following; the physical constraint is hardware-level.

**Phase:** Phase 3 (robot following), calibration validation.

---

### P-5: BU04 answers "OK" to AT before DW3000 is ready

**Risk:** `waitBU04Ready()` waits for the first `OK` response. The STM32F103 AT parser on BU04
boots in ~800 ms and responds to `AT` immediately, but the DW3000 UWB chip requires
an additional **2–4 s** for full initialization (XTAL settling, STS key load, calibration ROM).
Commands sent during this window (`AT+SETUWBMODE`, `AT+GETCFG`, `AT+ADDTAG`) either fail
silently or return stale NVM values that do not reflect the live chip state.

The code already has `delay(3000)` after `waitBU04Ready()` in `setup()` — this is correct.
The risk is that future code changes that "optimize" away this delay will break initialization.

**Warning signs:**
- `AT+GETCFG` immediately after `OK` returns `ID:65535` (uninitialized NVM sentinel).
- `AT+SETUWBMODE` appears to succeed but PDOA output never starts.
- Intermittent `INIT FAILED` in BU04 serial debug output (only visible if tapping STM32 USART1).

**Prevention:**
- Keep the 3 s post-ready delay; document it explicitly so it is not removed.
- After any `AT+SAVE` (which causes `NVIC_SystemReset`), wait at least 5 s before issuing
  the next command, not just until the first `OK`.
- The existing `configureBU04()` already handles this correctly for the TWR path.

**Phase:** Phase 0 (firmware bringup), all PDOA setup sequences.

---

## Moderate Pitfalls (likely to bite you)

### P-6: `ax.clear()` every animation frame — CPU spike and plot lag

**Risk:** The STACK.md sample code calls `ax.clear()` inside `update()` on every
`FuncAnimation` interval. On a polar axes this is expensive: matplotlib re-creates all
tick labels, the polar grid, and the coordinate transformer on each call. At 10 Hz
animation interval with 50+ scatter points, CPU reaches 80–100% on a mid-range laptop
and the animation begins to skip frames, introducing **perceived lag of 200–500 ms** in
the live polar display.

**Warning signs:**
- `top` shows Python using >70% CPU during visualization.
- Plot update appears visually slower than the incoming PDOA rate.
- `FuncAnimation` `interval` effectively stretches beyond the set 100 ms.

**Prevention:**
Replace the "clear and redraw" pattern with artist updates:

```python
scatter = ax.scatter([], [], s=8, alpha=0.4)    # created once
current = ax.scatter([], [], s=60, c="red")     # created once

def update(_):
    if not buf: return scatter, current
    angs = [a * pi / 180 for _, a in buf]
    rngs = [r for r, _ in buf]
    scatter.set_offsets(list(zip(angs, rngs)))
    lat_a = latest["angle"] * pi / 180
    current.set_offsets([[lat_a, latest["range"]]])
    return scatter, current
```

With `blit=False` (required for polar — blitting is broken on polar axes in matplotlib ≤ 3.10),
this still avoids the full axis reconstruction overhead.

**Phase:** Phase 2 (Python visualizer).

---

### P-7: `g_seq` uint8 wraps at ~51 s, confusing Python gap detection

**Risk:** `g_seq` is declared `uint8_t` in `main.cpp`. It increments every `POLL_INTERVAL_MS`
(200 ms) and wraps at 255, not 256 — giving a cycle time of **255 × 200 ms = 51 s**. Any Python
code that uses sequence number discontinuity to detect packet loss will fire a false positive
on every wrap (255 → 0). More critically, if the Python logger displays "packet gap" warnings,
a 51-second false alarm every 51 seconds will mask real gaps.

**Warning signs:**
- Python logs show a gap event every ~51 s even with no actual packet loss.
- Sequence numbers in CSV jump from 255 to 0 without any real data loss.

**Prevention:**
Either change `g_seq` to `uint16_t` (wraps at ~218 min), or make Python gap detection
handle uint8 wrap-around:

```python
def seq_gap(prev, cur, bits=8):
    diff = (cur - prev) & ((1 << bits) - 1)
    return diff > 1   # True only if more than one packet missing
```

**Phase:** Phase 1 (telemetry stream), Phase 2 (Python logger, TS-5).

---

### P-8: Only the last tag packet in a 200 ms window is processed

**Risk:** In the PDOA loop (`main.cpp`, `#if defined(ROLE_ANCHOR1) && defined(PDOA_MODE)`),
the code buffers all incoming BU04 lines and saves only the last line starting with
`Tag_Addr:` into `lastLine`. If the BU04 sends two packets in a 200 ms window (possible
if the tag's internal ranging completes faster than the poll interval), only the later one
is emitted over USB Serial.

At 200 ms poll interval the BU04 typically delivers one packet per interval, so this is
rarely a problem in practice. But if `POLL_INTERVAL_MS` is reduced to 100 ms or the BU04
is configured for faster ranging, packets are silently dropped.

**Prevention:**
If packet rate is increased, change the PDOA parsing to emit a CSV line for every complete
`Tag_Addr:` line received, not just the last one per cycle. Remove the `lastLine` accumulation
and emit inside the inner loop.

**Phase:** Phase 1 (telemetry stream stability testing).

---

### P-9: `AT+GETDLIST` requires tag to already be ranging — chicken-and-egg during auto-registration

**Risk:** `AT+GETDLIST` (used for auto-registration, P-2 fix) only returns tag long addresses
if the tag is actively sending UWB ranging requests. If the tag firmware has not yet started
(still configuring, or BU04 is in TWR mode), `AT+GETDLIST` returns an empty list. Auto-registration
code that polls GETDLIST without verifying the tag is ready will time out silently and leave
the anchor with no registered tag.

**Warning signs:**
- `AT+GETDLIST` consistently returns empty list even though the tag hardware is powered.
- PDOA anchor is in mode=1 (confirmed by `AT+GETUWBMODE`) but GETDLIST still empty.
- Issue disappears if you wait 15+ s after anchor boot.

**Prevention:**
- Add a status print for each GETDLIST poll: `# GETDLIST empty, waiting...`.
- Extend the retry window to 30 s (not 10 s). Tags start ranging ~10 s after boot.
- The tag must be in TWR mode (role=0 with `configureBU04`) before the anchor enters PDOA
  mode and calls GETDLIST. Ensure startup sequencing is documented.

**Phase:** Phase 1 (auto-registration, TS-3).

---

### P-10: `AT+GETCFG` format is completely different in TWR vs PDOA mode

**Risk:** Two distinct response formats:

```
TWR:  getcfg ID:0, Role:1, CH:1, Rate:1, Group:1
PDOA: getcfg Dlist:1 KList:1 Net:0000 AncID:0 Rate:1 Filter:0 UserCmd:0 pdoaOffset:0 rngOffset:0
```

The existing `configureBU04()` checks `cur.indexOf("ID:" + String(id) + ",")` — this string
never appears in the PDOA-format response, so `configureBU04()` called while BU04 is already
in PDOA mode will loop forever (or until the 2-phase TAG path detects role=1 and proceeds).

The PDOA setup path in `setup()` already correctly handles this by checking `AT+GETUWBMODE`
first and skipping `configureBU04()` when already in PDOA mode. The pitfall is in maintenance:
adding any new config check that uses the TWR GETCFG format will silently fail when the device
is in PDOA mode.

**Prevention:**
- Always call `AT+GETUWBMODE` before parsing `AT+GETCFG`.
- Add a comment at every `AT+GETCFG` parse site noting which format is expected.
- If PDOA mode is confirmed, use `AncID:` and `pdoaOffset:` keys, not `ID:` and `Role:`.

**Phase:** Firmware maintenance across all phases.

---

### P-11: Multipath from walls degrades PDOA angle accuracy non-uniformly

**Risk:** PDOA measures phase difference of the first-path signal. In an indoor environment
with reflective walls within 1 m of the anchor, the reflected signal can arrive within the
DW3000 accumulator window and corrupt the phase estimate. This manifests as **angle errors
that are position-dependent**: the measurement is accurate in one spot and off by 15–30°
in another, making calibration appear to work at one test point but fail at others.

BU04's DW3000 uses STS (Scrambled Timestamp Sequence) mode which provides some multipath
rejection, but not immunity. The effect is worst when:
- Anchor is mounted on a metallic surface (robot chassis).
- Tag is held below 50 cm height (floor reflection dominant).
- Operating range is > 3 m (lower SNR, less first-path dominance).

**Warning signs:**
- Grid accuracy test shows consistent error in one quadrant only.
- Error improves when you move anchor away from a wall.
- `Range` reads correctly while `Angle` is wrong (ToF is multipath-resistant, PDOA is not).

**Prevention:**
- During calibration, keep both anchor and tag at consistent height (1.0–1.2 m).
- Keep anchor ≥ 1.5 m from reflective walls.
- Mount anchor on a 3D-printed plastic bracket to isolate from metal robot chassis.

**Phase:** Phase 0 (calibration setup), Phase 3 (robot integration).

---

## Minor Pitfalls (good to know)

### P-12: `AT+SAVE` causes 3 s reboot — commands queued after SAVE may be lost

**Risk:** `AT+SAVE` calls `NVIC_SystemReset()` on the STM32F103 after writing NVM. Any AT
command sent in the 0–200 ms after `AT+SAVE` may be received by the STM32 RX buffer before
the reset, or lost entirely if the reset clears the UART FIFO before the command is
processed. In practice, only commands sent as a burst *before* `AT+SAVE` are safe (they sit
in the STM32 UART RX FIFO and are processed after the reboot from NVM). Commands sent *after*
the `AT+SAVE` call (i.e., the next `bu04.println()` in firmware) are NOT reliably received.

The existing burst pattern (`bu04.print("AT+SETUWBMODE=1\r\n"); bu04.print("AT+SAVE\r\n")`)
is correct precisely because SAVE is last. The risk is in code added later that sends
configuration after SAVE.

**Prevention:** Always put `AT+SAVE` as the last command in any configuration sequence. Never
add commands after SAVE without inserting `waitBU04ReadyForever()` first.

**Phase:** Firmware configuration sequences everywhere.

---

### P-13: TWR y-coordinate always non-negative — tag behind baseline undetectable

**Risk:** In TWR mode, `y = sqrtf(d1² - x²)`. The square root is always ≥ 0, so the tag
behind the anchor baseline (y < 0) produces the same output as the tag in front of it.
The `PLATFORM` CSV row shows the tag mirrored to the positive-y side. The code already
has a comment noting this; it is not a regression risk for PDOA mode but will produce
wrong robot direction if TWR mode is used as a fallback.

**Prevention:** Accept the constraint for the test bench (tag stays in front of the anchor
line). For production, a third anchor or the PDOA angle sign is needed to disambiguate.

**Phase:** TWR mode only (not PDOA). Relevant if TWR is used as fallback.

---

### P-14: `ANCHOR1_MAC` broadcast placeholder breaks ESP-NOW in TWR mode

**Risk:** `ANCHOR1_MAC` defaults to `{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}` (broadcast).
`isMacDefault()` checks this and prints a warning, but does not prevent `esp_now_add_peer()`
from being called with the broadcast address. On ESP-IDF 5.x, `esp_now_add_peer()` with
the broadcast address returns `ESP_ERR_ESPNOW_ARG` and the `Serial.println("# ESP-NOW add peer ОШИБКА")`
fires — ESP-NOW data never reaches ANCHOR1. The TWR PLATFORM row will never appear.

**Warning signs:**
- Serial output of ANCHOR2 shows `# ESP-NOW add peer ОШИБКА` on every boot.
- ANCHOR1 serial never shows `PLATFORM,...` rows despite ANCHOR2 showing `ANCHOR2,...` rows.

**Prevention:** Flash ANCHOR1 first, copy the printed MAC, paste into `config.h`, then flash
ANCHOR2. This is documented in `config.h` — the risk is that it is easy to skip during
quick re-flashing iterations.

**Phase:** TWR mode setup only.

---

### P-15: `Range` field from BU04 PDOA output is in centimetres, not metres

**Risk:** The BU04 PDOA JSON output (`Tag_Addr:XXXX, ..., Range:V, Angle:N`) uses **cm** for
`Range` and the `Xcm`/`Ycm` fields. The existing parse code correctly divides by 100.0f:

```cpp
Serial.printf("PDOA,%04X,%d,%.2f,%d,%.2f,%.2f,%lu\n",
              addr, seq,
              range / 100.0f, angle,   // ← correct
              xcm  / 100.0f,
              ycm  / 100.0f,  now);
```

The pitfall is that any future re-implementation that re-reads the raw BU04 output (e.g., a
Python direct-to-BU04 pass-through logger) will see `Range:154` and may treat it as 154 m
rather than 1.54 m, producing wildly out-of-range readings that pass the `MAX_VALID_DIST_M`
check (50.0f) but are 100× wrong.

**Prevention:** Add a comment to the AT+GETCFG / PDOA output documentation section in `config.h`
noting that Range/Xcm/Ycm are in cm. If Python ever reads BU04 directly, add an explicit
`/100.0` conversion.

**Phase:** Phase 2 (Python logger), any future direct-BU04 integration.

---

### P-16: `FuncAnimation` with `interval=100` runs at wall-clock rate, not data rate

**Risk:** `FuncAnimation(fig, update, interval=100)` calls `update()` every 100 ms regardless
of whether new data arrived. With PDOA at 5 Hz (200 ms), every other frame has no new data
and just redraws the same points. With a heavy `ax.clear()` pattern (P-6), this doubles the
unnecessary CPU work. The opposite risk: if the serial reader thread is slow and `buf` is
always empty when `update()` runs, the plot appears frozen even though data is arriving.

**Prevention:**
Use an event flag or check `buf` length in `update()` and early-return with no redraw if
nothing new arrived. Set `interval=50` (20 Hz) to keep the display responsive without
increasing actual redraw cost if artist updates (P-6 fix) are used.

**Phase:** Phase 2 (Python visualizer).

---

## Phase-Specific Warning Summary

| Phase | Highest-Risk Pitfall | Must-Address-Before-Starting |
|-------|---------------------|------------------------------|
| Phase 0: Firmware bringup | P-5 (cold-start timing), P-10 (GETCFG format) | Keep 3 s post-ready delay |
| Phase 1: PDOA telemetry + calibration | P-1 (offsets not sent), P-2 (manual ADDTAG), P-7 (seq wrap) | Implement offset send + auto-ADDTAG first |
| Phase 2: Python visualizer | P-3 (DTR reset), P-6 (ax.clear cost), P-15 (unit mismatch) | Open serial with `dtr=False` |
| Phase 3: Robot following | P-4 (±60° saturation), P-11 (multipath), P-13 (y ambiguity in TWR fallback) | Add saturation guard before motor commands |

---

## Sources

- BU04 AT指令 V1.0.6 (sections 3–5: SETCFG, SETUWBMODE, ADDTAG, GETDLIST, PDOAOFF/RNGOFF)
- Qorvo APS014 "DW3000 Antenna Delay Calibration" — PDOA angle range and systematic offset
- Qorvo DW3000 User Manual section 9 — PDOA operating range ±60°, first-path / multipath behavior
- Existing codebase: `src/main.cpp` (configureBU04 2-phase, PDOA parsing, TWR trilateration)
- `include/config.h` — AT_PDOAOFF / AT_RNGOFF defined but unused
- ESP32-C3 USB CDC DTR behavior: arduino-esp32 issue #5359, pyserial docs §2.6 (dtr parameter)
- matplotlib FuncAnimation polar blit limitation: matplotlib issue #16478
