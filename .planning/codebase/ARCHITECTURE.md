---
last_updated: 2026-05-15
focus: arch
---

# Architecture

## Pattern

**Single-file multi-role firmware.** All four production roles (ANCHOR1, ANCHOR2, TAG, ANCHOR1_PDOA) compile from one source file (`src/main.cpp`). Role selection is done entirely at compile time via `-D` build flags in `platformio.ini`. No runtime role switching. Arduino framework (`setup()` / `loop()`) is the execution model.

## System Roles

| Role | Build Flag | Description |
|------|------------|-------------|
| `anchor1` | `-DROLE_ANCHOR1=1` | Main anchor on the platform. Queries `AT+DISTANCE`, receives `d2` from ANCHOR2 via ESP-NOW, performs trilateration to compute x/y/angle, outputs `PLATFORM,...` CSV. |
| `anchor1_pdoa` | `-DROLE_ANCHOR1=1 -DPDOA_MODE=1` | Same ESP32 binary entry point as ANCHOR1, but switches BU04 to `AT+SETUWBMODE=1`. One anchor measures both distance and angle using phase-difference of arrival (PDOA). No ANCHOR2 or ESP-NOW needed. |
| `anchor2` | `-DROLE_ANCHOR2=1` | Slave anchor. Queries `AT+DISTANCE`, sends `d2` to ANCHOR1 via ESP-NOW (`EspNowPkt`), outputs `ANCHOR2,...` CSV. |
| `tag` | `-DROLE_TAG=1` | User-held tag. Queries `AT+DISTANCE` as the UWB initiator, applies moving average, outputs `TAG,...` CSV. Requires a 2-phase BU04 configuration sequence due to DW3000 cold-start constraints. |

## UWB Modes

### TWR Mode (default — 3 devices)

Two anchors on the platform each measure distance to the tag via `AT+DISTANCE` (Two-Way Ranging). ANCHOR2 forwards its measurement (`d2`) to ANCHOR1 wirelessly via ESP-NOW (`EspNowPkt` struct). ANCHOR1 computes tag position using trilateration:

```
ANCHOR1(0,0) ──── B ──── ANCHOR2(B,0)
              TAG(x, y)

x = (d1² − d2² + B²) / (2·B)
y = √(d1² − x²)
β = atan2(y, x)
```

`B` = `BASELINE_M` (default 0.30 m), configured in `include/config.h`.

### PDOA Mode (`-DPDOA_MODE=1`)

Single anchor with two antennas. BU04 is configured with `AT+SETUWBMODE=1`. The module autonomously measures angle via Phase Difference of Arrival and streams `Tag_Addr:XXXX, Seq:N, Xcm:V, Ycm:V, Range:V, Angle:V` lines over UART. No second anchor or ESP-NOW. Tags must be registered with `AT+ADDTAG`. BU04 reports position in cm; firmware converts to metres for CSV output.

## Data Flow

### TWR Mode

```
BU04 (DW3000 UWB ranging)
    │ UART AT+DISTANCE response
    ▼
ESP32 ANCHOR2: parseDistance() → d2
    │ ESP-NOW EspNowPkt{magic, dist_m, ts_ms}
    ▼
ESP32 ANCHOR1: onRecv() → g_d2_m / g_d2_age
    │
    ├── ANCHOR1 also queries AT+DISTANCE → d1
    ├── trilateration (d1, d2, BASELINE_M) → x, y, angle
    │
    ▼
Serial (USB CDC 115200 baud)
  ANCHOR1,seq,d1_m,avg_m,ts_ms
  PLATFORM,seq,d1_m,d2_m,angle_deg,x_m,y_m,ts_ms
```

### PDOA Mode

```
BU04 (DW3000 PDOA, 2 antennas)
    │ UART streaming lines: Tag_Addr:XXXX, Seq:N, Xcm:V, Ycm:V, Range:V, Angle:V
    ▼
ESP32 ANCHOR1: fieldVal() manual parser → addr, range, angle, x, y
    ▼
Serial (USB CDC 115200 baud)
  PDOA,addr_hex,seq,range_m,angle_deg,x_m,y_m,ts_ms
```

### BU04 Configuration Flow (startup)

```
setup()
  │
  ├── waitBU04Ready()  ← ping AT until OK (up to 15 s)
  ├── delay(3000)      ← NVM + DW3000 init
  │
  ├── [ANCHOR TWR] configureBU04(id, role=1)
  │     └── AT+GETCFG check → AT+SETCFG + AT+SAVE burst → power-cycle BU04
  │
  ├── [TAG]  configureBU04(id, role=0)
  │     └── 2-phase strategy:
  │           Phase 1: set as PDOA-anchor (role=1) → SAVE → reboot (DW3000 init)
  │           Phase 2: switch to tag (role=0) from working PDOA context → SAVE → reboot
  │
  └── [PDOA anchor] burst SETCFG + SETUWBMODE=1 + SAVE → reboot to PDOA branch
```

## Entry Points

All roles share the same `setup()` / `loop()` Arduino entry points in `src/main.cpp`. Role-specific code is wrapped in `#if defined(ROLE_*)` blocks.

**`setup()` responsibilities per role:**
- All: `Serial.begin()`, `bu04.begin()` (UART1), `waitBU04Ready()`, `configureBU04()`
- ANCHOR1/ANCHOR2: `initESPNow()` (WiFi STA + esp_now_init + peer registration)
- ANCHOR1 PDOA: burst SETUWBMODE + SAVE, `sendAT(AT_USER_CMD_JSON)`

**`loop()` responsibilities per role:**
- All: poll every `POLL_INTERVAL_MS` (200 ms) by incrementing `g_seq`
- ANCHOR1 TWR: `sendAT(AT_DISTANCE)` → `parseDistance()` → `updateAvg()` → trilateration → Serial CSV
- ANCHOR2: `sendAT(AT_DISTANCE)` → `parseDistance()` → `esp_now_send()` → Serial CSV
- TAG: `sendAT(AT_DISTANCE)` → `parseDistance()` → `updateAvg()` → Serial CSV
- ANCHOR1 PDOA: drain UART buffer → parse `Tag_Addr:` lines → Serial CSV; pass-through Serial→BU04

## Key Abstractions

**`EspNowPkt` struct** (`src/main.cpp`, compiled only for ANCHOR1/ANCHOR2):
```c
struct __attribute__((packed)) EspNowPkt {
    uint16_t magic;   // ESPNOW_MAGIC = 0xBB04 — packet guard
    float    dist_m;
    uint32_t ts_ms;
};
```
Magic guard protects against stray ESP-NOW packets from other devices.

**`sendAT(cmd, timeoutMs)`** — sends an AT command over `bu04` UART, collects response until `"OK"` or `"ERR"` or timeout. Returns raw `String`.

**`parseDistance(resp)`** — parses `"distance: X.XXX"` or `"distance:ID X.XXX"` formats from AT+DISTANCE response. Validates range `(0, MAX_VALID_DIST_M]`. Returns `-1.0f` on failure.

**`updateAvg(v)`** — circular-buffer moving average over `MOVING_AVG_SAMPLES` (5) readings, stored in module-static `g_avgBuf[]`.

**`configureBU04(id, role)`** — idempotent BU04 configuration. Checks current config via AT+GETCFG; returns immediately if already correct. For anchors: burst SETCFG+SAVE. For tags: 2-phase strategy to work around DW3000 cold-start limitation.

**`waitBU04Ready()` / `waitBU04ReadyForever()`** — polls `AT` command until BU04 responds `OK`. Used after power-on and after SAVE-triggered reboot.

**`fieldVal(key)`** lambda — manual key-value parser for PDOA `Tag_Addr:` lines (no sscanf on Arduino).

**AT command string macros** — defined in `include/config.h`: `AT_TEST`, `AT_SAVE`, `AT_GETCFG`, `AT_GETVER`, `AT_RESTART`, `AT_RESTORE`, `AT_DISTANCE`, `AT_USER_CMD_JSON`, etc.

## Hardware Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    BU04 Module                              │
│  DW3000 (UWB chip) ← SPI → STM32F103 (AT firmware)        │
│                                                             │
│  PA2 (USART2_TX, pin 4) ──────────────────► GPIO3 (RX)    │
│  PA3 (USART2_RX, pin 5) ◄────────────────── GPIO2 (TX)    │
│  3V3 (pins 23/24/34)    ──────────────────► 3V3 (≥500mA)  │
│  GND (pins 1/10)        ──────────────────► GND            │
└───────────────────────────┬─────────────────────────────────┘
                            │ UART AT commands @ 115200 baud
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                  ESP32-C3 SuperMini                         │
│  HardwareSerial bu04(1) — UART1                            │
│  WiFi / ESP-NOW (anchors only)                             │
│  USB CDC Serial — telemetry output @ 115200 baud           │
└─────────────────────────────────────────────────────────────┘
                            │ USB CDC
                            ▼
                      Host PC / telemetry consumer
```

**Key interface facts:**
- DW3000 is **not** directly accessible from ESP32. All UWB operations go through STM32F103 AT command interface.
- Alternative UART pins: PA9 (USART1_TX) → GPIO3, PA10 (USART1_RX) → GPIO2 (if USART2 unresponsive).
- BU04 requires `AT+SAVE` + reboot to persist configuration to NVM. Saving triggers `NVIC_SystemReset` on the STM32.
- TWR TAG and ANCHOR1 **must share the same BU04 ID** (`BU04_ID_TAG = BU04_ID_ANCHOR1 = 0`) — TWR protocol requirement.

## Architectural Constraints

- **No RTOS:** Pure Arduino event loop. `loop()` blocks on `sendAT()` for up to 800 ms per measurement cycle.
- **Global state:** `g_seq`, `g_avgBuf[]`, `g_avgIdx`, `g_avgFull` are module-static in `src/main.cpp`. ANCHOR1 also uses `g_d2_m` and `g_d2_age`.
- **ESP-NOW callback (`onRecv`) runs in WiFi task context** — writes `g_d2_m` / `g_d2_age` without mutex. Safe only because ESP32-C3 is single-core.
- **Compile-time role selection only** — no runtime switching. Wrong build environment = wrong firmware.
- **BU04 configuration is stateful across reboots** (NVM). If BU04 is already correctly configured, `configureBU04()` returns immediately; if not, it may require a physical power-cycle of the BU04 (not the ESP32).

## Anti-Patterns

### Do not use `delay()` in new code paths
**What happens:** `loop()` uses `delay(2)` inside PDOA read loop and `sendAT()` uses `delay(10)`.
**Why it's wrong:** Blocks the loop and prevents Serial pass-through from being responsive during AT waits.
**Do this instead:** Use non-blocking state machines or shorten polling windows.

### Do not add esp_now_send without magic guard validation
**What happens:** `EspNowPkt.magic = ESPNOW_MAGIC (0xBB04)` validated on receive in `handlePkt()`.
**Why it's wrong:** Without the guard, any ESP-NOW broadcast on the same channel corrupts `g_d2_m`.
**Do this instead:** Always set and verify `ESPNOW_MAGIC` in any new packet type.

---

*Architecture analysis: 2026-05-15*
