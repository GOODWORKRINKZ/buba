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

### 🔬 2026-06-08 16:15 — Deep SDK Analysis: AT Mode Architecture

**Critical discovery from `main.c` (line 110):**
```c
while (app.pConfig->s.userConfig.nodeAddr == 0xFFFF) {
    App_Module_Sys_Work_Mode_Event();  // AT command loop
}
```

**This means:** BU04 stays in AT command loop ONLY while `nodeAddr == 0xFFFF`. Once `AT+SETCFG` sets nodeAddr, BU04 exits AT loop and calls `node_start()`/`tag_start()`.

**From `cmd_fn.c` (line 79-141):**
```c
int f_setcfg(...) {
    sys_para.param_Config.s.userConfig.nodeAddr = id;  // sets nodeAddr
    // ...
    if (role == 1) node_start();  // IMMEDIATELY exits AT loop
    else tag_start();             // IMMEDIATELY exits AT loop
}
```

**Conclusion: RAM-only SETCFG is IMPOSSIBLE.** SETCFG always triggers node_start()/tag_start() and exits AT mode. There is no way to set config in RAM without starting the UWB stack.

**From `Generic.c` (line 162):**
```c
void App_Module_Sys_Work_Mode_Event() {
    Sys_Work_Mode mode = (nodeAddr != 0xffff) ? WORK_DONE : CFG_ING;
    App_Module_Sys_Deal_UART_CMD_Event(mode);  // processes AT commands
    App_Modelu_Sys_Deal_IO_LED_Event(mode);
}
```

**The ONLY way to stay in AT mode forever:** `AT+SETWORKMODE=1`
- From `main.c` (line 140): `if (workmode == 1) { while(1) { App_Module_Sys_Work_Mode_Event(); } }`
- This is "AT-only mode" — BU04 never calls node_start()/tag_start()
- Used for tag configuration where tag_start() fails without reset_DWIC

**Revised tag initialization strategy:**
1. `AT+SETWORKMODE=1` → BU04 stays in AT loop forever
2. `AT+SETCFG=id,0,ch,rate` → sets config in RAM, but workmode=1 prevents node_start()
3. `AT+SAVE` → writes NVM + NVIC_SystemReset
4. After reboot: workmode=1, nodeAddr=id, role=0 → stays in AT loop
5. `AT+SETWORKMODE=0` → exits AT loop, calls tag_start() with DW3000 already initialized

**This is the correct 2-phase strategy for tags.**

### 🔬 2026-06-08 17:00 — AT-bridge live investigation findings

**Setup:** AT-bridge on tag device, direct AT commands from console.

**Key observations from boot log:**
```
UWB Module 15:13:27 Jul 19 2024
UWB Module V1.0.0
UWB Module 1111 8 10 32   ← panID, numSlots, numTags, maxTags
UWB Module flag:AAAA        ← NVM valid
UWB Module start_count:87   ← factory test counter, not a bug
UWB Module bit1:0..bit5:0   ← zero error flags, no HardFault
rngOffset_mm:0, pdoaOffset_deg:0, motionfilter:1, user_cmd:0
slot:0, addr:0000           ← s.s_pdoa.addr (PDOA anchor ID) = 0
```

**IIC Error is NORMAL:** No OLED/accelerometer on bare BU04 module (no dev-kit). I2C init fails silently and execution continues. NOT related to DW3000.

**workmode response format:** "workmode: 1" followed by boot log digits creates "workmode: 17" artifact. Actual workmode = 1. AT-bridge reads everything together.

**start_count = 87:** factory test, incremented on every valid boot. Normal for new module.

**Two AT parsers confirmed:**
- `cmd_fn.c` (workmode=0): handles AT+SETCFG, AT+GETCFG, AT+SAVE etc.
- `aitcmd.lib` (workmode=1): handles only PDOA commands, NOT AT+SETCFG/AT+SAVE

**AT+SETCFG confirmed deadlock:** SETCFG sets nodeAddr in RAM then immediately calls tag_start() → dwt_initialise() → INIT FAILED → while(1) → watchdog → reboot → NVM unchanged. SAVE never called.

**AT+SAVE in workmode=1:** returns ERROR — aitcmd.lib does not know this command.

**Root conclusion:** tag_start() does NOT call reset_DWIC() (confirmed from SDK comments and Project.htm call graph analysis). node_start() and all examples call reset_DWIC() first. This is by design — tag_start() expects DW3000 already initialized. The ONLY path to get DW3000 initialized for tag role: flash via SWD/JTAG with modified firmware, or use anchor role first then switch.

---

## 🔬 2026-06-08 17:17 — CRITICAL HARDWARE FIX: VDDAON Jumper

**Problem:** DW3000 consistently failed to initialize with `INIT FAILED` error on both modules, regardless of firmware or AT commands.

**Root Cause:** VDDAON pin on BU04 module was not connected to 3.3V power supply. VDDAON powers the always-on domain of DW3000 which is required for proper initialization sequence.

**Solution:** Added jumper wire from 3.3V to VDDAON pin on BU04 module.

**Result:** 
- DW3000 now initializes successfully: `标签 v1.0 标签初始化成功 0212006150A2DD9A` (Tag v1.0 Tag initialization successful)
- EUI-64 address: `02:12:00:61:50:A2:DD:9A`
- Successfully configured as PDOA anchor: `ID:0, Role:1, CH:1, Rate:1`
- Configuration saved to NVM: `基站初始化成功` (Anchor initialization successful)

**Impact:** This hardware fix bypasses all ESP32 firmware issues. Direct AT command configuration through STM32 inside BU04 now works correctly.

**Next Steps:** 
1. Apply same VDDAON jumper fix to second module
2. Configure second module as PDOA tag
3. Test PDOA ranging between anchor and tag
4. Document VDDAON connection in hardware setup guide

---

## ✅ 2026-06-09 — PDOA SYSTEM WORKING! Direct USB + Utility Reverse Engineering

### Direct USB Connection (PA11/PA12)

**Discovery:** BU04 module can be connected directly to PC via USB through STM32F103 pins PA11 (USB_DM) and PA12 (USB_DP). The stock firmware includes USB CDC (Virtual COM Port) — appears as `0483:5740 STMicroelectronics Virtual COM Port`.

**Soldering:** PA11 (D-), PA12 (D+), GND, and 3.3V from USB cable to BU04 module pins.

**Key distinction:** Direct USB CDC has NO `>` echo prefix. ESP32 at_bridge HAS `> AT+...` echo. This is the reliable way to identify which port is which.

**⚠️ Pitfall:** `AT+RST` (software reset via NVIC_SystemReset) kills USB CDC — STM32 reboots but USB doesn't re-enumerate. Requires physical USB power-cycle (unplug/replug). Use `AT+SAVE` (which auto-reboots via NVIC_SystemReset too) sparingly on direct USB modules.

### Ai_Thinker_PDOA_V1_0_1 Utility Reverse Engineering

**Source:** Serial port capture (SUDT AccessPort log) from working Windows session.

**Exact initialization sequence used by official utility:**

| # | Time | Command | Response |
|---|------|---------|----------|
| 1 | 0.0s | `AT+DECA$` | `JS008D{"Info":{"Device":"PDOA Node",...}}` |
| 2 | 1.1s | `AT+GETKLIST` | `{"KList":[]}` |
| 3 | 2.0s | Poll: `AT+GETDLIST` + `AT+GETKLIST` | Both empty |
| 4 | ~18s | (spontaneous) | `JS001D{"NewTag":"043400086DD3657B"}` ← **auto-discovery!** |
| 5 | 18.2s | `AT+ADDTAG=043400086dd3657b,657b,0001,64,00` | `JS0064{"TagAdded":{...}}` |
| 6 | 18.3s+ | (streaming) | `{"TWR":{"a16":"657B","R":0,"D":121,"P":4,...}}` every ~100ms |

**Key insight:** The utility does NOT send `AT+PDOASETCFG` at all! It relies on:
1. Anchor being pre-configured (PAN ID, AncID saved in flash)
2. Anchor auto-discovers tags via UWB beacon/response
3. Utility just polls `AT+GETDLIST` and adds discovered tags via `AT+ADDTAG`
4. After ADDTAG, PDOA streaming starts automatically

**AT+ADDTAG format:** `AT+ADDTAG=<addr64_hex>,<addr16_hex>,<fast_rate>,64,<mode>`
- addr64: 16-char lowercase hex (e.g., `043400086dd3657b`)
- addr16: 4-char lowercase hex = last 4 chars of addr64 (e.g., `657b`)
- fast_rate: `0001` (= 1, minimum refresh interval)
- 64: fixed (maximum refresh interval)
- mode: `00` (= 0, tag mode)

### PAN ID Decimal Trap

**From `cmd_fn.c`:** `panID = atoi(argv[2])` — PAN ID is parsed as **decimal** integer, but DISPLAYED as hex (`%04X`).

| Input | Stored | Display | Correct? |
|-------|--------|---------|----------|
| `1111` | 1111₁₀ = 0x0457 | `Net:0457` | ❌ Wrong! |
| `4369` | 4369₁₀ = 0x1111 | `Net:1111` | ✅ Correct! |

**Correct command:** `AT+PDOASETCFG=1,1,4369,1,100,1,0` → PAN ID = 0x1111

### Final Working Configuration (2026-06-09)

**Port mapping:**
| Port | Path | Role | ID | FW |
|------|------|------|----|----|
| `/dev/ttyACM1` | BU04 direct USB (PA11/12) | ⚓ ANCHOR | 1 | Stock STM32 PDOA |
| `/dev/ttyACM0` | ESP32-C3 + BU04 | 🏷 TAG | 2 | at_bridge + Stock STM32 PDOA |

**Anchor config:**
```
GETCFG:    ID:1, Role:1, CH:1, Rate:1
PDOAGETCFG: Dlist:1 KList:1 Net:1111 AncID:1 Rate:100 Filter:1 UserCmd:0
UWBMODE:   twr_pdoa_mode: 1
```

**Tag config:**
```
GETCFG:    ID:2, Role:0, CH:1, Rate:1
PDOAGETCFG: Net:1111 AncID:65535 (default, not needed for tag)
UWBMODE:   twr_pdoa_mode: 1
```

**Tag address:** EUI-64 = `043400086DD3657B`, Short = `657B`

### PDOA Performance Results

```
Sample measurements (10 sec window, tag at ~135cm):
  D=136cm P=0°    D=136cm P=6°    D=135cm P=32°
  D=137cm P=19°   D=136cm P=7°    D=138cm P=17°
  ...
  41 samples / 10 sec ≈ 4.1 Hz update rate
  Distance stability: ±5cm around 135cm
  Angle range: -1° to +38° (tag moving in front of anchor)
```

### ESP32 Firmware Status

**Anchor firmware (`anchor1_pdoa`):** NOT NEEDED for direct USB setup. Stock STM32 firmware handles PDOA natively. ESP32-C3 only needed for UART bridge.

**Tag firmware (`tag`):** Partially working. Tag BU04 can be configured via AT commands through ESP32 at_bridge. The ESP32 `configureBU04()` function still has INIT FAILED retry issues, but bypassed by direct AT configuration.

**at_bridge firmware:** WORKING ✅. Successfully relays AT commands from USB Serial → BU04 UART and back. Useful as a "remote control" for the tag BU04.

### Key Files Modified This Session

| File | Change |
|------|--------|
| `scripts/bu04_terminal.py` | NEW — Interactive AT command terminal for direct USB BU04 |
| `.planning/phases/01-firmware-fixes/01-DISCUSSION-LOG.md` | This entry |

### Lessons Learned

1. **Direct USB beats ESP32 for configuration.** Stock STM32 firmware is reliable; ESP32 firmware adds unnecessary complexity for basic PDOA operation.
2. **Always use `AT+DECA$` first** to verify PDOA node is alive and get device info.
3. **Wait for auto-discovery.** Tag appears in DList within 2-18 seconds after anchor boot — no special trigger needed.
4. **AT+RST = USB death.** On direct-USB modules, use physical power cycle instead.
5. **PAN ID is decimal in, hex out.** `atoi()` parses as decimal; display uses `%04X`.
6. **VDDAON jumper is mandatory.** Both modules need it for DW3000 to initialize.
7. **IIC Error is noise.** Ignore `IIC Error` messages — no OLED/accelerometer on bare BU04.
