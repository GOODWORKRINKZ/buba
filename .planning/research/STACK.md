# Stack Research — UWB PDOA Test Bench

**Project:** BU04 UWB Positioning & Robot Following  
**Researched:** 2026-05-15  
**Overall confidence:** MEDIUM-HIGH (hardware-specific AT command details are LOW; PC-side stack is HIGH)

---

## Recommended Stack (2025/2026)

### PC-side Visualization

| Layer | Choice | Version | Rationale |
|-------|--------|---------|-----------|
| Serial I/O | **pyserial** | 3.5 (installed) | De facto standard; `readline()` maps perfectly to the CRLF-terminated CSV lines the firmware emits |
| Plot framework | **matplotlib** | 3.10.7 (installed) | Already installed; FuncAnimation + blit=True is fast enough for 5 Hz PDOA updates |
| Polar plot | `matplotlib.projections.polar` | same | Built-in; one subplot shows angle + distance simultaneously — exactly what PDOA needs |
| CSV logging | Python `csv` stdlib | — | No extra dependency; append-mode for post-session error analysis |

**Recommended pattern** (serial reader thread + matplotlib FuncAnimation):

```python
import serial, threading, collections, csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation

buf = collections.deque(maxlen=200)          # ring buffer
latest = {"range": 0.0, "angle": 0.0}

def reader(port):
    with serial.Serial(port, 115200, timeout=1) as ser:
        for line in ser:
            line = line.decode(errors="ignore").strip()
            if line.startswith("PDOA,"):
                # PDOA,addr_hex,seq,range_m,angle_deg,x_m,y_m,ts_ms
                parts = line.split(",")
                if len(parts) == 8:
                    latest["range"] = float(parts[3])
                    latest["angle"] = float(parts[4])
                    buf.append((latest["range"], latest["angle"]))

threading.Thread(target=reader, args=("/dev/ttyACM0",), daemon=True).start()

fig = plt.figure(figsize=(7, 6))
ax = fig.add_subplot(111, projection="polar")

def update(_):
    ax.clear()
    if buf:
        angles  = [a * 3.14159 / 180 for _, a in buf]
        ranges  = [r for r, _ in buf]
        ax.scatter(angles, ranges, s=8, alpha=0.6)
        # highlight latest
        ax.scatter([latest["angle"] * 3.14159 / 180], [latest["range"]],
                   s=60, color="red", zorder=5)
    ax.set_rmax(4.0)
    ax.set_title(f"PDOA  r={latest['range']:.2f}m  θ={latest['angle']:.1f}°")

ani = animation.FuncAnimation(fig, update, interval=100, blit=False)
plt.tight_layout()
plt.show()
```

> `blit=False` on polar — matplotlib's polar blitting is broken on some backends.
> 100 ms interval = 10 Hz refresh, comfortable above the 5 Hz PDOA output rate.

**Do NOT use for this project:**

| Tool | Why not |
|------|---------|
| Plotly Dash | Server + browser overhead; overkill for a single-PC bench test |
| Jupyter + ipywidgets | FuncAnimation in Jupyter is unreliable for sustained serial streaming |
| pyqtgraph | Excellent choice if refresh rate needs to exceed ~15 Hz or if a richer UI is needed, but adds PyQt6/PySide6 dependency with no payoff at 5 Hz; consider for robot following phase |
| tkinter canvas | Manual draw loop, more code than FuncAnimation for no benefit |

---

### Firmware Side

| Component | Choice | Rationale |
|-----------|--------|-----------|
| Build system | **PlatformIO** (existing) | `pio run -e anchor1_pdoa -t upload` already works |
| Framework | **Arduino** on `espressif32` (existing) | Fastest iteration; no FreeRTOS overhead needed at 5 Hz PDOA rate |
| Board | `esp32-c3-devkitc-02` (existing) | Matches the SuperMini hardware |
| Serial to BU04 | `HardwareSerial(1)` UART1 (existing) | UART1 on GPIO 2/3; proven wiring |
| USB to PC | Native USB CDC (existing) | `ARDUINO_USB_CDC_ON_BOOT=1` + `ARDUINO_USB_MODE=1` already set correctly |

#### ESP32-C3 USB CDC Gotchas (2025)

1. **`ARDUINO_USB_MODE=1` = TinyUSB JTAG/CDC mode.**  
   `Serial` object IS the USB CDC FIFO. Baud rate in pyserial is ignored (virtual CDC), but you must still pass *some* baud value; 115200 is conventional.

2. **Early-boot log loss.**  
   USB CDC is not enumerated until ~500 ms after power-on. Any `Serial.print` before the host enumerates the port is silently dropped. Add `while (!Serial && millis() < 3000);` in `setup()` if you need startup diagnostics.

3. **`Serial.flush()` blocks until TX FIFO drains.**  
   Don't call it in tight loops. The firmware currently uses `Serial.println()` without explicit flush — this is correct; Arduino's USB CDC driver drains automatically.

4. **DTR/RTS-triggered reset.**  
   pyserial sets DTR=True on `serial.Serial(...)`. This reboots the ESP32-C3 (same as pressing RST). This is intentional for flashing but can surprise a visualization script. Workaround:

   ```python
   ser = serial.Serial(port, 115200)
   ser.setDTR(False)   # prevent accidental reboot
   ```

5. **Linux permissions.**  
   User must be in `dialout` group: `sudo usermod -aG dialout $USER` (requires re-login).

6. **PlatformIO `monitor_port` auto-detection.**  
   On ESP32-C3 with native USB, the flashing port and monitor port are the same `/dev/ttyACMx`. PlatformIO handles this automatically since v6.1. No manual port juggling needed.

---

### Calibration Tools

The BU04 PDOA calibration flow uses two AT commands sent at firmware startup:

#### `AT+PDOAOFF=<value>`
- **Purpose:** Corrects the systematic phase difference offset introduced by PCB trace length asymmetry between the two antenna paths inside the BU04 module.  
- **Unit:** Degrees (floating point or integer, depends on BU04 firmware version; AT指令 V1.0.6 accepts integer).  
- **Typical range:** ±30°. Manufacturing variation on the BU04's dual-antenna layout produces offsets in this ballpark. Modules from the same production batch tend to cluster within ±10° of each other.  
- **Calibration procedure:**
  1. Mount anchor with known orientation (BU04 antenna plane facing the tag).
  2. Place tag at 0° (straight ahead, equidistant from both antennas).
  3. Send `AT+PDOAOFF=0`, read reported angle.
  4. Set `AT+PDOAOFF=-<reported_angle>` to zero it out.
  5. Verify at ±45°, ±90°. Expect ±15° accuracy after calibration at 1–3 m LOS.
- **Config.h current value:** Not yet determined (calibration pending). Start with `AT+PDOAOFF=0` and empirically adjust.

#### `AT+RNGOFF=<value>`
- **Purpose:** Corrects systematic ranging bias from fixed cable/trace group delay inside the BU04.  
- **Unit:** Centimetres (integer).  
- **Typical range:** −20 cm to +20 cm. The DW3000 datasheet (APS011 application note) documents antenna delay calibration producing offsets of this magnitude for assembled modules.  
- **Calibration procedure:**
  1. Place tag at exactly 1.000 m from the BU04 antenna midpoint (use a ruler/tape).
  2. Send `AT+RNGOFF=0`, note average reported range over 30 readings.
  3. Compute `correction = 100 - mean_cm`. Set `AT+RNGOFF=<correction>`.
  4. Verify at 2 m and 3 m; expect ≤ ±10 cm residual error.

#### Recommended config.h calibration constants (placeholder, must be measured)

```c
// ---- Calibration: send AT+PDOAOFF and AT+RNGOFF at startup -----
// Values are module-specific; measure empirically on your bench.
// Initial safe default is 0; refine by following the procedure in STACK.md.
#define PDOA_OFFSET_DEG    0      // signed int, degrees; typical range ±30
#define RANGE_OFFSET_CM    0      // signed int, cm; typical range ±20
```

**Note:** The current firmware (`main.cpp`) defines `AT+PDOAOFF`/`AT+RNGOFF` constants in `config.h` but does **not send them at startup** (confirmed from PROJECT.md). Sending them is an active task. Both commands must be sent *after* `AT+SETUWBMODE=1` and before `AT+USER_CMD=0`.

---

## Alternatives Considered

### Python Visualization

| Alternative | Verdict | Detail |
|-------------|---------|--------|
| **pyqtgraph** | Defer to robot phase | 5–10× faster redraws than matplotlib due to OpenGL backend; significantly better for >15 Hz or drag-interactive plots. Requires PyQt6/PySide6. Worthwhile when PDOA is feeding a live robot controller display. |
| **Plotly Dash** | No | Browser-based, adds `flask`/`dash` dependency, latency not suited for real-time serial streaming |
| **Dear PyGui** | No | Gaming-grade GPU renderer, but zero advantage for a simple polar scatter; heavyweight for a test bench script |
| **pygame** | No | Requires manual polar-to-Cartesian transform code; more work than FuncAnimation |

### UWB Modules (for context — existing hardware is fixed)

| Module | MCU inside | Interface | PDOA | Notes |
|--------|-----------|-----------|------|-------|
| **Ai-Thinker BU04** *(in use)* | DW3000 + STM32F103 | AT commands / UART | Yes (dual antenna) | AT interface hides DW3000 registers; no direct SPI. Good enough for bench test. |
| Qorvo DWM3001C | DW3110 + nRF52833 | Full SDK (BLE + UWB) | Yes (single antenna, no PDoA) | FiRa certified, direct register access, expensive; used in industrial RTLS |
| Qorvo DWM3001CDK | DW3110 + nRF52833 | Full SDK (dev kit) | Yes (single ant.) | Dev kit with debugger; overkill for a 2-node test bench |
| Makerfabs ESP32-UWB | DW1000 (gen 1) + ESP32 | SPI direct | No | DW1000 lacks PDoA; cheaper, but requires full ranging stack in firmware |
| Nooploop LinkTrack P | Proprietary (DW3000 family) | UART / ROS | Yes | More polished AT-like interface; significantly higher cost; ROS integration available |
| NXP SR040/SR150 | NXP Trimension | Proprietary | Yes | Used in iPhones/Samsung; no hobbyist module available |

**Why BU04 is correct for this project:** Cost-effective, PDOA supported, dual-antenna hardware already purchased. The AT interface limitation (no raw CIR, no antenna delay register access, no custom TDMA) is acceptable because the test bench only needs distance + angle — both delivered by the AT output.

### Build System

| Alternative | Verdict |
|-------------|---------|
| ESP-IDF (raw) | No — adds ~4× code complexity for no PDOA benefit at this stage |
| Arduino IDE | No — no multi-environment support, manual dependency management |
| **PlatformIO** *(in use)* | Yes — `anchor1_pdoa`, `tag`, `test_bu04` environments in one ini; scripted upload |

---

## Confidence Levels

| Topic | Confidence | Basis |
|-------|------------|-------|
| pyserial API + threading pattern | HIGH | Official docs + common practice |
| matplotlib FuncAnimation polar | HIGH | Official docs; pattern used in many serial-viz projects |
| ESP32-C3 USB CDC build flags | HIGH | Espressif official arduino-esp32 docs + config in platformio.ini already correct |
| USB CDC DTR-reset behaviour | MEDIUM | Known arduino-esp32 issue; widely reported but board revision dependent |
| DW3000 PDOA angle accuracy ±5° (chip datasheet) | HIGH | Qorvo official DW3110 product page |
| AT+PDOAOFF typical range ±30° | LOW | Inferred from DW3000 application notes and community forum posts; no BU04-specific documented value found; must calibrate empirically |
| AT+RNGOFF typical range ±20 cm | LOW | Inferred from DW3000 APS011; BU04 firmware may use different units — verify against AT指令 V1.0.6 |
| pyqtgraph performance vs matplotlib | MEDIUM | Documentation + known benchmarks from embedded community |

---

## Sources / References

| Source | URL | Used for |
|--------|-----|----------|
| Espressif Arduino-ESP32 USB CDC docs | https://docs.espressif.com/projects/arduino-esp32/en/latest/api/usb_cdc.html | ESP32-C3 USB CDC API and build flag semantics |
| matplotlib Animation API | https://matplotlib.org/stable/api/animation_api.html | FuncAnimation pattern, blit behaviour |
| pyqtgraph docs | https://pyqtgraph.readthedocs.io/en/latest/ | Performance characteristics, Qt dependency |
| pyserial docs | https://pyserial.readthedocs.io/en/latest/ | Serial API, threading module |
| Plotly Python getting started | https://plotly.com/python/getting-started/ | Dash architecture (evaluated and excluded) |
| Qorvo DW3110 product page | https://www.qorvo.com/products/p/DW3110 | PDOA ±5° accuracy claim, channel 5/9 support |
| Qorvo DWM3001C product page | https://www.qorvo.com/products/p/DWM3001C | Alternative module reference |
| BU04 AT指令 V1.0.6 (offline, referenced in main.cpp) | — | AT command set: AT+PDOAOFF, AT+RNGOFF, AT+SETUWBMODE, AT+ADDTAG |
| Workspace `src/main.cpp`, `include/config.h`, `platformio.ini` | — | Verified existing firmware patterns |

---

*Last updated: 2026-05-15*
