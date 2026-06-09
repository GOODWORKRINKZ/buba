---
phase: 02-visualizer
plan: 01
type: execute
wave: 1
depends_on: []
files_modified:
  - tools/visualizer.py
autonomous: true
requirements:
  - VIZ-01
  - VIZ-02
  - VIZ-03
  - VIZ-04
  - VIZ-05

must_haves:
  truths:
    - "python tools/visualizer.py — polar plot window opens, dot moves with tag position"
    - "Auto-detects STM32 port by VID 0483 PID 5740, identifies anchor via AT+DECA$"
    - "Parses both JSON ({\"TWR\":...}) from stock STM32 and CSV (PDOA,...) from ESP32 firmware"
    - "Polar plot shows scatter dot + fading trail(200pts) + dashed rings at 1/2/3m ±10cm"
    - "CSV file logs/YYYYMMDD_HHMMSS.csv created with full TWR fields + raw_json"
    - "LOST overlay appears when no data for >1s, disappears on next packet"
    - "Ctrl+C in plot window → graceful shutdown, CSV properly closed"
  artifacts:
    - path: "tools/visualizer.py"
      provides: "Interactive PDOA polar visualizer with CSV logging"
      exports: ["main"]
      contains: ["Measurement", "find_anchor_port", "parse_json_line", "parse_csv_line"]
  key_links:
    - from: "find_anchor_port()"
      to: "serial.Serial()"
      via: "scan /dev/ttyACM*, check VID:PID, send AT+DECA$"
    - from: "reader_thread()"
      to: "collections.deque"
      via: "threading.Lock"
    - from: "FuncAnimation callback"
      to: "scatter.set_offsets()"
      via: "deque copy under lock"
---

<objective>
Build `tools/visualizer.py` — a Python script that opens the BU04 anchor serial port,
parses PDOA measurements, displays a live polar plot (range vs angle), and logs all
data to CSV. Works with both stock STM32 firmware (JSON) and ESP32 firmware (CSV).

Single file, zero external config — just `python3 tools/visualizer.py`.
</objective>

<execution_context>
@~/.copilot/get-shit-done/workflows/execute-plan.md
</execution_context>

<context>
@/home/ros2/buba/scripts/dual_monitor.py
@/home/ros2/buba/scripts/bu04_terminal.py
@/home/ros2/buba/.planning/phases/02-visualizer/02-CONTEXT.md
@/home/ros2/buba/.planning/phases/01-firmware-fixes/01-SERIAL-LOG.txt

<interfaces>
### Existing code patterns (from scripts/)

**dual_monitor.py** — threading + pyserial pattern:
```python
ser = serial.Serial(port, baud, timeout=1)
# Daemon reader thread
threading.Thread(target=read_serial, args=(...), daemon=True)
```

**bu04_terminal.py** — AT command interaction + ANSI colors:
```python
ser.write(b'AT+DECA$\r\n')
resp = ser.read(ser.in_waiting or 1).decode('utf-8', errors='replace')
# Port identification: direct USB has no '> ' echo, ESP32 bridge has '> ' echo
```

### JSON format (from 01-SERIAL-LOG.txt — actual BU04 output)
```json
JS006A{"TWR": {
  "a16": "657B",      // short address (hex)
  "R": 0,              // sequence number
  "T": 774068,         // DW3000 timestamp
  "D": 121,            // distance (cm)
  "P": 4,              // PDOA angle (degrees, signed)
  "Xcm": 15,           // X coordinate (cm)
  "Ycm": 230,          // Y coordinate (cm)
  "Angle": 4           // angle (degrees)
}}
```
Note: Lines have a `JS` prefix (4 hex chars = payload length) before the JSON.
The parser must strip `JSXXXX{...}` → `{...}`.

### CSV format (from ESP32 anchor1_pdoa firmware)
```
PDOA,<addr_hex>,<seq>,<range_m>,<angle_deg>,<x_m>,<y_m>,<ts_ms>
```

### Port identification
- STM32 direct USB: no `>` echo, responds to AT+DECA$ with `{"Info":{"Device":"PDOA Node"...}}`
- ESP32 at_bridge: `> AT+...` echo prefix, passes AT commands to BU04 behind it
- VID:PID for STM32 CDC: `0483:5740`
</interfaces>

### Dependencies
```
pip install pyserial matplotlib numpy
```
</context>

---

## Threat Model

| Threat ID | Category | Component | Disposition | Mitigation |
|-----------|----------|-----------|-------------|------------|
| T-02-01 | Denial of Service | Serial port held by another process | mitigate | Catch `SerialException`, print "Port {name} busy", exit 1 |
| T-02-02 | Data Corruption | Malformed JSON from BU04 | mitigate | `json.loads()` in try/except; skip bad lines, increment `skipped_count` |
| T-02-03 | Resource Exhaustion | CSV file grows unbounded | accept | Test bench sessions < 1 hour; ~10 MB/hr at 4 Hz full JSON |
| T-02-04 | UI Freeze | matplotlib FuncAnimation blocked by serial read | mitigate | Reader in daemon thread with `threading.Lock`; deque copy under lock in animation callback |
| T-02-05 | Port Confusion | Multiple STM32 CDC devices, wrong one selected | mitigate | `AT+DECA$` verification — only port responding `"Device":"PDOA Node"` is selected |

---

## Wave Structure

| Wave | Plans | Description | Autonomous |
|------|-------|-------------|------------|
| 1 | Plan 2.1, 2.2, 2.3, 2.4 | All code — single file, independent functions, same file | yes |
| 2 | Plan 2.5 | Hardware-in-loop burn-in — requires live anchor + tag | checkpoint |

All plans write to `tools/visualizer.py`. Execute sequentially (2.1 → 2.2 → 2.3 → 2.4) because each builds on the previous. Plan 2.5 runs after all code is done and requires physical hardware.

---

<tasks>

<!-- ═══════════════════════════════════════════════════════════
     PLAN 2.1 — Core: Measurement, parser, serial reader thread
     ═══════════════════════════════════════════════════════════ -->

<task type="auto">
  <name>Task 2.1a — Measurement dataclass + parser functions</name>
  <files>tools/visualizer.py</files>
  <action>
Create `tools/visualizer.py` with imports and core data structures:

```python
#!/usr/bin/env python3
"""
BU04 PDOA Live Visualizer — polar plot + CSV logger.
Usage: python3 tools/visualizer.py [--port /dev/ttyACM1]
"""

import serial
import serial.tools.list_ports
import threading
import json
import csv
import sys
import os
import time
import re
from datetime import datetime
from dataclasses import dataclass, field
from collections import deque

import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # non-interactive backend for thread safety
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ── Data structures ────────────────────────────────────────

@dataclass
class Measurement:
    """Unified internal format for PDOA measurements."""
    timestamp: float          # Python time.time()
    range_cm: float           # distance in cm
    angle_deg: float          # PDOA angle in degrees
    seq: int = 0              # sequence number
    raw_json: str = ""        # original line from BU04

# ── Parsers ────────────────────────────────────────────────

def parse_json_line(line: str) -> Measurement | None:
    """Parse stock STM32 JSON format: JSXXXX{...} or {...}"""
    try:
        # Strip JS length prefix if present
        if line.startswith('JS'):
            line = line[6:]  # skip 'JSXXXX'
        data = json.loads(line)
        twr = data.get('TWR', data)
        return Measurement(
            timestamp=time.time(),
            range_cm=float(twr.get('D', 0)),
            angle_deg=float(twr.get('P', 0)),
            seq=int(twr.get('R', 0)),
            raw_json=line
        )
    except (json.JSONDecodeError, KeyError, ValueError):
        return None

def parse_csv_line(line: str) -> Measurement | None:
    """Parse ESP32 CSV format: PDOA,addr,seq,range_m,angle_deg,x_m,y_m,ts_ms"""
    try:
        parts = line.strip().split(',')
        if len(parts) < 5 or parts[0] != 'PDOA':
            return None
        range_m = float(parts[3])
        return Measurement(
            timestamp=time.time(),
            range_cm=range_m * 100.0,  # m → cm
            angle_deg=float(parts[4]),
            seq=int(parts[2]) if len(parts) > 2 else 0,
            raw_json=line
        )
    except (ValueError, IndexError):
        return None

def detect_format(line: str) -> str:
    """Return 'json' or 'csv' based on first meaningful line."""
    line = line.strip()
    if not line:
        return 'unknown'
    if line.startswith('{') or line.startswith('JS'):
        return 'json'
    if line.startswith('PDOA,'):
        return 'csv'
    return 'unknown'
```
  </action>
  <verify>
    <automated>cd /home/ros2/buba && python3 -c "from tools.visualizer import Measurement, parse_json_line, parse_csv_line, detect_format; print('imports OK')"</automated>
  </verify>
  <done>python3 imports succeed with no errors</done>
</task>

<task type="auto">
  <name>Task 2.1b — Serial reader daemon thread</name>
  <files>tools/visualizer.py</files>
  <action>
Add serial reader thread to `tools/visualizer.py`. Append after the parser functions:

```python
# ── Serial reader thread ───────────────────────────────────

def reader_thread(ser: serial.Serial, data_deque: deque, lock: threading.Lock,
                  fmt: str, stop_event: threading.Event, stats: dict):
    """Daemon thread: read serial, parse measurements, push to deque."""
    parse_fn = parse_json_line if fmt == 'json' else parse_csv_line
    
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                raw = ser.read(ser.in_waiting)
                text = raw.decode('utf-8', errors='replace')
                for line in text.split('\n'):
                    line = line.strip()
                    if not line:
                        continue
                    m = parse_fn(line)
                    if m:
                        with lock:
                            data_deque.append(m)
                        stats['received'] += 1
                        stats['last_packet_time'] = time.time()
                    elif any(c in line for c in '{}[]'):
                        stats['skipped'] += 1
            else:
                time.sleep(0.02)
        except (OSError, serial.SerialException):
            stop_event.set()
            break
        except Exception:
            stats['errors'] += 1
    
    stats['running'] = False
```
  </action>
  <verify>
    <automated>cd /home/ros2/buba && python3 -c "from tools.visualizer import reader_thread; print('reader_thread OK')"</automated>
  </verify>
  <done>python3 import succeeds</done>
</task>

<task type="auto">
  <name>Task 2.1c — Port auto-discovery + CLI argument parsing</name>
  <files>tools/visualizer.py</files>
  <action>
Add port discovery and CLI parsing. Append after reader_thread:

```python
# ── Port discovery ─────────────────────────────────────────

def find_anchor_port(verbose: bool = True) -> str | None:
    """Scan /dev/ttyACM* for STM32 CDC devices, identify anchor via AT+DECA$."""
    import serial.tools.list_ports
    
    candidates = []
    for port in serial.tools.list_ports.comports():
        if port.vid == 0x0483 and port.pid == 0x5740:
            candidates.append(port.device)
    
    if not candidates:
        # Fallback: check /dev/ttyACM*
        import glob
        candidates = sorted(glob.glob('/dev/ttyACM*'))
    
    if not candidates:
        return None
    
    if verbose:
        print(f"Scanning {len(candidates)} port(s) for PDOA anchor...")
    
    for port in candidates:
        try:
            ser = serial.Serial(port, 115200, timeout=1.5)
            time.sleep(0.3)
            ser.reset_input_buffer()
            ser.write(b'AT+DECA$\r\n')
            time.sleep(0.8)
            resp = ser.read(ser.in_waiting or 1).decode('utf-8', errors='replace')
            ser.close()
            
            if 'PDOA Node' in resp or '"Device":"PDOA' in resp:
                if verbose:
                    print(f"  {port} → PDOA Node ✓")
                return port
            elif verbose:
                print(f"  {port} → not a PDOA anchor")
        except (OSError, serial.SerialException):
            if verbose:
                print(f"  {port} → cannot open")
    
    return None

def parse_args():
    """Parse command-line arguments. Returns (port, baud)."""
    import argparse
    ap = argparse.ArgumentParser(
        description='BU04 PDOA Live Visualizer — polar plot + CSV logger')
    ap.add_argument('--port', '-p', default=None,
                    help='Serial port (auto-detect if not specified)')
    ap.add_argument('--baud', '-b', type=int, default=115200,
                    help='Baud rate (default: 115200)')
    ap.add_argument('--list', '-l', action='store_true',
                    help='List available ports and exit')
    return ap.parse_args()
```
  </action>
  <verify>
    <automated>cd /home/ros2/buba && python3 -c "from tools.visualizer import find_anchor_port, parse_args; print('discovery OK')"</automated>
  </verify>
  <done>python3 import succeeds</done>
</task>

<!-- ═══════════════════════════════════════════════════════════
     PLAN 2.2 — Polar plot: scatter + trail + rings + LOST
     ═══════════════════════════════════════════════════════════ -->

<task type="auto">
  <name>Task 2.2 — Polar plot with scatter, trail, tolerance rings, LOST indicator</name>
  <files>tools/visualizer.py</files>
  <action>
Add polar plot setup and animation callback. Append after port discovery:

```python
# ── Polar plot ─────────────────────────────────────────────

def setup_polar_plot():
    """Create polar plot with scatter artist, trail, tolerance rings, LOST text."""
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    fig.canvas.manager.set_window_title('BU04 PDOA Visualizer')
    
    # Configure polar axes
    ax.set_theta_zero_location('N')   # 0° = north (forward)
    ax.set_theta_direction(-1)        # clockwise
    ax.set_thetamin(-90)
    ax.set_thetamax(90)
    ax.set_rmax(3.5)                  # max 3.5m range
    ax.set_rlabel_position(135)
    ax.set_xlabel('Angle (degrees)')
    ax.set_title('BU04 PDOA — Live Range & Angle', pad=20)
    
    # Initial scatter (empty)
    scatter = ax.scatter([], [], s=80, c='red', zorder=5, label='Current')
    
    # Trail scatter (fading)
    trail = ax.scatter([], [], s=20, c='blue', alpha=0.3, zorder=4, label='Trail')
    
    # Tolerance rings at 1m, 2m, 3m
    ring_styles = [(1.0, 'green', '1 m'), (2.0, 'blue', '2 m'), (3.0, 'orange', '3 m')]
    for r, color, label in ring_styles:
        theta = np.linspace(-np.pi/2, np.pi/2, 100)
        ax.plot(theta, [r]*100, '--', color=color, alpha=0.5, linewidth=1)
        # Shaded tolerance band ±10cm
        ax.fill_between(theta, r - 0.1, r + 0.1, color=color, alpha=0.08)
        ax.text(np.pi/2.2, r, label, fontsize=7, color=color, alpha=0.7)
    
    # LOST indicator (hidden initially)
    lost_text = ax.text(0, 0, 'LOST', fontsize=24, color='red', alpha=0,
                        ha='center', va='center', weight='bold', zorder=10)
    
    ax.legend(loc='upper right', fontsize=7)
    
    return fig, ax, scatter, trail, lost_text

def update_plot(frame, data_deque, lock, scatter, trail, lost_text, stats):
    """FuncAnimation callback — updates scatter + trail + LOST from deque."""
    with lock:
        if not data_deque:
            points = []
        else:
            points = list(data_deque)
    
    if not points:
        # No data — check LOST
        if time.time() - stats['last_packet_time'] > 1.0:
            lost_text.set_alpha(1.0)
        return [scatter, trail, lost_text]
    
    lost_text.set_alpha(0)  # data flowing — hide LOST
    
    # Convert to polar coordinates: theta = angle in radians, r = range in meters
    angles = [np.radians(m.angle_deg) for m in points]
    ranges = [m.range_cm / 100.0 for m in points]
    
    # Current position = last point (big red dot)
    if angles:
        scatter.set_offsets(np.c_[angles[-1:], ranges[-1:]])
        scatter.set_alpha(1.0)
    
    # Trail = last 200 points with fading alpha
    trail_n = min(len(angles), 200)
    if trail_n > 0:
        trail_angles = angles[-trail_n:]
        trail_ranges = ranges[-trail_n:]
        trail.set_offsets(np.c_[trail_angles, trail_ranges])
        # Fading alpha: older → smaller
        alphas = np.linspace(0.05, 0.5, trail_n)
        trail.set_alpha(alphas[-1] if trail_n == 1 else 0.5)
    
    # Update title with stats
    stats_str = f"Rx: {stats['received']} | Skip: {stats['skipped']} | Err: {stats['errors']}"
    if points:
        last = points[-1]
        stats_str += f" | D={last.range_cm:.0f}cm P={last.angle_deg:.0f}°"
    scatter.axes.set_title(f'BU04 PDOA — {stats_str}', pad=20, fontsize=9)
    
    return [scatter, trail, lost_text]
```
  </action>
  <verify>
    <automated>cd /home/ros2/buba && python3 -c "from tools.visualizer import setup_polar_plot, update_plot; print('plot OK')"</automated>
  </verify>
  <done>python3 import succeeds (matplotlib may open a window briefly)</done>
</task>

<!-- ═══════════════════════════════════════════════════════════
     PLAN 2.3 — CSV session logger
     ═══════════════════════════════════════════════════════════ -->

<task type="auto">
  <name>Task 2.3 — CSV session logger</name>
  <files>tools/visualizer.py</files>
  <action>
Add CSV logging. Append after plot functions:

```python
# ── CSV logger ─────────────────────────────────────────────

def open_csv_log(logs_dir: str = 'logs') -> tuple:
    """Create logs/ dir, open CSV file with timestamped name. Returns (writer, file, path)."""
    os.makedirs(logs_dir, exist_ok=True)
    filename = datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv'
    filepath = os.path.join(logs_dir, filename)
    
    f = open(filepath, 'w', newline='')
    writer = csv.DictWriter(f, fieldnames=[
        'timestamp', 'range_cm', 'angle_deg', 'seq', 'raw_json'
    ])
    writer.writeheader()
    f.flush()
    
    return writer, f, filepath

def csv_logger_thread(data_deque: deque, lock: threading.Lock,
                      stop_event: threading.Event, csv_filepath: str):
    """Write measurements to CSV. Flushes every 10 rows."""
    os.makedirs('logs', exist_ok=True)
    
    with open(csv_filepath, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[
            'timestamp', 'range_cm', 'angle_deg', 'seq', 'raw_json'
        ])
        
        last_written_seq = -1
        rows_since_flush = 0
        
        while not stop_event.is_set():
            with lock:
                if data_deque:
                    # Get new measurements since last write
                    new_items = [m for m in data_deque if m.seq > last_written_seq]
                else:
                    new_items = []
            
            for m in new_items:
                writer.writerow({
                    'timestamp': m.timestamp,
                    'range_cm': m.range_cm,
                    'angle_deg': m.angle_deg,
                    'seq': m.seq,
                    'raw_json': m.raw_json
                })
                last_written_seq = m.seq
                rows_since_flush += 1
            
            if rows_since_flush >= 10:
                f.flush()
                rows_since_flush = 0
            
            time.sleep(0.1)
```
  </action>
  <verify>
    <automated>cd /home/ros2/buba && python3 -c "from tools.visualizer import open_csv_log, csv_logger_thread; print('csv OK')"</automated>
  </verify>
  <done>python3 import succeeds</done>
</task>

<!-- ═══════════════════════════════════════════════════════════
     PLAN 2.4 — main(): wire everything together
     ═══════════════════════════════════════════════════════════ -->

<task type="auto">
  <name>Task 2.4 — main() function wiring all components</name>
  <files>tools/visualizer.py</files>
  <action>
Add `main()` function that wires all components together. Append at end of file:

```python
# ── Main ───────────────────────────────────────────────────

def main():
    args = parse_args()
    
    # List ports mode
    if args.list:
        print("Available serial ports:")
        for port in serial.tools.list_ports.comports():
            print(f"  {port.device} — {port.description} "
                  f"[{port.vid:04x}:{port.pid:04x}]" if port.vid else "")
        return
    
    # Find port
    port = args.port or find_anchor_port()
    if not port:
        print("ERROR: No PDOA anchor found.")
        print("Specify port manually: python3 tools/visualizer.py --port /dev/ttyACM1")
        print("Or list ports: python3 tools/visualizer.py --list")
        sys.exit(1)
    
    print(f"Anchor port: {port} @ {args.baud} baud")
    
    # Open serial with dtr=False (prevents ESP32-C3 reset)
    try:
        ser = serial.Serial(port, args.baud, timeout=1)
        ser.dtr = False
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {port}: {e}")
        sys.exit(1)
    
    time.sleep(0.5)
    ser.reset_input_buffer()
    
    # Detect format — read first meaningful line
    print("Detecting data format...")
    fmt = 'json'  # default
    t0 = time.time()
    while time.time() - t0 < 5.0:
        if ser.in_waiting:
            raw = ser.read(ser.in_waiting).decode('utf-8', errors='replace')
            for line in raw.split('\n'):
                line = line.strip()
                if line:
                    detected = detect_format(line)
                    if detected != 'unknown':
                        fmt = detected
                        break
            if fmt != 'json':
                break
        time.sleep(0.1)
    
    print(f"Format: {fmt.upper()} ({'stock STM32' if fmt == 'json' else 'ESP32 firmware'})")
    
    # Setup shared state
    data_deque = deque(maxlen=500)
    lock = threading.Lock()
    stop_event = threading.Event()
    stats = {
        'received': 0,
        'skipped': 0,
        'errors': 0,
        'last_packet_time': time.time(),
        'running': True
    }
    
    # Setup CSV logger
    csv_path = os.path.join('logs', datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv')
    csv_writer, csv_file, _ = open_csv_log()
    os.makedirs('logs', exist_ok=True)
    
    # Start reader thread
    reader = threading.Thread(
        target=reader_thread,
        args=(ser, data_deque, lock, fmt, stop_event, stats),
        daemon=True
    )
    reader.start()
    
    # Start CSV logger thread
    csv_thread = threading.Thread(
        target=csv_logger_thread,
        args=(data_deque, lock, stop_event, csv_path),
        daemon=True
    )
    csv_thread.start()
    
    # Setup plot
    print("Starting polar plot (close window or Ctrl+C to exit)...")
    fig, ax, scatter, trail, lost_text = setup_polar_plot()
    
    # FuncAnimation @ 10 Hz
    ani = FuncAnimation(
        fig, update_plot,
        fargs=(data_deque, lock, scatter, trail, lost_text, stats),
        interval=100,  # ms
        blit=False,
        cache_frame_data=False
    )
    
    def on_close(event):
        """Graceful shutdown on window close."""
        print("\nShutting down...")
        stop_event.set()
        ser.close()
        csv_file.close()
        print(f"CSV log: {csv_path}")
        print(f"Stats: {stats['received']} received, {stats['skipped']} skipped, {stats['errors']} errors")
    
    fig.canvas.mpl_connect('close_event', on_close)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        on_close(None)

if __name__ == '__main__':
    main()
```
  </action>
  <verify>
    <automated>cd /home/ros2/buba && python3 -c "from tools.visualizer import main; print('main OK')"</automated>
  </verify>
  <done>python3 import succeeds, --list mode works without hardware</done>
</task>

<!-- ═══════════════════════════════════════════════════════════
     PLAN 2.5 — Hardware-in-loop burn-in test
     ═══════════════════════════════════════════════════════════ -->

<task type="manual">
  <name>Task 2.5 — Hardware burn-in: 60 sec PDOA stream with visualizer</name>
  <files>tools/visualizer.py</files>
  <action>
With anchor + tag powered on and in range (~1.5m):

1. Run: `python3 tools/visualizer.py`
2. Verify: polar plot window opens, red dot appears within 5 sec
3. Move tag to ~1m at 0° → dot near 1m ring
4. Move tag to ~2m at +45° → dot moves accordingly
5. Move tag out of range or power off → "LOST" appears within 2 sec
6. Power tag back on → "LOST" disappears, dot returns
7. Let run 60 sec → Ctrl+C
8. Verify CSV: `wc -l logs/*.csv` shows ≥ 200 rows
9. Verify CSV content: `head -3 logs/*.csv` shows header + data
  </action>
  <verify>
    <manual>Visual inspection: plot responds to tag movement, CSV log created with ≥200 rows in 60 sec</manual>
  </verify>
  <done>All 9 manual checks pass</done>
</task>

</tasks>
