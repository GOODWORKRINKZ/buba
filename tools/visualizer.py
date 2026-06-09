#!/usr/bin/env python3
"""
BU04 PDOA Live Visualizer — polar plot + CSV logger.

Connects to BU04 anchor via serial (direct USB or ESP32), parses PDOA
measurements, displays live polar chart (range vs angle), and logs all
data to CSV.

Usage:
    python3 tools/visualizer.py                    # auto-detect anchor
    python3 tools/visualizer.py --port /dev/ttyACM1  # manual port
    python3 tools/visualizer.py --list             # list available ports
    python3 tools/visualizer.py --filter kalman    # Kalman smoothing
    python3 tools/visualizer.py --filter median    # median filter (window 5)
    python3 tools/visualizer.py --calibrate        # collect 100 samples, print offsets

Formats supported:
    - Stock STM32 firmware: JSON with JS length prefix
      JS006A{"TWR":{"D":121,"P":4,"R":0,...}}
    - ESP32 anchor1_pdoa firmware: CSV
      PDOA,addr,seq,range_m,angle_deg,x_m,y_m,ts_ms
"""

import serial
import serial.tools.list_ports
import threading
import json
import csv
import sys
import os
import time
import glob
import argparse
from datetime import datetime
from dataclasses import dataclass
from collections import deque

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ═══════════════════════════════════════════════════════════════
#  Data structures
# ═══════════════════════════════════════════════════════════════

@dataclass
class Measurement:
    """Unified internal format for PDOA measurements."""
    timestamp: float          # Python time.time()
    range_cm: float           # distance in cm
    angle_deg: float          # PDOA angle in degrees
    seq: int = 0              # sequence number (R field in JSON)
    raw_json: str = ""        # original line from BU04


# ═══════════════════════════════════════════════════════════════
#  Filters — Kalman, Median, Outlier rejection
# ═══════════════════════════════════════════════════════════════

class KalmanFilter1D:
    """
    Simple 1D Kalman filter for range or angle smoothing.

    State: [value, velocity]
    Measurement: value only

    Tuned for BU04 PDOA at ~4 Hz update rate.
    Default R (measurement noise) matches typical PDOA variance.
    """

    def __init__(self, R: float = 25.0, Q: float = 0.1):
        """
        Args:
            R: Measurement noise covariance (higher = trust measurements less)
            Q: Process noise covariance (higher = trust model less)
        """
        self.x = np.zeros(2)       # state: [value, velocity]
        self.P = np.eye(2) * 100   # initial uncertainty (high = fast convergence)
        self.F = np.array([[1, 1], [0, 1]])        # state transition (constant velocity)
        self.H = np.array([[1, 0]])                 # measurement function (position only)
        self.R = np.array([[R]])                    # measurement noise
        self.Q = np.array([[Q/4, Q/2], [Q/2, Q]])  # process noise
        self.initialized = False

    def update(self, z: float, dt: float = 0.25) -> float:
        """
        Update filter with new measurement.

        Args:
            z: New measurement value
            dt: Time since last update (seconds)

        Returns:
            Filtered value
        """
        if not self.initialized:
            self.x[0] = z
            self.x[1] = 0
            self.initialized = True
            return z

        # Update transition matrix with actual dt
        self.F[0, 1] = dt

        # Predict
        x_pred = self.F @ self.x
        P_pred = self.F @ self.P @ self.F.T + self.Q

        # Update
        y = z - self.H @ x_pred            # innovation
        S = self.H @ P_pred @ self.H.T + self.R  # innovation covariance
        K = P_pred @ self.H.T / S[0, 0]    # Kalman gain

        self.x = x_pred + K.flatten() * y
        self.P = P_pred - np.outer(K, self.H @ P_pred)

        return float(self.x[0])


class MedianFilter:
    """Sliding window median filter."""

    def __init__(self, window: int = 5):
        self.window = window
        self.buffer = deque(maxlen=window)

    def update(self, value: float) -> float:
        self.buffer.append(value)
        if len(self.buffer) < 2:
            return value
        return float(np.median(list(self.buffer)))


class OutlierRejector:
    """Reject measurements that jump too far from the running average."""

    def __init__(self, max_jump_cm: float = 50.0, warmup: int = 5):
        self.max_jump = max_jump_cm
        self.warmup = warmup
        self.last_valid: float | None = None
        self.count = 0

    def check(self, value: float) -> bool:
        """Return True if value passes outlier check."""
        self.count += 1
        if self.count <= self.warmup or self.last_valid is None:
            self.last_valid = value
            return True
        if abs(value - self.last_valid) > self.max_jump:
            return False
        self.last_valid = value
        return True


# ═══════════════════════════════════════════════════════════════
#  Parsers
# ═══════════════════════════════════════════════════════════════

def parse_json_line(line: str) -> Measurement | None:
    """Parse stock STM32 JSON format: JSXXXX{...} or bare {...}"""
    try:
        # Strip JS length prefix if present (e.g., "JS006A{...}" → "{...}")
        cleaned = line.strip()
        if cleaned.startswith('JS'):
            # JS followed by 4 hex digits then JSON
            cleaned = cleaned[6:]

        data = json.loads(cleaned)

        # Handle both {"TWR": {...}} and bare {...} formats
        twr = data.get('TWR', data)

        return Measurement(
            timestamp=time.time(),
            range_cm=float(twr.get('D', 0)),
            angle_deg=float(twr.get('P', 0)),
            seq=int(twr.get('R', 0)),
            raw_json=line.strip()
        )
    except (json.JSONDecodeError, KeyError, ValueError, TypeError):
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
            range_cm=range_m * 100.0,   # m → cm
            angle_deg=float(parts[4]),
            seq=int(parts[2]) if len(parts) > 2 else 0,
            raw_json=line.strip()
        )
    except (ValueError, IndexError):
        return None


def detect_format(line: str) -> str:
    """Return 'json', 'csv', or 'unknown' based on first meaningful line."""
    line = line.strip()
    if not line:
        return 'unknown'
    if line.startswith('{') or line.startswith('JS'):
        return 'json'
    if line.startswith('PDOA,'):
        return 'csv'
    return 'unknown'


# ═══════════════════════════════════════════════════════════════
#  Serial reader thread
# ═══════════════════════════════════════════════════════════════

def reader_thread(ser: serial.Serial, data_deque: deque, lock: threading.Lock,
                  fmt: str, stop_event: threading.Event, stats: dict,
                  filter_mode: str = 'kalman'):
    """
    Daemon thread: read serial port, parse measurements, apply filter, push to deque.

    Args:
        ser: Open serial port
        data_deque: Thread-safe deque for filtered measurements
        lock: threading.Lock for deque access
        fmt: 'json' or 'csv'
        stop_event: Set to stop the thread
        stats: Shared dict with keys: received, skipped, errors,
               last_packet_time, running, raw_range, raw_angle, filt_range, filt_angle
        filter_mode: 'kalman', 'median', or 'none'
    """
    parse_fn = parse_json_line if fmt == 'json' else parse_csv_line

    # Initialize filters
    range_filter = KalmanFilter1D(R=25.0, Q=0.1)
    angle_filter = KalmanFilter1D(R=100.0, Q=0.5)
    median_range = MedianFilter(window=5)
    median_angle = MedianFilter(window=5)
    outlier = OutlierRejector(max_jump_cm=50.0)

    last_dt = 0.25  # assumed ~4 Hz

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
                        stats['received'] += 1
                        now = time.time()
                        dt = now - stats.get('last_raw_time', now)
                        if 0.01 < dt < 2.0:
                            last_dt = dt
                        stats['last_raw_time'] = now
                        stats['last_packet_time'] = now

                        # Store raw values for calibration/stats
                        stats['raw_range'] = m.range_cm
                        stats['raw_angle'] = m.angle_deg

                        # Outlier rejection on range only
                        if not outlier.check(m.range_cm):
                            stats['skipped'] += 1
                            continue

                        # Apply filter
                        if filter_mode == 'kalman':
                            m.range_cm = range_filter.update(m.range_cm, last_dt)
                            m.angle_deg = angle_filter.update(m.angle_deg, last_dt)
                        elif filter_mode == 'median':
                            m.range_cm = median_range.update(m.range_cm)
                            m.angle_deg = median_angle.update(m.angle_deg)
                        # 'none': no filtering

                        stats['filt_range'] = m.range_cm
                        stats['filt_angle'] = m.angle_deg

                        with lock:
                            data_deque.append(m)
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


# ═══════════════════════════════════════════════════════════════
#  Port discovery
# ═══════════════════════════════════════════════════════════════

def list_ports():
    """Print all available serial ports with VID:PID info."""
    print("Available serial ports:")
    for port in serial.tools.list_ports.comports():
        vid_pid = ""
        if port.vid and port.pid:
            vid_pid = f" [{port.vid:04x}:{port.pid:04x}]"
        print(f"  {port.device} — {port.description}{vid_pid}")


def find_anchor_port(verbose: bool = True) -> str | None:
    """
    Scan for BU04 PDOA anchor among USB CDC devices.

    Priority:
    1. STM32 Virtual COM Port devices (VID 0483, PID 5740)
    2. Fallback: any /dev/ttyACM* device

    Identifies anchor by sending AT+DECA$ and checking for
    "PDOA Node" in the response.
    """
    candidates = []

    # First: look for STM32 CDC devices by VID:PID
    for port in serial.tools.list_ports.comports():
        if port.vid == 0x0483 and port.pid == 0x5740:
            candidates.append(port.device)

    # Fallback: any /dev/ttyACM* not already listed
    for dev in sorted(glob.glob('/dev/ttyACM*')):
        if dev not in candidates:
            candidates.append(dev)

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
                # Show brief response for debugging
                short = resp.strip()[:60].replace('\n', ' ')
                print(f"  {port} → {short if short else '(no response)'}")

        except (OSError, serial.SerialException):
            if verbose:
                print(f"  {port} → cannot open")

    return None


# ═══════════════════════════════════════════════════════════════
#  Polar plot
# ═══════════════════════════════════════════════════════════════

def setup_polar_plot():
    """
    Create polar plot figure with scatter artist, trail, tolerance rings,
    and LOST indicator text.

    Returns:
        fig, ax, scatter, trail, lost_text
    """
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    fig.canvas.manager.set_window_title('BU04 PDOA Visualizer')

    # Configure polar axes
    ax.set_theta_zero_location('N')    # 0° = north (forward from anchor)
    ax.set_theta_direction(-1)         # clockwise
    ax.set_thetamin(-90)
    ax.set_thetamax(90)
    ax.set_rmax(3.5)                   # max 3.5m range
    ax.set_rlabel_position(135)
    ax.set_title('BU04 PDOA — Live Range & Angle', pad=20, fontsize=10)

    # Initial scatter (current position — big red dot)
    scatter = ax.scatter([], [], s=80, c='red', zorder=5, label='Current')

    # Trail scatter (fading history)
    trail = ax.scatter([], [], s=20, c='blue', alpha=0.3, zorder=4, label='Trail')

    # Tolerance rings at 1m, 2m, 3m with ±10cm shaded bands
    ring_styles = [
        (1.0, 'green',  '1 m'),
        (2.0, 'blue',   '2 m'),
        (3.0, 'orange', '3 m'),
    ]
    theta = np.linspace(-np.pi / 2, np.pi / 2, 100)
    for r, color, label in ring_styles:
        ax.plot(theta, [r] * 100, '--', color=color, alpha=0.5, linewidth=1)
        ax.fill_between(theta, r - 0.1, r + 0.1, color=color, alpha=0.08)
        ax.text(np.pi / 2.2, r, label, fontsize=7, color=color, alpha=0.7)

    # LOST indicator (hidden initially, shown when no data >1 sec)
    lost_text = ax.text(
        0, 0, 'LOST',
        fontsize=24, color='red', alpha=0,
        ha='center', va='center', weight='bold', zorder=10
    )

    ax.legend(loc='upper right', fontsize=7)

    return fig, ax, scatter, trail, lost_text


def update_plot(frame, data_deque, lock, scatter, trail, lost_text, stats):
    """
    FuncAnimation callback — updates scatter + trail + LOST from shared deque.

    Called at ~10 Hz by matplotlib's animation timer.
    """
    with lock:
        points = list(data_deque) if data_deque else []

    if not points:
        # No data at all — check if we've lost signal
        if time.time() - stats['last_packet_time'] > 1.0:
            lost_text.set_alpha(1.0)
        return [scatter, trail, lost_text]

    lost_text.set_alpha(0)  # data flowing — hide LOST

    # Convert to polar coordinates
    angles = [np.radians(m.angle_deg) for m in points]
    ranges = [m.range_cm / 100.0 for m in points]

    # Current position = last point (big red dot)
    scatter.set_offsets(np.c_[angles[-1:], ranges[-1:]])
    scatter.set_alpha(1.0)

    # Trail = last 200 points with fading alpha
    trail_n = min(len(angles), 200)
    trail_angles = angles[-trail_n:]
    trail_ranges = ranges[-trail_n:]
    trail.set_offsets(np.c_[trail_angles, trail_ranges])

    # Update title with statistics
    stat_parts = [
        f"Rx: {stats['received']}",
        f"Skip: {stats['skipped']}",
    ]
    last = points[-1]
    raw_r = stats.get('raw_range', last.range_cm)
    raw_a = stats.get('raw_angle', last.angle_deg)
    stat_parts.append(f"Raw: D={raw_r:.0f}cm P={raw_a:.0f}°")
    stat_parts.append(f"Flt: D={last.range_cm:.0f}cm P={last.angle_deg:.0f}°")
    scatter.axes.set_title(
        'BU04 PDOA — ' + ' | '.join(stat_parts),
        pad=20, fontsize=9
    )

    return [scatter, trail, lost_text]


# ═══════════════════════════════════════════════════════════════
#  CSV logger
# ═══════════════════════════════════════════════════════════════

CSV_FIELDS = ['timestamp', 'range_cm', 'angle_deg', 'seq', 'raw_json']


def csv_logger_thread(data_deque: deque, lock: threading.Lock,
                      stop_event: threading.Event, csv_path: str):
    """
    Write measurements to CSV file. Flushes every 10 rows.

    Writes only new measurements (by seq number) to avoid duplicates
    in case the deque accumulates faster than we write.
    """
    os.makedirs('logs', exist_ok=True)

    with open(csv_path, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        f.flush()

        last_written_seq = -1
        rows_since_flush = 0

        while not stop_event.is_set():
            with lock:
                items = list(data_deque) if data_deque else []

            new_items = [m for m in items if m.seq != last_written_seq]

            for m in new_items:
                writer.writerow({
                    'timestamp': m.timestamp,
                    'range_cm': m.range_cm,
                    'angle_deg': m.angle_deg,
                    'seq': m.seq,
                    'raw_json': m.raw_json,
                })
                last_written_seq = m.seq
                rows_since_flush += 1

            if rows_since_flush >= 10:
                f.flush()
                rows_since_flush = 0

            time.sleep(0.1)


# ═══════════════════════════════════════════════════════════════
#  CLI
# ═══════════════════════════════════════════════════════════════

def parse_args():
    """Parse command-line arguments."""
    ap = argparse.ArgumentParser(
        description='BU04 PDOA Live Visualizer — polar plot + CSV logger'
    )
    ap.add_argument(
        '--port', '-p', default=None,
        help='Serial port (auto-detect if not specified)'
    )
    ap.add_argument(
        '--baud', '-b', type=int, default=115200,
        help='Baud rate (default: 115200)'
    )
    ap.add_argument(
        '--list', '-l', action='store_true',
        help='List available ports and exit'
    )
    ap.add_argument(
        '--filter', '-f', choices=['kalman', 'median', 'none'],
        default='kalman',
        help='Filter mode: kalman (default), median (window 5), none'
    )
    ap.add_argument(
        '--calibrate', '-c', action='store_true',
        help='Calibration mode: collect 100 samples at stationary position, print recommended PDOAOFF/RNGOFF'
    )
    return ap.parse_args()


# ═══════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════

def main():
    args = parse_args()

    # --list mode: show ports and exit
    if args.list:
        list_ports()
        return

    # Find anchor port
    port = args.port or find_anchor_port()
    if not port:
        print("ERROR: No PDOA anchor found.")
        print("Specify port manually: python3 tools/visualizer.py --port /dev/ttyACM1")
        print("Or list ports: python3 tools/visualizer.py --list")
        sys.exit(1)

    print(f"Anchor port: {port} @ {args.baud} baud")

    # Open serial with dtr=False (prevents ESP32-C3 reset on connect)
    try:
        ser = serial.Serial(port, args.baud, timeout=1)
        ser.dtr = False
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {port}: {e}")
        sys.exit(1)

    time.sleep(0.5)
    ser.reset_input_buffer()

    # Detect data format — read until we get a meaningful line
    print("Detecting data format...")
    fmt = 'json'  # default assumption
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

    fmt_name = 'stock STM32 JSON' if fmt == 'json' else 'ESP32 CSV'
    print(f"Format: {fmt.upper()} ({fmt_name})")

    # Shared state between threads
    data_deque = deque(maxlen=500)
    lock = threading.Lock()
    stop_event = threading.Event()
    stats = {
        'received': 0,
        'skipped': 0,
        'errors': 0,
        'last_packet_time': time.time(),
        'last_raw_time': time.time(),
        'raw_range': 0.0,
        'raw_angle': 0.0,
        'filt_range': 0.0,
        'filt_angle': 0.0,
        'running': True,
    }

    # CSV log path
    os.makedirs('logs', exist_ok=True)
    csv_path = os.path.join('logs', datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv')

    # ── Calibration mode ──────────────────────────────────
    if args.calibrate:
        print()
        print("=" * 55)
        print("  BU04 PDOA CALIBRATION MODE")
        print("=" * 55)
        print()
        print("Place the tag at a KNOWN position and keep it STILL.")
        print("Example: 1.00 m directly in front of anchor (0°)")
        print()
        print("Collecting 100 samples...")
        print()

        # Start reader without plot
        reader = threading.Thread(
            target=reader_thread,
            args=(ser, data_deque, lock, fmt, stop_event, stats, 'none'),
            daemon=True, name='serial-reader'
        )
        reader.start()

        # Collect 100 raw samples
        samples_r = []
        samples_a = []
        while len(samples_r) < 100:
            with lock:
                items = list(data_deque)
            for m in items:
                if len(samples_r) >= 100:
                    break
                samples_r.append(m.range_cm)
                samples_a.append(m.angle_deg)
                # Progress
                n = len(samples_r)
                if n % 10 == 0:
                    print(f"\r  [{n}/100] D={m.range_cm:.0f}cm P={m.angle_deg:.0f}°", end='', flush=True)
            time.sleep(0.05)

        stop_event.set()
        ser.close()
        print()

        # Compute statistics
        import statistics
        mean_r = statistics.mean(samples_r)
        mean_a = statistics.mean(samples_a)
        stdev_r = statistics.stdev(samples_r)
        stdev_a = statistics.stdev(samples_a)

        print()
        print("=" * 55)
        print("  CALIBRATION RESULTS")
        print("=" * 55)
        print(f"  Samples:       {len(samples_r)}")
        print(f"  Range:         mean={mean_r:.1f} cm  stdev={stdev_r:.1f} cm")
        print(f"  Angle:         mean={mean_a:.1f} °    stdev={stdev_a:.1f} °")
        print()
        print("  Now measure the TRUE distance and angle with a ruler.")
        print("  Then apply corrections:")
        print()
        print(f"  AT+RNGOFF={-int(mean_r - 100):d}     # if true distance = 100 cm")
        print(f"  AT+PDOAOFF={-int(mean_a):d}          # if true angle = 0°")
        print()
        print("  Example for true distance = 100 cm, true angle = 0°:")
        rngoff = 100 - int(mean_r)
        pdoaoff = -int(mean_a)
        print(f"    AT+RNGOFF={rngoff}")
        print(f"    AT+PDOAOFF={pdoaoff}")
        print(f"    AT+SAVE")
        print()
        print("  Or set values in include/config.h:")
        print(f"    #define RANGE_OFFSET_CM  {rngoff}")
        print(f"    #define PDOA_OFFSET_DEG  {pdoaoff}")
        print()
        return

    # ── Normal visualization mode ─────────────────────────
    # Start reader thread
    reader = threading.Thread(
        target=reader_thread,
        args=(ser, data_deque, lock, fmt, stop_event, stats, args.filter),
        daemon=True, name='serial-reader'
    )
    reader.start()

    # Start CSV logger thread
    csv_thread = threading.Thread(
        target=csv_logger_thread,
        args=(data_deque, lock, stop_event, csv_path),
        daemon=True,
        name='csv-logger'
    )
    csv_thread.start()

    # Setup polar plot
    print("Starting polar plot (close window or Ctrl+C to exit)...")
    fig, ax, scatter, trail, lost_text = setup_polar_plot()

    # FuncAnimation at 10 Hz (100 ms interval)
    ani = FuncAnimation(
        fig, update_plot,
        fargs=(data_deque, lock, scatter, trail, lost_text, stats),
        interval=100,
        blit=False,
        cache_frame_data=False,
    )

    def on_close(event=None):
        """Graceful shutdown on window close or Ctrl+C."""
        print("\nShutting down...")
        stop_event.set()
        ser.close()
        print(f"CSV log: {csv_path}")
        print(
            f"Stats: {stats['received']} received, "
            f"{stats['skipped']} skipped, "
            f"{stats['errors']} errors"
        )

    fig.canvas.mpl_connect('close_event', on_close)

    try:
        plt.show()
    except KeyboardInterrupt:
        on_close()


if __name__ == '__main__':
    main()
