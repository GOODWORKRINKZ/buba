#!/usr/bin/env python3
"""
BU04 PDOA Live Visualizer — polar plot + CSV logger.

Connects to BU04 anchor via serial (direct USB or ESP32), parses PDOA
measurements, displays live polar chart (range vs angle), and logs all
data to CSV.

Usage:
    python3 tools/visualizer.py                    # auto-detect, 4-view dashboard
    python3 tools/visualizer.py --port /dev/ttyACM1  # manual port
    python3 tools/visualizer.py --list             # list available ports
    python3 tools/visualizer.py --calibrate        # calibration mode

4-View Dashboard:
    Top-left:     Raw Scatter + Trail (unfiltered)
    Top-right:    Kalman Filtered (smooth)
    Bottom-left:  Density Heatmap (hot = more visits)
    Bottom-right: Compare All (Raw + Kalman + Median overlaid)

All views have tolerance rings (1/2/3m ±10cm) and LOST indicator.

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

def reader_thread(ser: serial.Serial, deques: dict, locks: dict,
                  fmt: str, stop_event: threading.Event, stats: dict):
    """
    Daemon thread: read serial, parse, apply all filters, push to 3 deques.

    deques: {'raw': deque, 'kalman': deque, 'median': deque}
    locks:  {'raw': Lock, 'kalman': Lock, 'median': Lock}

    Raw gets unfiltered measurements.
    Kalman gets Kalman-filtered measurements.
    Median gets median-filtered measurements (window=5).
    All three get outlier rejection applied.
    """
    parse_fn = parse_json_line if fmt == 'json' else parse_csv_line

    # Filters
    kf_range = KalmanFilter1D(R=25.0, Q=0.1)
    kf_angle = KalmanFilter1D(R=100.0, Q=0.5)
    med_range = MedianFilter(window=5)
    med_angle = MedianFilter(window=5)
    outlier = OutlierRejector(max_jump_cm=50.0)

    last_dt = 0.25

    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                raw_data = ser.read(ser.in_waiting)
                text = raw_data.decode('utf-8', errors='replace')
                for line in text.split('\n'):
                    line = line.strip()
                    if not line:
                        continue
                    m = parse_fn(line)
                    if not m:
                        if any(c in line for c in '{}[]'):
                            stats['skipped'] += 1
                        continue

                    stats['received'] += 1
                    now = time.time()
                    dt = now - stats.get('last_raw_time', now)
                    if 0.01 < dt < 2.0:
                        last_dt = dt
                    stats['last_raw_time'] = now
                    stats['last_packet_time'] = now

                    raw_r, raw_a = m.range_cm, m.angle_deg

                    # Outlier rejection on raw range
                    if not outlier.check(raw_r):
                        stats['skipped'] += 1
                        continue

                    # ── Raw (no filter) ──
                    m_raw = Measurement(
                        timestamp=now, range_cm=raw_r, angle_deg=raw_a,
                        seq=m.seq, raw_json=m.raw_json)
                    with locks['raw']:
                        deques['raw'].append(m_raw)

                    # ── Kalman ──
                    m_kal = Measurement(
                        timestamp=now,
                        range_cm=kf_range.update(raw_r, last_dt),
                        angle_deg=kf_angle.update(raw_a, last_dt),
                        seq=m.seq, raw_json=m.raw_json)
                    with locks['kalman']:
                        deques['kalman'].append(m_kal)

                    # ── Median ──
                    m_med = Measurement(
                        timestamp=now,
                        range_cm=med_range.update(raw_r),
                        angle_deg=med_angle.update(raw_a),
                        seq=m.seq, raw_json=m.raw_json)
                    with locks['median']:
                        deques['median'].append(m_med)

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
    Create 2×2 polar plot grid:

    ┌─────────────────────┬─────────────────────┐
    │ Raw Scatter + Trail │ Kalman Filtered     │
    │ (red, fading trail) │ (blue, smooth)      │
    ├─────────────────────┼─────────────────────┤
    │ Density Heatmap     │ All Filters Overlay │
    │ (hot = more visits) │ Raw+Kalman+Median   │
    └─────────────────────┴─────────────────────┘

    All four have tolerance rings (1/2/3m) and LOST indicator.

    Keyboard shortcuts:
      h — help overlay
      q — quit

    Returns:
        fig, axes (2×2), artists dict, view_state dict
    """
    fig, axes = plt.subplots(2, 2, subplot_kw={'projection': 'polar'},
                             figsize=(13, 10))
    fig.canvas.manager.set_window_title('BU04 PDOA — 4-View Dashboard')

    configs = [
        {'title': 'Raw Scatter + Trail',  'color': 'red',   'idx': 0},
        {'title': 'Kalman Filtered',      'color': 'blue',  'idx': 1},
        {'title': 'Density Heatmap',      'color': 'orange','idx': 2},
        {'title': 'Compare: Raw+Kalman+Median', 'color': 'purple', 'idx': 3},
    ]

    artists = {}

    for cfg in configs:
        idx = cfg['idx']
        ax = axes.flat[idx]
        color = cfg['color']

        # Axes config
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.set_thetamin(-90)
        ax.set_thetamax(90)
        ax.set_rmax(10.0)
        ax.set_title(cfg['title'], pad=12, fontsize=9, color=color, weight='bold')

        # Tolerance rings (all plots) — 1/2/3/5/7/10m
        theta = np.linspace(-np.pi / 2, np.pi / 2, 100)
        ring_radii = [1.0, 2.0, 3.0, 5.0, 7.0, 10.0]
        ring_alphas = [0.4, 0.35, 0.3, 0.2, 0.15, 0.1]
        for r, alpha in zip(ring_radii, ring_alphas):
            ax.plot(theta, [r] * 100, '--', color='grey', alpha=alpha, linewidth=0.6)
            ax.fill_between(theta, r - 0.1, r + 0.1, color='grey', alpha=alpha * 0.12)

        # Scatter + Trail (plots 0,1,3)
        if idx != 2:
            artists[f'scat_{idx}'] = ax.scatter(
                [], [], s=70, c=color, zorder=5)
            artists[f'trail_{idx}'] = ax.scatter(
                [], [], s=12, c=color, alpha=0.2, zorder=4)

        # Heatmap (plot 2) — we use pcolormesh updated each frame
        if idx == 2:
            # Pre-allocate mesh (will be updated in update_plot)
            mesh = ax.pcolormesh(
                np.array([[0]]), np.array([[0]]), np.array([[0]]),
                cmap='hot', alpha=0.7, shading='auto', zorder=3
            )
            artists['heatmap_mesh'] = mesh
            artists['heatmap_scat'] = ax.scatter(
                [], [], s=15, c='white', alpha=0.6, zorder=5)

        # Overlay lines for plot 3 (Kalman & Median separate)
        if idx == 3:
            artists['over_k_s'] = ax.scatter(
                [], [], s=15, c='blue', alpha=0.4, zorder=3, marker='s')
            artists['over_m_s'] = ax.scatter(
                [], [], s=15, c='green', alpha=0.4, zorder=3, marker='^')

        # LOST text
        artists[f'lost_{idx}'] = ax.text(
            0, 0, 'LOST', fontsize=16, color='red', alpha=0,
            ha='center', va='center', weight='bold', zorder=10)

    # Help overlay (plot 0, hidden)
    help_text = axes.flat[0].text(
        0, 1.5,
        'h:help q:quit',
        fontsize=7, color='grey', alpha=0.3, ha='center', va='center'
    )
    artists['help_text'] = help_text

    fig.tight_layout(pad=2.5)
    return fig, axes, artists


def compute_heatmap(points, n_bins=30):
    """Compute 2D polar histogram for heatmap display."""
    if len(points) < 5:
        return None, None, None

    angles = [np.radians(m.angle_deg) for m in points]
    ranges = [m.range_cm / 100.0 for m in points]

    # Bin edges
    theta_edges = np.linspace(-np.pi / 2, np.pi / 2, n_bins + 1)
    r_edges = np.linspace(0, 10.0, n_bins + 1)

    hist, _, _ = np.histogram2d(angles, ranges,
                                bins=[theta_edges, r_edges])
    # Smooth slightly
    hist = hist.T  # transpose so rows=r, cols=theta

    theta_centers = (theta_edges[:-1] + theta_edges[1:]) / 2
    r_centers = (r_edges[:-1] + r_edges[1:]) / 2
    T, R = np.meshgrid(theta_centers, r_centers)

    return T, R, hist


def update_plot(frame, data_queues, locks, artists, stats, fig):
    """
    Update all 4 subplots. data_queues is a dict with keys:
    'raw', 'kalman', 'median' — each a deque of Measurement.
    """
    # Gather data under locks
    pts = {}
    for key in ('raw', 'kalman', 'median'):
        with locks[key]:
            pts[key] = list(data_queues[key]) if data_queues[key] else []

    raw = pts['raw']
    kal = pts['kalman']
    med = pts['median']

    all_pts = raw  # primary is raw

    # ── Plot 0: Raw Scatter + Trail ──────────────────────
    _draw_scatter_trail(artists, 0, raw, 'red')

    # ── Plot 1: Kalman Filtered ──────────────────────────
    _draw_scatter_trail(artists, 1, kal, 'blue')

    # ── Plot 2: Density Heatmap ──────────────────────────
    _draw_heatmap(artists, raw, fig.axes[2])

    # ── Plot 3: Overlay Comparison ───────────────────────
    _draw_scatter_trail(artists, 3, raw, 'red')
    _draw_overlay(artists, kal, med)

    # ── LOST indicators ──────────────────────────────────
    for idx in range(4):
        lost = artists[f'lost_{idx}']
        if not all_pts and time.time() - stats['last_packet_time'] > 1.0:
            lost.set_alpha(1.0)
        else:
            lost.set_alpha(0)

    # ── Title bar ────────────────────────────────────────
    r_raw = raw[-1].range_cm if raw else 0
    a_raw = raw[-1].angle_deg if raw else 0
    r_kal = kal[-1].range_cm if kal else 0
    a_kal = kal[-1].angle_deg if kal else 0
    r_med = med[-1].range_cm if med else 0
    a_med = med[-1].angle_deg if med else 0

    fig.suptitle(
        f'Rx:{stats["received"]} Skip:{stats["skipped"]} | '
        f'Raw: {r_raw:.0f}cm {a_raw:.0f}° | '
        f'Kalman: {r_kal:.0f}cm {a_kal:.0f}° | '
        f'Median: {r_med:.0f}cm {a_med:.0f}°',
        fontsize=9, y=0.995
    )

    return list(artists.values())


def _draw_scatter_trail(artists, idx, points, color):
    """Draw scatter point + fading trail for one subplot."""
    scat = artists.get(f'scat_{idx}')
    trail = artists.get(f'trail_{idx}')
    if scat is None or trail is None:
        return

    if not points:
        scat.set_offsets(np.empty((0, 2)))
        trail.set_offsets(np.empty((0, 2)))
        return

    angles = [np.radians(m.angle_deg) for m in points]
    ranges = [m.range_cm / 100.0 for m in points]

    scat.set_offsets(np.c_[angles[-1:], ranges[-1:]])
    scat.set_alpha(1.0)

    tn = min(len(angles), 200)
    trail.set_offsets(np.c_[angles[-tn:], ranges[-tn:]])


def _draw_heatmap(artists, points, ax):
    """Draw density heatmap for plot 2."""
    mesh = artists.get('heatmap_mesh')
    hscat = artists.get('heatmap_scat')

    # Clear old mesh
    if mesh is not None:
        mesh.remove()
        artists['heatmap_mesh'] = None

    if not points or len(points) < 5:
        if hscat:
            hscat.set_offsets(np.empty((0, 2)))
        return

    T, R, hist = compute_heatmap(points, n_bins=25)
    if T is None:
        return

    # Draw new mesh
    mesh = ax.pcolormesh(T, R, hist, cmap='hot', alpha=0.7,
                         shading='auto', zorder=3)
    artists['heatmap_mesh'] = mesh

    # Overlay recent points
    angles = [np.radians(m.angle_deg) for m in points[-50:]]
    ranges = [m.range_cm / 100.0 for m in points[-50:]]
    if hscat:
        hscat.set_offsets(np.c_[angles, ranges])


def _draw_overlay(artists, kalman_pts, median_pts):
    """Draw Kalman and Median as separate markers on overlay plot."""
    for key, pts, marker in [('over_k_s', kalman_pts, 's'),
                              ('over_m_s', median_pts, '^')]:
        artist = artists.get(key)
        if artist is None:
            continue
        if pts:
            a = [np.radians(m.angle_deg) for m in pts[-200:]]
            r = [m.range_cm / 100.0 for m in pts[-200:]]
            artist.set_offsets(np.c_[a, r])
        else:
            artist.set_offsets(np.empty((0, 2)))


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
        '--calibrate', '-c', action='store_true',
        help='Calibration mode: collect 100 samples, print recommended AT+PDOAOFF/AT+RNGOFF'
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

    # Shared state — three deques for three filter modes
    deques = {
        'raw': deque(maxlen=500),
        'kalman': deque(maxlen=500),
        'median': deque(maxlen=500),
    }
    locks = {
        'raw': threading.Lock(),
        'kalman': threading.Lock(),
        'median': threading.Lock(),
    }
    stop_event = threading.Event()
    stats = {
        'received': 0,
        'skipped': 0,
        'errors': 0,
        'last_packet_time': time.time(),
        'last_raw_time': time.time(),
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
        print("Collecting 100 raw samples...")
        print()

        # Start reader
        reader = threading.Thread(
            target=reader_thread,
            args=(ser, deques, locks, fmt, stop_event, stats),
            daemon=True, name='serial-reader'
        )
        reader.start()

        # Collect 100 raw samples
        samples_r = []
        samples_a = []
        while len(samples_r) < 100:
            with locks['raw']:
                items = list(deques['raw'])
            for m in items[len(samples_r):]:
                if len(samples_r) >= 100:
                    break
                samples_r.append(m.range_cm)
                samples_a.append(m.angle_deg)
                n = len(samples_r)
                if n % 10 == 0:
                    print(f"\r  [{n}/100] D={m.range_cm:.0f}cm P={m.angle_deg:.0f}°", end='', flush=True)
            time.sleep(0.05)

        stop_event.set()
        ser.close()
        print()

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
        print("  Measure TRUE distance/angle with a ruler, then:")
        print()
        rngoff = 100 - int(mean_r)
        pdoaoff = -int(mean_a)
        print(f"  AT+RNGOFF={rngoff}       # if true distance = 100 cm")
        print(f"  AT+PDOAOFF={pdoaoff}     # if true angle = 0°")
        print(f"  AT+SAVE")
        print()
        print(f"  Or in include/config.h:")
        print(f"    #define RANGE_OFFSET_CM  {rngoff}")
        print(f"    #define PDOA_OFFSET_DEG  {pdoaoff}")
        print()
        return

    # ── Normal visualization mode ─────────────────────────
    reader = threading.Thread(
        target=reader_thread,
        args=(ser, deques, locks, fmt, stop_event, stats),
        daemon=True, name='serial-reader'
    )
    reader.start()

    # CSV logger — logs raw data
    csv_thread = threading.Thread(
        target=csv_logger_thread,
        args=(deques['raw'], locks['raw'], stop_event, csv_path),
        daemon=True, name='csv-logger'
    )
    csv_thread.start()

    # Setup 4-view dashboard
    print("Starting 4-view dashboard (close window or Ctrl+C to exit)...")
    fig, axes, artists = setup_polar_plot()

    ani = FuncAnimation(
        fig, update_plot,
        fargs=(deques, locks, artists, stats, fig),
        interval=100,
        blit=False,
        cache_frame_data=False,
    )

    def on_close(event=None):
        print("\nShutting down...")
        stop_event.set()
        ser.close()
        print(f"CSV log: {csv_path}")
        print(f"Stats: {stats['received']} rx, {stats['skipped']} skip, {stats['errors']} err")

    fig.canvas.mpl_connect('close_event', on_close)

    try:
        plt.show()
    except KeyboardInterrupt:
        on_close()


if __name__ == '__main__':
    main()
