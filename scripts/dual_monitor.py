#!/usr/bin/env python3
"""
Dual serial monitor for BU04 UWB test bench.
Opens two serial ports simultaneously and displays logs side-by-side.

Usage:
    python3 scripts/dual_monitor.py [anchor_port] [tag_port]
    
Default ports: /dev/ttyACM1 (anchor), /dev/ttyACM3 (tag)
"""

import serial
import threading
import sys
import time
from datetime import datetime

# ANSI color codes
COLORS = {
    'anchor': '\033[94m',  # Blue
    'tag': '\033[92m',     # Green
    'reset': '\033[0m',
    'bold': '\033[1m',
    'dim': '\033[2m',
}

def timestamp():
    """Return current time as HH:MM:SS.mmm"""
    now = datetime.now()
    return now.strftime("%H:%M:%S.") + f"{now.microsecond // 1000:03d}"

def read_serial(port, baud, label, color):
    """Read from serial port and print with label and color"""
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"{COLORS['bold']}{color}[{label}] Connected to {port} @ {baud}{COLORS['reset']}")
        
        while True:
            try:
                line = ser.readline().decode('utf-8', errors='replace').rstrip()
                if line:
                    ts = timestamp()
                    print(f"{COLORS['dim']}{ts}{COLORS['reset']} {color}[{label}]{COLORS['reset']} {line}")
            except UnicodeDecodeError:
                pass
            except Exception as e:
                print(f"{color}[{label}] Read error: {e}{COLORS['reset']}")
                break
                
    except serial.SerialException as e:
        print(f"{COLORS['bold']}\033[91m[{label}] Failed to open {port}: {e}{COLORS['reset']}")
        sys.exit(1)

def main():
    # Parse arguments
    if len(sys.argv) >= 3:
        anchor_port = sys.argv[1]
        tag_port = sys.argv[2]
    else:
        anchor_port = '/dev/ttyACM0'
        tag_port = '/dev/ttyACM1'
    
    baud = 115200
    
    print(f"{COLORS['bold']}{'='*70}{COLORS['reset']}")
    print(f"{COLORS['bold']}BU04 UWB Dual Serial Monitor{COLORS['reset']}")
    print(f"{COLORS['bold']}{'='*70}{COLORS['reset']}")
    print(f"Anchor: {anchor_port}")
    print(f"Tag:    {tag_port}")
    print(f"Baud:   {baud}")
    print(f"{COLORS['bold']}{'='*70}{COLORS['reset']}")
    print(f"Press Ctrl+C to exit\n")
    
    # Start threads for each port
    anchor_thread = threading.Thread(
        target=read_serial,
        args=(anchor_port, baud, 'ANCHOR', COLORS['anchor']),
        daemon=True
    )
    
    tag_thread = threading.Thread(
        target=read_serial,
        args=(tag_port, baud, 'TAG', COLORS['tag']),
        daemon=True
    )
    
    anchor_thread.start()
    tag_thread.start()
    
    # Keep main thread alive
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print(f"\n{COLORS['bold']}{'='*70}{COLORS['reset']}")
        print(f"{COLORS['bold']}Exiting...{COLORS['reset']}")
        sys.exit(0)

if __name__ == '__main__':
    main()
