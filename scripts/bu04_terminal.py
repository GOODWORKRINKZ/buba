#!/usr/bin/env python3
"""
BU04 Direct USB AT Command Terminal

Interactive terminal for direct communication with BU04 module
via USB CDC (PA11/PA12 on STM32F103).

Usage:
    python3 scripts/bu04_terminal.py [port] [baud]
    
Default: /dev/ttyACM0 @ 115200

Commands (prefixed with '.'):
    .help       — show this help
    .raw        — toggle raw mode (show \r\n)
    .burst      — toggle burst mode (send multiple cmds with ;)
    .disc       — AT+DISC? discover devices
    .list       — AT+GETDLIST get tag list
    .mode       — AT+GETUWBMODE get UWB mode
    .cfg        — AT+GETCFG get config
    .addtag ID  — AT+ADDTAG=ID add tag (PDOA mode)
    .deltag ID  — AT+DELTAG=ID delete tag
    .dist ID    — AT+DISTANCE=ID measure distance (TWR mode)
    .pdoaoff N  — AT+PDOAOFF=N set PDOA angle offset
    .rngoff N   — AT+RNGOFF=N set range offset
    .save       — AT+SAVE save config to flash
    .reboot     — AT+RST reboot module
    .quit       — exit

Burst mode: prefix with '!' then separate commands with ';'
    Example:  !AT+SETCFG=1,0,1,1;AT+SAVE
"""

import serial
import sys
import time
import threading
import readline  # enables line editing and history

# ── ANSI colors ──────────────────────────────────────────────
C = {
    'R': '\033[91m',  # Red
    'G': '\033[92m',  # Green
    'Y': '\033[93m',  # Yellow
    'B': '\033[94m',  # Blue
    'M': '\033[95m',  # Magenta
    'C': '\033[96m',  # Cyan
    'W': '\033[97m',  # White
    'X': '\033[0m',   # Reset
    'b': '\033[1m',   # Bold
    'd': '\033[2m',   # Dim
}

# ── State ────────────────────────────────────────────────────
g_raw_mode = False
g_burst_mode = False
g_running = True

# ── Reader thread ────────────────────────────────────────────
def reader_thread(ser):
    """Continuously read from serial and print"""
    global g_running
    while g_running:
        try:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                text = data.decode('utf-8', errors='replace')
                if g_raw_mode:
                    print(f"{C['d']}[RAW]{C['X']} {data!r}")
                else:
                    # Print each line nicely
                    for line in text.split('\n'):
                        line = line.rstrip('\r')
                        if line:
                            print(f"{C['C']}← {line}{C['X']}")
            else:
                time.sleep(0.05)
        except (OSError, serial.SerialException):
            if g_running:
                print(f"\n{C['R']}[DISCONNECTED]{C['X']}")
            break
        except Exception as e:
            if g_running:
                print(f"{C['R']}[ERR] {e}{C['X']}")

# ── Help ─────────────────────────────────────────────────────
def show_help():
    print(f"""
{C['b']}{C['Y']}╔══════════════════════════════════════════════════╗
║     BU04 Direct USB AT Command Terminal         ║
╚══════════════════════════════════════════════════╝{C['X']}

{C['b']}AT Commands (just type them):{C['X']}
  AT              — ping (should respond OK)
  AT+GETCFG       — get config (ID, Role, CH, Rate)
  AT+SETCFG=X1,X2,X3,X4  — set config
  AT+GETUWBMODE   — get UWB mode (0=TWR, 1=PDOA)
  AT+SETUWBMODE=N — set UWB mode
  AT+GETDLIST     — get tag list (JSON)
  AT+DISC?        — discover devices
  AT+ADDTAG=ID    — add tag (PDOA mode)
  AT+DELTAG=ID    — delete tag
  AT+DISTANCE=ID  — measure distance (TWR mode)
  AT+PDOAOFF=N    — set PDOA angle offset (degrees*10)
  AT+RNGOFF=N     — set range offset (cm)
  AT+USER_CMD=N   — set output format (0=stream, 1=query)
  AT+SAVE         — save config to flash
  AT+RST          — reboot module
  AT+SWVER?       — firmware version

{C['b']}Dot Commands:{C['X']}
  .help           — this help
  .raw            — toggle raw mode (show raw bytes)
  .burst          — toggle burst mode (prefix '!')
  .cfg            — shortcut for AT+GETCFG
  .mode           — shortcut for AT+GETUWBMODE
  .list           — shortcut for AT+GETDLIST
  .disc           — shortcut for AT+DISC?
  .addtag ID      — shortcut for AT+ADDTAG=ID
  .deltag ID      — shortcut for AT+DELTAG=ID
  .pdoaoff N      — shortcut for AT+PDOAOFF=N
  .rngoff N       — shortcut for AT+RNGOFF=N
  .save           — shortcut for AT+SAVE
  .reboot         — shortcut for AT+RST
  .quit / .exit   — exit

{C['b']}Burst mode:{C['X']} prefix with ! and separate commands with ;
  Example: !AT+SETCFG=1,0,1,1;AT+SAVE
""")

# ── Main ─────────────────────────────────────────────────────
def main():
    global g_raw_mode, g_burst_mode, g_running
    
    # Parse args
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    
    print(f"{C['b']}{C['Y']}=== BU04 Direct USB Terminal ==={C['X']}")
    print(f"Port: {port} @ {baud} baud")
    print(f"Type {C['b']}.help{C['X']} for commands, {C['b']}.quit{C['X']} to exit")
    
    # Open serial
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(0.3)
        ser.reset_input_buffer()
        print(f"{C['G']}Connected!{C['X']}\n")
    except serial.SerialException as e:
        print(f"{C['R']}Failed to open {port}: {e}{C['X']}")
        sys.exit(1)
    
    # Start reader
    reader = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    reader.start()
    
    # Interactive loop
    try:
        while g_running:
            try:
                line = input(f"{C['b']}BU04>{C['X']} ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            
            if not line:
                continue
            
            # ── Dot commands ──
            if line.startswith('.'):
                parts = line.split()
                cmd = parts[0].lower()
                
                if cmd in ('.quit', '.exit', '.q'):
                    break
                elif cmd == '.help':
                    show_help()
                elif cmd == '.raw':
                    g_raw_mode = not g_raw_mode
                    print(f"{C['Y']}Raw mode: {g_raw_mode}{C['X']}")
                elif cmd == '.burst':
                    g_burst_mode = not g_burst_mode
                    print(f"{C['Y']}Burst mode: {g_burst_mode}{C['X']}")
                elif cmd == '.cfg':
                    ser.write(b'AT+GETCFG\r\n')
                    print(f"{C['d']}> AT+GETCFG{C['X']}")
                elif cmd == '.mode':
                    ser.write(b'AT+GETUWBMODE\r\n')
                    print(f"{C['d']}> AT+GETUWBMODE{C['X']}")
                elif cmd == '.list':
                    ser.write(b'AT+GETDLIST\r\n')
                    print(f"{C['d']}> AT+GETDLIST{C['X']}")
                elif cmd == '.disc':
                    ser.write(b'AT+DISC?\r\n')
                    print(f"{C['d']}> AT+DISC?{C['X']}")
                elif cmd == '.addtag' and len(parts) > 1:
                    cmd_str = f"AT+ADDTAG={parts[1]}\r\n"
                    ser.write(cmd_str.encode())
                    print(f"{C['d']}> {cmd_str.strip()}{C['X']}")
                elif cmd == '.deltag' and len(parts) > 1:
                    cmd_str = f"AT+DELTAG={parts[1]}\r\n"
                    ser.write(cmd_str.encode())
                    print(f"{C['d']}> {cmd_str.strip()}{C['X']}")
                elif cmd == '.pdoaoff' and len(parts) > 1:
                    cmd_str = f"AT+PDOAOFF={parts[1]}\r\n"
                    ser.write(cmd_str.encode())
                    print(f"{C['d']}> {cmd_str.strip()}{C['X']}")
                elif cmd == '.rngoff' and len(parts) > 1:
                    cmd_str = f"AT+RNGOFF={parts[1]}\r\n"
                    ser.write(cmd_str.encode())
                    print(f"{C['d']}> {cmd_str.strip()}{C['X']}")
                elif cmd == '.save':
                    ser.write(b'AT+SAVE\r\n')
                    print(f"{C['d']}> AT+SAVE{C['X']}")
                elif cmd == '.reboot':
                    ser.write(b'AT+RST\r\n')
                    print(f"{C['d']}> AT+RST{C['X']}")
                else:
                    print(f"{C['R']}Unknown command: {cmd}{C['X']}")
                continue
            
            # ── Burst mode ──
            if line.startswith('!'):
                line = line[1:]
                print(f"{C['Y']}[BURST] {line}{C['X']}")
                for subcmd in line.split(';'):
                    subcmd = subcmd.strip()
                    if subcmd:
                        cmd_bytes = (subcmd + '\r\n').encode()
                        ser.write(cmd_bytes)
                        print(f"{C['d']}> {subcmd}{C['X']}")
            else:
                # ── Normal AT command ──
                cmd_bytes = (line + '\r\n').encode()
                ser.write(cmd_bytes)
                print(f"{C['d']}> {line}{C['X']}")
            
            # Small delay between commands
            time.sleep(0.1)
    
    finally:
        g_running = False
        ser.close()
        print(f"\n{C['Y']}Disconnected. Bye!{C['X']}")

if __name__ == '__main__':
    main()
