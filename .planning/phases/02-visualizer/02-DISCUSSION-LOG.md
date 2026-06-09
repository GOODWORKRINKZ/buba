# Phase 2: Python Visualizer — Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in 02-CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-06-09
**Phase:** 02-visualizer
**Areas discussed:** Data format, Port discovery, Visual style, CSV columns, Multi-device handling

---

## Data Format

| Option | Description | Selected |
|--------|-------------|----------|
| Parse native STM32 JSON only | Read direct USB anchor JSON output, no ESP32 needed | — |
| Keep ESP32 + our CSV | Use anchor1_pdoa firmware on ESP32, generate CSV as planned | — |
| Support both formats | Auto-detect by first line pattern | ✓ |

**Decision:** Auto-detect format by first meaningful line: `{` → JSON parser, `PDOA,` → CSV parser. Remember format for the session.

**User context:** Second device may also be converted to direct USB later, but not yet. Need both formats now.

---

## Format Detection Method

| Option | Description | Selected |
|--------|-------------|----------|
| Auto by first line | Read first meaningful line, detect pattern, lock format | ✓ |
| Flag --format json|csv | User explicitly specifies format | — |
| Try both parsers | Attempt JSON first, fallback to CSV — slower but robust | — |

---

## Port Discovery

| Option | Description | Selected |
|--------|-------------|----------|
| Auto-detect by VID:PID | Scan /dev/ttyACM*, check 0483:5740, send AT+DECA$ | — |
| Command-line argument | python3 visualizer.py /dev/ttyACM1 | — |
| Both (auto + manual) | No arg → auto-detect; with arg → use specified port | ✓ |

---

## Visual Style

| Option | Description | Selected |
|--------|-------------|----------|
| Scatter + trail (50 pts) | Current position bright, history fading | — |
| Heatmap density | Color map of where tag was most often | — |
| Scatter + trail + tolerance rings | Points + fading trail + dashed rings at 1/2/3m | — |
| All on one graph | Scatter + trail + rings + LOST indicator + optional heatmap | ✓ |

**Decision:** Everything on one graph. Heatmap deferred as `--heatmap` option.

---

## CSV Columns

| Option | Description | Selected |
|--------|-------------|----------|
| Minimal: timestamp, range_cm, angle_deg | Only distance and angle | — |
| Full JSON: all TWR fields | timestamp + all parsed fields + raw JSON | ✓ |
| Standard: timestamp, range_m, angle_deg, seq, x_m, y_m | As originally planned | — |

**Decision:** Full JSON dump for maximum data. Column `raw_json` preserves the original line for debugging.

---

## Multiple BU04 Handling

| Option | Description | Selected |
|--------|-------------|----------|
| One anchor only | Connect to single port, tag visualization not needed | — |
| Two windows — anchor and tag | Separate windows for each device | — |
| Auto — find all BU04, identify anchor | Scan all STM32 ports, AT+DECA$ to find anchor | ✓ |

**Decision:** Auto-scan. Tag ports (Role:0) ignored — they don't respond to AT+DECA$ as "PDOA Node".

---

## Key Context from Phase 1

- **Direct USB works:** Anchor BU04 via PA11/PA12, VID:0483 PID:5740, no `>` echo
- **JSON format confirmed:** `{"TWR":{"D":121,"P":4,"R":0,"T":774068,...}}` with JS header
- **AT+DECA$ handshake:** Returns `{"Info":{"Device":"PDOA Node",...}}` — used for port identification
- **PDOA performance:** ~4.1 Hz, distance ±5cm at 135cm, angle -1° to +38°
- **dtr=False:** Still needed for ESP32-C3 (prevents reset on serial connect)
- **AT+RST kills USB CDC:** On direct STM32, use physical power cycle — visualizer should NOT send AT+RST
