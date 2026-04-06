# FPM Sim (Marlin-like 4-axis Printer Simulator)

A Python simulator that accepts G-code, processes it through a Marlin-style command loop, and renders an interactive 3D scene in the browser.

## What is implemented

- 4-axis machine state: X, Y, Z, A (A = rotary around Z)
- Marlin-like serial behavior:
  - `ok` after successful commands
  - `echo:busy: processing` during long-running operations
  - `Error:...` messages on failures
  - Linux virtual serial endpoint (PTY) for host-software integration
- G-code support:
  - `G0` / `G1` linear moves
  - `G28` realistic multi-stage homing (fast seek, retract, slow seek per axis)
  - `G30` single probe at current XY downward in Z
  - `G4` dwell (`P` in ms or `S` in s)
  - `G90` / `G91` absolute/relative mode
  - `G92` set current position
  - `G38.2` probe toward target, stop on contact, error on miss
  - `G38.3` probe toward target, stop on contact, no error on miss
  - `G38.4` probe away from target, stop on contact break, error on miss
  - `G38.5` probe away from target, stop on contact break, no error on miss
- M-code support:
  - `M17`, `M18`, `M84` motors enable/disable
  - `M114` report position
  - `M24` resume simulated print state
  - `M108` cancel wait state
  - `M876` answer/query host prompt state
  - `M210` set/report A-axis max rotary speed (`A` parameter)
  - `M211` software endstops on/off (`S0` or `S1`)
  - `M201` set acceleration per axis
  - `M203` set/report max feedrate per axis (mm/s)
  - `M204` set accel profile (`S` for XYZ, `A` for rotary)
  - `M503` report current simulator motion settings
  - `M105` (basic temperature keepalive style response)
  - `M110`, `M115`, `M400` (host compatibility helpers)
- Interactive renderer:
  - Toolhead movement + A-axis rotation visualization
  - Motion path visualization
  - Mesh import (`.stl`, `.obj`, `.ply`) and display
  - Live serial monitor and state monitor

## Architecture

### 1) Transport + protocol boundary

- FastAPI HTTP endpoints for:
  - enqueueing G-code blocks
  - uploading probe meshes
  - reading state
  - discovering the virtual serial port path (`/api/serial`)
- WebSocket stream for:
  - serial-like text lines (`ok`, `echo:...`, `Error:...`)
  - live machine-state snapshots for UI updates

### 2) Simulator core (deterministic command engine)

- `GcodeParser` parses raw text lines into command objects.
- `MarlinEngine` dispatches parsed commands to handlers.
- `PrinterState` owns machine kinematics and mutable state.
- `ProbeWorld` owns imported meshes and ray/segment probe checks.

This separation keeps protocol concerns outside command semantics, so you can later swap transport layers (CLI, TCP serial bridge, test harness) without rewriting motion logic.

The parser also accepts Marlin-style line numbers/checksum blocks such as `N12 G1 X10 Y5*42`.

### 3) Execution model

- Commands are queued in order (single worker task).
- Each command:
  - executes deterministically
  - emits zero or more serial messages
  - optionally simulates run-time delay with busy responses
  - ends with `ok` on success

Single-threaded command execution mirrors firmware behavior and avoids race conditions.

### Motion timing model

- Moves are simulated with acceleration-aware trapezoidal/triangular profiles (not constant-speed only).
- Path speed is constrained by:
  - commanded feedrate (`F`)
  - per-axis max feedrate (`M203`)
  - per-axis acceleration (`M201` / `M204`)
- Rotary A-axis uses the same profile logic with its own speed/accel limits.

### 4) Frontend visualization layer

- Three.js scene shows bed, toolhead, path, and imported meshes.
- UI submits G-code blocks and receives real-time updates via WebSocket.
- Rendered toolhead motion follows backend trapezoid profiles (velocity + acceleration), not just endpoint jumps.

## Project layout

- `app/main.py`: FastAPI app, runtime, queue worker, API and WebSocket
- `app/gcode_engine.py`: command dispatch + Marlin-like responses
- `app/gcode_parser.py`: line parsing and comment stripping
- `app/machine.py`: machine state and move/homing mechanics
- `app/probing.py`: mesh loading and segment-intersection probing
- `app/static/index.html`: UI shell
- `app/static/app.js`: Three.js renderer + client logic
- `app/static/styles.css`: UI styling

## Run

### One-command startup (recommended)

```bash
./start.sh
```

This script will:

- create `.venv` if missing
- recover `pip` if needed
- install dependencies from `requirements.txt`
- start the simulator

Optional environment variables:

- `HOST=0.0.0.0` (default `127.0.0.1`)
- `PORT=8001` (default `8000`)
- `SKIP_INSTALL=1` to skip dependency install on startup
- `AUTO_PORT=1` auto-switch to next free port if selected port is busy (default `1`)
- `MAX_PORT_SEARCH=25` number of subsequent ports to scan when auto-switching

### Manual startup

1. Install dependencies:

   pip install -r requirements.txt

2. Start the simulator:

   uvicorn app.main:app --reload

3. Open:

   http://127.0.0.1:8000

## Connect Through Virtual Serial (Linux)

When the app starts, it creates a pseudo-serial device (PTY) and exposes it via:

- `GET /api/serial`

to find the serial port, run the following command:

- `url -sS http://127.0.0.1:PORT/api/serial`

Example response:

```json
{
  "enabled": true,
  "path": "/dev/pts/7",
  "error": null
}
```

Your external virtual machine / slicer / host software can open that `path` as a serial port and exchange Marlin-like lines (`G...`, `M...`, `ok`, `echo:busy: processing`, etc.).

## Notes on Marlin fidelity

This is an engineering simulator and not a byte-for-byte Marlin clone. It mirrors the core host-facing interaction pattern (`ok`, `busy`, command ordering, and key G/M-code semantics) while staying easy to extend.

If you want tighter Marlin parity next, add:

- line number + checksum protocol (`N...*..`)
- planner-buffer depth simulation and `busy: paused for user`
- EEPROM-like persistent settings (`M500/M501`)
- richer endstop/homing/probe state transitions
