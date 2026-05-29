# MightyMavlink

A small C++17 wrapper around the MAVLink v2 C library for a camera / VIO companion board that talks to an ArduPilot autopilot over a serial port.

The library handles the boilerplate: identifies itself as a Visual-Inertial Odometry component (`MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`, compid `197`), emits a 1 Hz `HEARTBEAT`, packs `ODOMETRY` messages, and dispatches `COMMAND_LONG` requests addressed to the board (`MAV_CMD_USER_1..5`) to user-provided callbacks.

## Layout

```
include/might_mavlink/MightMavlink.h   public API
src/MightMavlink.cpp                   implementation
src/SerialPort.{h,cpp}                 Linux/macOS serial wrapper (raw, 8N1, non-blocking)
examples/board_mock.cpp                fake VIO board, streams a spiraling pose @ 30 Hz
examples/loopback_test.cpp             PTY-based round-trip test (wired into ctest)
docker/Dockerfile                      ArduCopter SITL image
docker/run.sh                          docker run wrapper with the right port mapping
```

## Build

Requirements: CMake ≥ 3.16, a C++17 compiler, network access on the first configure (the build fetches `mavlink/c_library_v2` via `FetchContent`).

```sh
cmake -S . -B build
cmake --build build -j
```

Artifacts land in `build/`:

- `libmight_mavlink.a` — the static library
- `board_mock` — example that pretends to be the VIO board
- `loopback_test` — hardware-free smoke test

Run the smoke test (via `ctest`, or directly):

```sh
ctest --test-dir build --output-on-failure
# or
./build/loopback_test
```

The loopback test opens a PTY pair, runs the library on the slave side, and drives raw MAVLink bytes on the master side. It checks that `HEARTBEAT` is emitted, `ODOMETRY` round-trips, every `MAV_CMD_USER_n` triggers its handler and is ACKed, and that `setRecording(true)` flips the heartbeat `custom_mode` bit and emits a `STATUSTEXT`.

## Library usage

```cpp
#include "might_mavlink/MightMavlink.h"

might_mavlink::MightMavlink mav;             // sysid=1, compid=197
mav.open("/dev/ttyUSB0", 921600);

mav.onStart         ([]{ /* GCS pressed start */ });
mav.onStop          ([]{ /* ... */ });
mav.onStartRecording([]{ /* ... */ });

might_mavlink::Cov21 nan_cov; nan_cov.fill(std::nanf(""));

while (running) {
    mav.poll();                              // drives RX + 1 Hz HEARTBEAT
    if (have_new_vio_sample) {
        mav.sendOdometry(t_us, pose, twist, nan_cov, nan_cov, /*quality=*/100);
    }
}
```

Pose is in a local NED-like frame (`MAV_FRAME_LOCAL_FRD`), twist is in body FRD (`MAV_FRAME_BODY_FRD`). Pass an all-NaN `Cov21` to skip covariance reporting.

`SerialPort::open` also accepts a path of the form `fd:N` to wrap an already-open file descriptor — useful for PTYs (that is how `loopback_test` runs without hardware).

## SITL (no flight controller needed)

The `docker/` directory builds an Ubuntu 22.04 image containing ArduCopter SITL plus MAVProxy, preconfigured so that `SERIAL2` speaks MAVLink2 at 921 kBaud and ArduPilot does **not** try to fuse incoming `ODOMETRY` into its EKF (`VISO_TYPE=0` — it just forwards to the GCS link).

### One-time image build + run

```sh
./docker/run.sh
```

First run takes ~5–10 minutes (cloning ArduPilot and compiling SITL). Subsequent runs are instant. The script wires up:

- Host TCP `5763` → SITL `SERIAL2`  (where `board_mock` connects)
- Host TCP `5760` → SITL `SERIAL0` console (extra MAVLink tooling, optional)
- SITL → `host.docker.internal:14550` UDP — point QGroundControl at UDP `14550` on the Mac

### Bridge SITL's TCP port to a local PTY and run `board_mock`

`board_mock` speaks to a serial path, so use `socat` to expose SITL's TCP port as a PTY:

```sh
# terminal A — SITL
./docker/run.sh

# terminal B — bridge TCP 5763 to a PTY
socat -d -d pty,raw,echo=0,link=/tmp/board_pty tcp:127.0.0.1:5763

# terminal C — run the mock board against that PTY
./build/board_mock /tmp/board_pty 921600
```

`board_mock` starts emitting `ODOMETRY` once it receives a `START` command (`MAV_CMD_USER_1`). Send it from QGroundControl's MAVLink inspector (or `pymavlink`) addressed to sysid `1`, compid `197`.

## Connecting to real hardware

Point `board_mock` (or your own code using the library) at the actual serial device wired to the autopilot:

```sh
./build/board_mock /dev/serial0 115200      # Pi GPIO header to FC TELEM port
./build/board_mock /dev/ttyUSB0 921600      # USB-to-UART adapter
```

### Flight controller parameters

The FC's serial port that's wired to the companion must be configured for MAVLink. From any GCS (Parameters / Full Parameter List), set:

| Parameter | Value | Why |
|---|---|---|
| `SERIALn_PROTOCOL` | `2` | MAVLink2 — required for the wrapper's messages to be parsed and routed |
| `SERIALn_BAUD` | matches `board_mock`'s baud (`115` for 115200, `921` for 921600) | Mismatched baud → silence in one or both directions |
| `VISO_TYPE` | `0` for passthrough | If non-zero, ArduPilot consumes `ODOMETRY` into its EKF and won't forward it to the GCS link |

`n` is the FC's serial port number — check the autopilot manual for which `SERIALn` maps to TELEM1 / TELEM2 / etc. Both directions of the UART must be physically wired (companion TX → FC RX, companion RX → FC TX, common GND); a "telemetry-out only" cable will look like the board is silent from the FC's perspective.

### Custom commands the wrapper listens for

The library accepts five `COMMAND_LONG` messages addressed to compid `197`:

| Command | ID | Handler | Effect in `board_mock` |
|---|---|---|---|
| `MAV_CMD_USER_1` | 31010 | `onStart` | Begin streaming `ODOMETRY` at 30 Hz |
| `MAV_CMD_USER_2` | 31011 | `onStop` | Stop streaming `ODOMETRY` |
| `MAV_CMD_USER_3` | 31012 | `onReset` | No-op handler (ACK only) |
| `MAV_CMD_USER_4` | 31013 | `onStartRecording` | Set HEARTBEAT `custom_mode = 1`, emit STATUSTEXT `"REC ON"` |
| `MAV_CMD_USER_5` | 31014 | `onStopRecording` | Set `custom_mode = 0`, emit STATUSTEXT `"REC OFF"` |

### Viewing ODOMETRY in a GCS

`ODOMETRY` is msgid 331. Look under **sysid 1, compid 197** — the wrapper identifies itself as `MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`. Until you send `MAV_CMD_USER_1`, only `HEARTBEAT` will be present under compid 197.

- **Mission Planner**: `Ctrl+F` → click *Mavlink Inspector* → expand sysid 1 → compid 197 → `ODOMETRY`.
- **QGroundControl**: *Q menu* (top-left) → *Analyze Tools* → *MAVLink Inspector* → expand *System 1 → Component 197 → ODOMETRY*.

### Sending commands from Mission Planner

MP has a dialog for arbitrary `COMMAND_LONG`:

1. `Ctrl+F` → click *Mavlink* (some MP versions label it *MAV Cmd*).
2. Fill in:
   - **Target System**: `1`
   - **Target Component**: `197`
   - **Command**: the numeric ID from the table above
   - **Confirmation** and **params 1–7**: `0`
3. Click *Send*.

You'll see `COMMAND_ACK ACCEPTED` in MP and `[cmd] <name>` in `board_mock`'s stdout.

### Sending commands from QGroundControl (via MAVProxy)

QGroundControl has **no built-in UI** for sending arbitrary `COMMAND_LONG` to a non-autopilot component. (Its *MAVLink Console* sends shell text via `SERIAL_CONTROL`, which is unrelated and will not reach compid 197.) The standard workaround is to slot **MAVProxy** in between the FC and QGC and use its CLI for command injection.

Disconnect QGC from the FC first — only one process can hold the link. Then on your laptop:

```sh
pip3 install MAVProxy

# Adjust device and baud to match your FC's USB / TELEM port
mavproxy.py --master=/dev/tty.usbmodem1401,57600 \
            --out=udp:127.0.0.1:14550
```

In QGC: *Application Settings* → *Comm Links* → *Add* → Type *UDP*, Listening Port `14550` → connect.

In the MAVProxy prompt, **retarget to the board** (the default target is the autopilot, compid 1) and send commands:

```
set target_component 197
long MAV_CMD_USER_1 0 0 0 0 0 0 0      # START
long MAV_CMD_USER_4 0 0 0 0 0 0 0      # START_RECORDING
long MAV_CMD_USER_5 0 0 0 0 0 0 0      # STOP_RECORDING
long MAV_CMD_USER_3 0 0 0 0 0 0 0      # RESET (no-op handler)
long MAV_CMD_USER_2 0 0 0 0 0 0 0      # STOP
```

Run `set target_component` with no value to print the current target and confirm the switch took effect.

Each line should produce `Got COMMAND_ACK: USER_n: ACCEPTED` in MAVProxy and a `[cmd] ...` line in `board_mock`'s terminal. After START, ODOMETRY starts appearing in QGC's MAVLink Inspector under compid 197.

When you want subsequent commands to go to the autopilot again, flip back with `set target_component 1`. If a `long ...` returns `UNSUPPORTED` instead of `ACCEPTED`, the target wasn't switched — the autopilot received the command and rejected it.

### Sending commands with pymavlink (no MAVProxy CLI needed)

Same effect from a single shell command, from any terminal with a MAVLink-reachable endpoint:

```sh
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('udp:127.0.0.1:14551')
m.wait_heartbeat()
for cmd in (31010, 31013, 31014, 31012, 31011):
    m.mav.command_long_send(1, 197, cmd, 0, 0,0,0,0,0,0,0)
    print('sent', cmd)
    import time; time.sleep(1)
"
```

For this to coexist with QGC, add `--out=udp:127.0.0.1:14551` as a second fanout when launching MAVProxy. Without MAVProxy, point `mavutil.mavlink_connection(...)` directly at the FC (`/dev/tty.usbmodem1401`, etc.) — but you'll need QGC disconnected to free the device.

### Quick troubleshooting

| Symptom | Likely cause |
|---|---|
| FC heartbeats visible to the companion, but **compid 197 never appears in the GCS** | Pi→FC direction not wired, or `SERIALn_PROTOCOL` / `SERIALn_BAUD` mismatch on the FC port the companion is on |
| MAVProxy `Got COMMAND_ACK: USER_n: UNSUPPORTED` | Target wasn't switched — autopilot received the command. Run `set target_component 197` and resend |
| `ODOMETRY` shows up in MAVProxy but **not in the GCS** | `VISO_TYPE` is non-zero on the FC, so ArduPilot is consuming it into the EKF instead of forwarding. Set `VISO_TYPE=0` |
| `board_mock` opens the port but is killed when SSH disconnects | Launch it detached (`systemd-run --unit=board-mock ...`) or run it inside `screen` / `tmux` |
