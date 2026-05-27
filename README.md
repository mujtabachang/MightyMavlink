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

### Talking to real hardware

Skip the docker bits — point `board_mock` (or your own code using the library) at the actual serial device:

```sh
./build/board_mock /dev/ttyUSB0 921600
```

On the autopilot side, the relevant ArduPilot parameters are the same as the SITL defaults: set `SERIALn_PROTOCOL=2` (MAVLink2) and the matching baud on whichever serial port the board is wired to, and `VISO_TYPE=0` if you only want forwarding rather than EKF fusion.
