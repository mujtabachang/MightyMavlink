#!/usr/bin/env bash
# Runs the SITL container with the right port + host wiring for the wrapper test.
# Usage:  docker/run.sh                  # default: ArduCopter on UDP 14550 + TCP 5763
#         docker/run.sh -v ArduPlane     # override the sim_vehicle.py args
set -euo pipefail

IMG=mightmavlink-sitl

if ! docker image inspect "$IMG" >/dev/null 2>&1; then
  echo "image '$IMG' not found — building (one-time, ~5-10 min)..."
  docker build -t "$IMG" "$(dirname "$0")"
fi

# 5763  -> SITL SERIAL2 (where board_mock connects via socat)
# 5760  -> SITL SERIAL0 console (handy for extra mavlink tooling)
# host.docker.internal -> Mac, so SITL can push MAVProxy out to QGC on UDP 14550
exec docker run -it --rm \
  -p 5760:5760 -p 5763:5763 \
  --add-host=host.docker.internal:host-gateway \
  --name mightmavlink-sitl \
  "$IMG" "$@"
