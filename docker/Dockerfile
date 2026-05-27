# ArduPilot SITL for testing MightMavlink end-to-end.
# Builds native to the host arch (arm64 on Apple Silicon, amd64 elsewhere).
#
# Layout once running:
#   container          host (Mac)
#   ---------          ----------
#   SITL SERIAL2 (TCP 5763) <--> 127.0.0.1:5763  (socat → /tmp/board_pty → board_mock)
#   SITL MAVProxy out (UDP) -->  host.docker.internal:14550 (QGroundControl)
#   SITL stdin (MAVProxy console) attached to your terminal

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive TZ=UTC

RUN apt-get update && apt-get install -y \
    sudo git python3 python3-pip lsb-release tzdata locales curl ca-certificates \
 && locale-gen en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ArduPilot's prereq script refuses to run as root.
RUN useradd -m -s /bin/bash ap && echo "ap ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
USER ap
WORKDIR /home/ap

# Shallow + recursive clone keeps the image lean.
RUN git clone --recursive --depth 1 https://github.com/ArduPilot/ardupilot.git
WORKDIR /home/ap/ardupilot

# Installs build deps, MAVProxy, pymavlink, sets up env in ~/.profile.
# USER env must be set explicitly: Docker's non-login shell leaves it empty,
# which breaks the script's `usermod -a -G dialout $USER` step.
RUN USER=ap ./Tools/environment_install/install-prereqs-ubuntu.sh -y

ENV PATH=/home/ap/.local/bin:/usr/lib/ccache:/home/ap/ardupilot/Tools/autotest:$PATH

# Build ArduCopter SITL once. This is the slow layer (~5-10 min on M2).
RUN bash -lc './waf configure --board sitl && ./waf copter'

# Pre-load params so SERIAL2 is MAVLink2 @ 921k from boot, and ArduPilot doesn't
# try to fuse incoming ODOMETRY into its EKF (VISO_TYPE=0 → just forward to GCS).
RUN printf 'SERIAL2_PROTOCOL 2\nSERIAL2_BAUD 921\nVISO_TYPE 0\n' > /home/ap/defaults.parm

EXPOSE 5760 5762 5763 14550/udp

WORKDIR /home/ap/ardupilot/ArduCopter
ENTRYPOINT ["bash", "-lc"]
# `-A --serial2=tcp:3` binds SITL's SERIAL2 to TCP port 5763 (base_port 5760 + 3).
CMD ["sim_vehicle.py -v ArduCopter -w --no-rebuild --add-param-file=/home/ap/defaults.parm --out=udp:host.docker.internal:14550 -A '--serial2=tcp:3'"]
