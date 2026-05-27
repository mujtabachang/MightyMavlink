// Pretends to be the VIO board: streams a spiraling pose at 30 Hz, prints
// commands received from GCS. Wire this up to ArduPilot's SERIAL2 (or whatever
// you set SERIALn_PROTOCOL=2 on) and ArduPilot forwards everything to the GCS
// link automatically.
//
// Usage: ./board_mock <serial_port> <baud>
//   e.g. ./board_mock /dev/ttyUSB0 921600
//        ./board_mock /dev/ttys005 115200    (a socat-created PTY for local testing)

#include "might_mavlink/MightMavlink.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace {
std::atomic<bool> g_run{true};
void onSig(int) { g_run = false; }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <serial_port> <baud>\n", argv[0]);
        return 2;
    }
    const std::string port = argv[1];
    const int baud = std::atoi(argv[2]);

    std::signal(SIGINT, onSig);
    std::signal(SIGTERM, onSig);

    might_mavlink::MightMavlink mav;
    if (!mav.open(port, baud)) {
        std::fprintf(stderr, "failed to open %s @ %d\n", port.c_str(), baud);
        return 1;
    }

    std::atomic<bool> running{false};

    mav.onStart([&]{ running = true;  std::puts("[cmd] START"); });
    mav.onStop ([&]{ running = false; std::puts("[cmd] STOP");  });
    mav.onReset([&]{                 std::puts("[cmd] RESET"); });
    mav.onStartRecording([&]{
        std::puts("[cmd] START RECORDING");
        mav.setRecording(true);
    });
    mav.onStopRecording([&]{
        std::puts("[cmd] STOP RECORDING");
        mav.setRecording(false);
    });

    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();
    auto next_tx = t0;

    might_mavlink::Cov21 nan_cov;
    nan_cov.fill(std::nanf(""));

    std::printf("board_mock running on %s @ %d. Ctrl-C to stop.\n", port.c_str(), baud);

    while (g_run) {
        mav.poll();

        auto now = clock::now();
        if (now >= next_tx) {
            next_tx += std::chrono::milliseconds(33);  // ~30 Hz

            if (running) {
                float t = std::chrono::duration<float>(now - t0).count();
                might_mavlink::Pose p{};
                p.x = 2.0f * std::cos(0.5f * t);
                p.y = 2.0f * std::sin(0.5f * t);
                p.z = -1.0f - 0.2f * std::sin(0.3f * t);
                const float yaw = 0.5f * t;
                p.qw = std::cos(yaw * 0.5f);
                p.qx = 0.0f;
                p.qy = 0.0f;
                p.qz = std::sin(yaw * 0.5f);

                might_mavlink::Twist v{};
                v.vx = -1.0f * std::sin(0.5f * t);
                v.vy =  1.0f * std::cos(0.5f * t);
                v.vz = -0.06f * std::cos(0.3f * t);
                v.yawspeed = 0.5f;

                uint64_t t_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                    now.time_since_epoch()).count();
                mav.sendOdometry(t_us, p, v, nan_cov, nan_cov, /*quality=*/100);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    std::puts("\nshutting down");
    return 0;
}
