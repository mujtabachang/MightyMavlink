// End-to-end smoke test with no hardware: open a PTY pair, run MightMavlink on
// the slave side, drive raw MAVLink bytes on the master side.
//
//   - Verifies HEARTBEAT is emitted within 1.5s
//   - Verifies ODOMETRY round-trips byte-for-byte through the parser
//   - Verifies COMMAND_LONG with each MAV_CMD_USER_n triggers its handler and ACKs

#include "might_mavlink/MightMavlink.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <thread>
#include <unistd.h>
#if defined(__APPLE__)
#include <util.h>   // openpty
#else
#include <pty.h>    // openpty on glibc/musl Linux (needs -lutil at link)
#endif

#define MAVLINK_HELPER static inline
#include <mavlink.h>

namespace {

constexpr uint8_t kBoardSys = 1;
constexpr uint8_t kBoardComp = 197;
constexpr uint8_t kGcsSys = 255;
constexpr uint8_t kGcsComp = 190;

bool writeAll(int fd, const uint8_t* buf, size_t len) {
    size_t w = 0;
    while (w < len) {
        ssize_t n = ::write(fd, buf + w, len - w);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        w += static_cast<size_t>(n);
    }
    return true;
}

void sendCmd(int fd, uint16_t cmd) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        kGcsSys, kGcsComp, &msg,
        kBoardSys, kBoardComp, cmd,
        /*confirmation*/ 0,
        0, 0, 0, 0, 0, 0, 0);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
    if (!writeAll(fd, buf, n)) std::fprintf(stderr, "writeAll failed\n");
}

struct ParsedCounts {
    int heartbeat = 0;
    int odometry = 0;
    int acks_accepted = 0;
};

// Drain master_fd for `dur`, parse incoming, return counts.
ParsedCounts drainParse(int master_fd, std::chrono::milliseconds dur) {
    ParsedCounts c{};
    mavlink_status_t st{};
    mavlink_message_t msg{};
    auto deadline = std::chrono::steady_clock::now() + dur;
    uint8_t buf[256];

    while (std::chrono::steady_clock::now() < deadline) {
        ssize_t n = ::read(master_fd, buf, sizeof(buf));
        if (n > 0) {
            for (ssize_t i = 0; i < n; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_1, buf[i], &msg, &st)) {
                    switch (msg.msgid) {
                        case MAVLINK_MSG_ID_HEARTBEAT: c.heartbeat++; break;
                        case MAVLINK_MSG_ID_ODOMETRY:  c.odometry++;  break;
                        case MAVLINK_MSG_ID_COMMAND_ACK: {
                            mavlink_command_ack_t a;
                            mavlink_msg_command_ack_decode(&msg, &a);
                            if (a.result == MAV_RESULT_ACCEPTED) c.acks_accepted++;
                            break;
                        }
                        default: break;
                    }
                }
            }
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return c;
}

int fails = 0;
void check(bool ok, const char* what) {
    std::printf("  %s %s\n", ok ? "[ok]  " : "[FAIL]", what);
    if (!ok) ++fails;
}

}  // namespace

int main() {
    int master_fd = -1, slave_fd = -1;
    char slave_name[128] = {0};
    if (openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr) != 0) {
        std::perror("openpty");
        return 1;
    }
    int flags = fcntl(master_fd, F_GETFL, 0);
    fcntl(master_fd, F_SETFL, flags | O_NONBLOCK);
    std::printf("pty master=%d slave=%s\n", master_fd, slave_name);

    might_mavlink::MightMavlink mav(kBoardSys, kBoardComp);
    // Use the slave fd directly so the library reads/writes the right end.
    if (!mav.open(std::string("fd:") + std::to_string(slave_fd), 115200)) {
        std::fprintf(stderr, "mav.open failed\n");
        return 1;
    }

    std::atomic<int> n_start{0}, n_stop{0}, n_reset{0}, n_srec{0}, n_erec{0};
    mav.onStart         ([&]{ ++n_start; });
    mav.onStop          ([&]{ ++n_stop;  });
    mav.onReset         ([&]{ ++n_reset; });
    mav.onStartRecording([&]{ ++n_srec;  });
    mav.onStopRecording ([&]{ ++n_erec;  });

    std::atomic<bool> run{true};
    std::thread pump([&]{
        while (run) {
            mav.poll();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });

    std::puts("test: heartbeat within 1.5s");
    {
        auto c = drainParse(master_fd, std::chrono::milliseconds(1500));
        check(c.heartbeat >= 1, "received >=1 HEARTBEAT");
    }

    std::puts("test: ODOMETRY transmit");
    {
        might_mavlink::Pose p{1, 2, 3, 1, 0, 0, 0};
        might_mavlink::Twist v{0.1f, 0.2f, 0.3f, 0, 0, 0};
        might_mavlink::Cov21 cov;
        cov.fill(std::nanf(""));
        mav.sendOdometry(123456789, p, v, cov, cov, 50);
        auto c = drainParse(master_fd, std::chrono::milliseconds(300));
        check(c.odometry == 1, "received exactly 1 ODOMETRY");
    }

    std::puts("test: each MAV_CMD_USER_n fires handler and ACKs");
    {
        sendCmd(master_fd, MAV_CMD_USER_1);
        sendCmd(master_fd, MAV_CMD_USER_2);
        sendCmd(master_fd, MAV_CMD_USER_3);
        sendCmd(master_fd, MAV_CMD_USER_4);
        sendCmd(master_fd, MAV_CMD_USER_5);
        auto c = drainParse(master_fd, std::chrono::milliseconds(500));
        check(n_start == 1, "onStart fired");
        check(n_stop  == 1, "onStop fired");
        check(n_reset == 1, "onReset fired");
        check(n_srec  == 1, "onStartRecording fired");
        check(n_erec  == 1, "onStopRecording fired");
        check(c.acks_accepted == 5, "received 5 COMMAND_ACK(ACCEPTED)");
    }

    std::puts("test: setRecording emits STATUSTEXT + heartbeat with custom_mode bit");
    {
        // Drain anything already queued.
        drainParse(master_fd, std::chrono::milliseconds(50));
        mav.setRecording(true);
        // Parse the next batch and look specifically for a heartbeat with custom_mode == 1.
        mavlink_status_t st{};
        mavlink_message_t msg{};
        bool saw_rec_hb = false;
        bool saw_status = false;
        uint8_t buf[256];
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
        while (std::chrono::steady_clock::now() < deadline && !(saw_rec_hb && saw_status)) {
            ssize_t n = ::read(master_fd, buf, sizeof(buf));
            if (n > 0) {
                for (ssize_t i = 0; i < n; ++i) {
                    if (mavlink_parse_char(MAVLINK_COMM_2, buf[i], &msg, &st)) {
                        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            mavlink_heartbeat_t hb;
                            mavlink_msg_heartbeat_decode(&msg, &hb);
                            if (hb.custom_mode & 0x1u) saw_rec_hb = true;
                        } else if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                            saw_status = true;
                        }
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        check(saw_rec_hb, "heartbeat reports recording bit");
        check(saw_status, "STATUSTEXT emitted on recording change");
    }

    run = false;
    pump.join();
    mav.close();
    ::close(master_fd);

    std::printf("\n%s (%d failure%s)\n",
                fails == 0 ? "PASS" : "FAIL",
                fails, fails == 1 ? "" : "s");
    return fails == 0 ? 0 : 1;
}
