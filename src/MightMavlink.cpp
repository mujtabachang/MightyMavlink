#include "might_mavlink/MightMavlink.h"

#include "SerialPort.h"

#include <chrono>
#include <cstdio>
#include <cstring>

// mavlink_helpers.h emits its functions with external linkage by default, so
// including it in multiple TUs would multiply-define them. Force static-inline
// so each TU gets a private copy. Don't define MAVLINK_USE_CONVENIENCE_FUNCTIONS
// at all -- those helpers depend on a global mavlink_system + comm_send_ch.
#define MAVLINK_HELPER static inline
#include <mavlink.h>

namespace might_mavlink {

namespace {

uint64_t nowUs() {
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

constexpr uint16_t kCmdUserStart          = MAV_CMD_USER_1;
constexpr uint16_t kCmdUserStop           = MAV_CMD_USER_2;
constexpr uint16_t kCmdUserReset          = MAV_CMD_USER_3;
constexpr uint16_t kCmdUserStartRecording = MAV_CMD_USER_4;
constexpr uint16_t kCmdUserStopRecording  = MAV_CMD_USER_5;

constexpr uint32_t kHeartbeatPeriodUs = 1'000'000;

}  // namespace

struct MightMavlink::Impl {
    uint8_t sysid;
    uint8_t compid;
    SerialPort port;

    mavlink_status_t rx_status{};
    mavlink_message_t rx_msg{};

    Handler on_start, on_stop, on_reset, on_start_rec, on_stop_rec;

    bool recording = false;
    uint64_t last_heartbeat_us = 0;

    Impl(uint8_t s, uint8_t c) : sysid(s), compid(c) {}

    void sendMsg(const mavlink_message_t& msg) {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
        port.writeAll(buf, n);
    }

    void sendHeartbeat() {
        mavlink_message_t msg;
        const uint32_t custom_mode = recording ? 0x1u : 0x0u;
        mavlink_msg_heartbeat_pack(
            sysid, compid, &msg,
            MAV_TYPE_ONBOARD_CONTROLLER,
            MAV_AUTOPILOT_INVALID,
            /*base_mode*/ 0,
            custom_mode,
            MAV_STATE_ACTIVE);
        sendMsg(msg);
    }

    void sendStatusText(MAV_SEVERITY sev, const char* text) {
        mavlink_message_t msg;
        char buf[50] = {0};
        std::strncpy(buf, text, sizeof(buf) - 1);
        mavlink_msg_statustext_pack(sysid, compid, &msg, sev, buf, 0, 0);
        sendMsg(msg);
    }

    void sendAck(uint16_t cmd, uint8_t result, uint8_t target_sys, uint8_t target_comp) {
        mavlink_message_t msg;
        mavlink_msg_command_ack_pack(
            sysid, compid, &msg, cmd, result,
            /*progress*/ 0, /*result_param2*/ 0,
            target_sys, target_comp);
        sendMsg(msg);
    }

    void handleCommandLong(const mavlink_message_t& msg) {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(&msg, &cmd);

        // Only act if addressed to us (or broadcast).
        const bool for_us =
            (cmd.target_system == 0 || cmd.target_system == sysid) &&
            (cmd.target_component == 0 || cmd.target_component == compid);
        if (!for_us) return;

        Handler* h = nullptr;
        switch (cmd.command) {
            case kCmdUserStart:          h = &on_start;     break;
            case kCmdUserStop:           h = &on_stop;      break;
            case kCmdUserReset:          h = &on_reset;     break;
            case kCmdUserStartRecording: h = &on_start_rec; break;
            case kCmdUserStopRecording:  h = &on_stop_rec;  break;
            default:
                sendAck(cmd.command, MAV_RESULT_UNSUPPORTED, msg.sysid, msg.compid);
                return;
        }

        if (*h) (*h)();
        sendAck(cmd.command, MAV_RESULT_ACCEPTED, msg.sysid, msg.compid);
    }

    void dispatch(const mavlink_message_t& msg) {
        if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
            handleCommandLong(msg);
        }
        // Other msgids ignored for now.
    }

    void pumpRx() {
        uint8_t buf[256];
        for (;;) {
            int n = port.read(buf, sizeof(buf));
            if (n <= 0) return;
            for (int i = 0; i < n; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &rx_msg, &rx_status)) {
                    dispatch(rx_msg);
                }
            }
        }
    }

    void pumpHeartbeat() {
        uint64_t now = nowUs();
        if (now - last_heartbeat_us >= kHeartbeatPeriodUs) {
            sendHeartbeat();
            last_heartbeat_us = now;
        }
    }
};

MightMavlink::MightMavlink(uint8_t sysid, uint8_t compid)
    : impl_(std::make_unique<Impl>(sysid, compid)) {}

MightMavlink::~MightMavlink() = default;

bool MightMavlink::open(const std::string& port, int baud) {
    return impl_->port.open(port, baud);
}

void MightMavlink::close() { impl_->port.close(); }
bool MightMavlink::isOpen() const { return impl_->port.isOpen(); }

void MightMavlink::poll() {
    if (!impl_->port.isOpen()) return;
    impl_->pumpRx();
    impl_->pumpHeartbeat();
}

void MightMavlink::sendOdometry(uint64_t time_usec,
                                const Pose& pose, const Twist& twist,
                                const Cov21& pose_cov, const Cov21& vel_cov,
                                uint8_t quality) {
    if (!impl_->port.isOpen()) return;

    const float q[4] = {pose.qw, pose.qx, pose.qy, pose.qz};

    mavlink_message_t msg;
    mavlink_msg_odometry_pack(
        impl_->sysid, impl_->compid, &msg,
        time_usec,
        MAV_FRAME_LOCAL_FRD,
        MAV_FRAME_BODY_FRD,
        pose.x, pose.y, pose.z,
        q,
        twist.vx, twist.vy, twist.vz,
        twist.rollspeed, twist.pitchspeed, twist.yawspeed,
        pose_cov.data(), vel_cov.data(),
        /*reset_counter*/ 0,
        MAV_ESTIMATOR_TYPE_VISION,
        quality);
    impl_->sendMsg(msg);
}

void MightMavlink::setRecording(bool recording) {
    if (recording == impl_->recording) return;
    impl_->recording = recording;
    impl_->sendStatusText(MAV_SEVERITY_INFO, recording ? "REC ON" : "REC OFF");
    // Push a fresh heartbeat so GCS sees the new custom_mode immediately.
    impl_->sendHeartbeat();
    impl_->last_heartbeat_us = nowUs();
}

void MightMavlink::onStart(Handler h)          { impl_->on_start     = std::move(h); }
void MightMavlink::onStop(Handler h)           { impl_->on_stop      = std::move(h); }
void MightMavlink::onReset(Handler h)          { impl_->on_reset     = std::move(h); }
void MightMavlink::onStartRecording(Handler h) { impl_->on_start_rec = std::move(h); }
void MightMavlink::onStopRecording(Handler h)  { impl_->on_stop_rec  = std::move(h); }

}  // namespace might_mavlink
