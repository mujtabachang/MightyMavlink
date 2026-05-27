#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace might_mavlink {

struct Pose {
    // Position in local-NED-like frame, meters. Origin is wherever your VIO started.
    float x, y, z;
    // Orientation quaternion, Hamilton convention (w, x, y, z).
    float qw, qx, qy, qz;
};

struct Twist {
    // Linear velocity, m/s, expressed in body (FRD) frame.
    float vx, vy, vz;
    // Angular velocity, rad/s, body frame.
    float rollspeed, pitchspeed, yawspeed;
};

// Upper-triangle row-major 6x6 covariance, 21 floats. Order per MAVLink ODOMETRY:
// (x, y, z, roll, pitch, yaw) for pose_cov; (vx, vy, vz, rs, ps, ys) for vel_cov.
// Pass an all-NaN array if you don't want to report covariance.
using Cov21 = std::array<float, 21>;

using Handler = std::function<void()>;

class MightMavlink {
public:
    // sysid should match your autopilot's so ArduPilot routes us as a companion component.
    // compid 197 = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY.
    explicit MightMavlink(uint8_t sysid = 1, uint8_t compid = 197);
    ~MightMavlink();

    MightMavlink(const MightMavlink&) = delete;
    MightMavlink& operator=(const MightMavlink&) = delete;

    bool open(const std::string& port, int baud);
    void close();
    bool isOpen() const;

    // Drives RX parsing and emits HEARTBEAT at ~1 Hz. Call frequently from your main loop.
    void poll();

    void sendOdometry(uint64_t time_usec,
                      const Pose& pose, const Twist& twist,
                      const Cov21& pose_cov, const Cov21& vel_cov,
                      uint8_t quality = 0);

    // Updates HEARTBEAT custom_mode and emits a STATUSTEXT on state change.
    void setRecording(bool recording);

    void onStart(Handler h);
    void onStop(Handler h);
    void onReset(Handler h);
    void onStartRecording(Handler h);
    void onStopRecording(Handler h);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace might_mavlink
