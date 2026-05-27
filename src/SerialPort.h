#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace might_mavlink {

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Opens `path` at `baud`, 8N1, raw mode, non-blocking reads.
    // Accepts an already-open fd (e.g. a pty master) when path begins with "fd:" -- e.g. "fd:7".
    bool open(const std::string& path, int baud);
    void close();
    bool isOpen() const { return fd_ >= 0; }
    int fd() const { return fd_; }

    // Non-blocking. Returns bytes read (>=0) or -1 on error. 0 means no data ready.
    int read(uint8_t* buf, size_t len);

    // Blocks until all bytes written or error. Returns true on success.
    bool writeAll(const uint8_t* buf, size_t len);

private:
    int fd_ = -1;
    bool owns_fd_ = true;
};

}  // namespace might_mavlink
