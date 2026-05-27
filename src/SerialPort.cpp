#include "SerialPort.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#ifdef __linux__
#include <asm/termbits.h>
#include <sys/ioctl.h>
#endif

#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>
#endif

namespace might_mavlink {

namespace {

speed_t toSpeedT(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B500000
        case 500000: return B500000;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
#ifdef B1000000
        case 1000000: return B1000000;
#endif
        default: return 0;
    }
}

bool applyTermios(int fd, int baud) {
    struct termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
        std::fprintf(stderr, "[SerialPort] tcgetattr: %s\n", std::strerror(errno));
        return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag &= ~(PARENB | CSTOPB);
    tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    speed_t s = toSpeedT(baud);
    if (s != 0) {
        cfsetispeed(&tio, s);
        cfsetospeed(&tio, s);
    }

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        std::fprintf(stderr, "[SerialPort] tcsetattr: %s\n", std::strerror(errno));
        return false;
    }

    // For non-standard rates (or to set exact rate on Linux), use BOTHER via termios2.
#ifdef __linux__
    if (s == 0) {
        struct termios2 t2{};
        if (ioctl(fd, TCGETS2, &t2) == 0) {
            t2.c_cflag &= ~CBAUD;
            t2.c_cflag |= BOTHER;
            t2.c_ispeed = baud;
            t2.c_ospeed = baud;
            if (ioctl(fd, TCSETS2, &t2) != 0) {
                std::fprintf(stderr, "[SerialPort] TCSETS2 (%d baud): %s\n", baud, std::strerror(errno));
                return false;
            }
        }
    }
#elif defined(__APPLE__)
    if (s == 0) {
        // macOS lacks B* constants above 230400; use IOSSIOSPEED for arbitrary rates.
        // Pseudo-terminals (e.g. /dev/ttysNNN created by openpty/socat) don't
        // implement this ioctl -- baud is meaningless on a software pipe -- so
        // accept the failure silently when the fd points at a PTY.
        speed_t speed = baud;
        if (ioctl(fd, IOSSIOSPEED, &speed) != 0) {
            char name[64] = {0};
            const bool is_pty = (ttyname_r(fd, name, sizeof(name)) == 0) &&
                                std::strncmp(name, "/dev/ttys", 9) == 0;
            if (!is_pty) {
                std::fprintf(stderr, "[SerialPort] IOSSIOSPEED (%d baud) on %s: %s\n",
                             baud, name[0] ? name : "?", std::strerror(errno));
                return false;
            }
        }
    }
#else
    if (s == 0) {
        std::fprintf(stderr, "[SerialPort] baud %d not supported on this platform\n", baud);
        return false;
    }
#endif

    return true;
}

}  // namespace

SerialPort::SerialPort() = default;
SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& path, int baud) {
    close();

    if (path.rfind("fd:", 0) == 0) {
        fd_ = std::atoi(path.c_str() + 3);
        owns_fd_ = false;
        if (fd_ < 0) {
            std::fprintf(stderr, "[SerialPort] invalid fd in path: %s\n", path.c_str());
            return false;
        }
        int flags = fcntl(fd_, F_GETFL, 0);
        if (flags >= 0) fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
        // If it's a tty, apply termios. Otherwise (pipe/socket) just leave it.
        if (isatty(fd_)) applyTermios(fd_, baud);
        return true;
    }

    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::fprintf(stderr, "[SerialPort] open(%s): %s\n", path.c_str(), std::strerror(errno));
        return false;
    }
    owns_fd_ = true;

    if (!applyTermios(fd_, baud)) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0 && owns_fd_) ::close(fd_);
    fd_ = -1;
    owns_fd_ = true;
}

int SerialPort::read(uint8_t* buf, size_t len) {
    if (fd_ < 0) return -1;
    ssize_t n = ::read(fd_, buf, len);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return -1;
    }
    return static_cast<int>(n);
}

bool SerialPort::writeAll(const uint8_t* buf, size_t len) {
    if (fd_ < 0) return false;
    size_t written = 0;
    while (written < len) {
        ssize_t n = ::write(fd_, buf + written, len - written);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            return false;
        }
        written += static_cast<size_t>(n);
    }
    return true;
}

}  // namespace might_mavlink
