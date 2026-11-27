#pragma once
#ifndef LQS_POSIX_SERIAL_HPP
#define LQS_POSIX_SERIAL_HPP
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace lqs {
struct JointKey {
    uint8_t bus;
    uint16_t id;
    bool operator<(const JointKey& o) const {
        return (bus != o.bus) ? (bus < o.bus) : (id < o.id);
    }
};

class PosixSerial {
public:
    PosixSerial() = default;
    ~PosixSerial() { Close(); }
    bool Open(const std::string& port, int baudrate = 921600) {
        Close();
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) return false;
        struct termios tty{};
        if (tcgetattr(fd_, &tty) != 0) { Close(); return false; }
        speed_t speed = (baudrate == 2000000) ? B2000000 : B921600;
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
        tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) { Close(); return false; }
        tcflush(fd_, TCIOFLUSH);
        return true;
    }
    void Close() { if (fd_ >= 0) { close(fd_); fd_ = -1; } }
    bool IsOpen() const { return fd_ >= 0; }
    static bool IsTerminalErrno(int err) {
        return err == EIO || err == EPIPE || err == ENODEV || err == ENXIO || err == EBADF;
    }
    bool HasHangup() const {
        if (fd_ < 0) return true;
        pollfd port{fd_, 0, 0};
        int rc;
        do {
            rc = poll(&port, 1, 0);
        } while (rc < 0 && errno == EINTR);
        return rc == 1 && (port.revents & (POLLHUP | POLLERR | POLLNVAL));
    }
    bool WaitReadableNs(int64_t timeout_ns, bool& hangup) const {
        hangup = false;
        if (fd_ < 0) { hangup = true; return false; }
        if (timeout_ns < 0) timeout_ns = 0;
        pollfd port{fd_, POLLIN, 0};
        timespec ts{timeout_ns / 1000000000LL, timeout_ns % 1000000000LL};
        int rc;
        do {
            rc = ppoll(&port, 1, &ts, nullptr);
        } while (rc < 0 && errno == EINTR);
        if (rc <= 0) return false;
        if (port.revents & (POLLHUP | POLLERR | POLLNVAL)) hangup = true;
        return (port.revents & POLLIN) != 0;
    }
    // 非阻塞串口上的有界完整写入：处理 EINTR/EAGAIN 与部分写，避免把协议帧腰斩。
    int WriteAll(const void* data, size_t size, int64_t initial_wait_ns, int64_t finish_wait_ns,
                 bool& terminal_error) {
        terminal_error = false;
        if (fd_ < 0) { terminal_error = true; return -1; }
        const auto* bytes = static_cast<const uint8_t*>(data);
        size_t sent = 0;
        const auto start = std::chrono::steady_clock::now();
        while (sent < size) {
            const ssize_t n = write(fd_, bytes + sent, size - sent);
            if (n > 0) {
                sent += static_cast<size_t>(n);
                continue;
            }
            if (n < 0 && errno == EINTR) continue;
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                const auto budget_ns = sent ? finish_wait_ns : initial_wait_ns;
                const auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now() - start).count();
                const int64_t rem_ns = budget_ns - elapsed_ns;
                if (rem_ns <= 0) break;
                pollfd port{fd_, POLLOUT, 0};
                timespec ts{rem_ns / 1000000000LL, rem_ns % 1000000000LL};
                int rc;
                do {
                    rc = ppoll(&port, 1, &ts, nullptr);
                } while (rc < 0 && errno == EINTR);
                if (rc < 0) {
                    if (IsTerminalErrno(errno)) terminal_error = true;
                    return -1;
                }
                if (rc == 0) break;
                if (port.revents & (POLLHUP | POLLERR | POLLNVAL)) {
                    terminal_error = true;
                    return -1;
                }
                continue;
            }
            if (n < 0 && IsTerminalErrno(errno)) terminal_error = true;
            return -1;
        }
        return static_cast<int>(sent);
    }
    int Write(const void* data, size_t size) { return fd_ < 0 ? -1 : write(fd_, data, size); }
    int Read(void* buffer, size_t max_size) { return fd_ < 0 ? -1 : read(fd_, buffer, max_size); }
private:
    int fd_ = -1;
};
}  // namespace lqs
#endif  // LQS_POSIX_SERIAL_HPP
