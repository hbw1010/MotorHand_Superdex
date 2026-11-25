#pragma once

#include "g4_api.h"
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace ve33 {

// 只允许原生 G4 所有者调用。按串口故障或整条链路持续无回包重开，单轴或物理输入过期仍走控制期限。
class GatewayTransport {
public:
    enum class Event { none, lost, restored };

    explicit GatewayTransport(const std::string& port, int axis_count = 12)
        : port_(port), axis_count_(axis_count) {
        if (!port_.empty() && !(device_ = g4_open_axes(port_.c_str(), 921600, 200, axis_count_)))
            throw std::runtime_error("无法打开 G4 网关");
    }
    GatewayTransport(const GatewayTransport&) = delete;
    GatewayTransport& operator=(const GatewayTransport&) = delete;
    ~GatewayTransport() { close(); }

    void* handle() const { return device_; }
    unsigned reconnects() const { return reconnects_; }
    int axis_count() const { return axis_count_; }

    Event poll(int64_t now) {
        if (closed_ || port_.empty()) return Event::none;
        if (device_) {
            if (!g4_transport_lost(device_)) return Event::none;
            g4_close(device_);
            device_ = nullptr;
            delay_ = initial_delay;
            retry_at_ = now + delay_;
            return Event::lost;
        }
        if (now < retry_at_) return Event::none;
        if (!(device_ = g4_open_fresh_axes(port_.c_str(), 921600, 200, axis_count_))) {
            delay_ = std::min(delay_ * 2, max_delay);
            retry_at_ = now + delay_;
            return Event::none;
        }
        ++reconnects_;
        return Event::restored;
    }

    void close() {
        closed_ = true;
        if (device_) g4_close(device_);
        device_ = nullptr;
    }

private:
    const std::string port_;
    const int axis_count_;
    void* device_ = nullptr;
    static constexpr int64_t initial_delay = 250'000'000, max_delay = 2'000'000'000;
    int64_t retry_at_ = 0, delay_ = initial_delay;
    unsigned reconnects_ = 0;
    bool closed_ = false;
};
} // namespace ve33
