#pragma once

#include "g4_api_12dof.h"
#include "g4_api_16dof.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace ve33 {

// 并列支持 12-DOF (encos_g4_gateway_12dof, 200 Hz, CAN 1–3) 与 16-DOF (encos_g4_gateway_16dof, 150 Hz, CAN 1–3)。
// 编译期按 Dof 分派到对应独立驱动 C ABI，12-DOF 与 16-DOF 驱动零交叉耦合。
template <std::size_t Dof = 12>
class GatewayTransportT {
public:
    static_assert(Dof == 12 || Dof == 16, "GatewayTransportT supports 12 or 16 DOF");
    static constexpr std::size_t dof = Dof;
    static constexpr unsigned default_hz = (Dof == 16) ? 150u : 200u;

    enum class Event { none, lost, restored };

    explicit GatewayTransportT(const std::string& port) : port_(port) {
        if (!port_.empty() && !(device_ = open_driver(port_.c_str())))
            throw std::runtime_error(Dof == 16 ? "无法打开 16-DOF G4 网关" : "无法打开 12-DOF G4 网关");
    }
    GatewayTransportT(const GatewayTransportT&) = delete;
    GatewayTransportT& operator=(const GatewayTransportT&) = delete;
    ~GatewayTransportT() { close(); }

    void* handle() const { return device_; }
    unsigned reconnects() const { return reconnects_; }
    static constexpr int axis_count() { return int(Dof); }

    Event poll(int64_t now) {
        if (closed_ || port_.empty()) return Event::none;
        if (device_) {
            if (!is_transport_lost(device_)) return Event::none;
            close_driver(device_);
            device_ = nullptr;
            delay_ = initial_delay;
            retry_at_ = now + delay_;
            return Event::lost;
        }
        if (now < retry_at_) return Event::none;
        if (!(device_ = open_fresh_driver(port_.c_str()))) {
            delay_ = std::min(delay_ * 2, max_delay);
            retry_at_ = now + delay_;
            return Event::none;
        }
        ++reconnects_;
        return Event::restored;
    }

    void close() {
        closed_ = true;
        if (device_) close_driver(device_);
        device_ = nullptr;
    }

    void set_disabled_mask(uint32_t mask) {
        if (!device_) return;
        if constexpr (Dof == 16) g4_16dof_set_disabled_mask(device_, mask);
        else g4_12dof_set_disabled_mask(device_, mask);
    }

    void release(uint32_t bus, uint32_t id) {
        if (!device_) return;
        if constexpr (Dof == 16) g4_16dof_release(device_, bus, id);
        else g4_12dof_release(device_, bus, id);
    }

    int snapshots_with_current_time(G4Joint* out, int64_t* current_times, int capacity) const {
        if (!device_) return 0;
        if constexpr (Dof == 16) return g4_16dof_snapshots_with_current_time(device_, out, current_times, capacity);
        else return g4_12dof_snapshots_with_current_time(device_, out, current_times, capacity);
    }

    int renew_mit_batch(const float* commands, int count, int64_t deadline_ns, int64_t previous_deadline_ns) {
        if (!device_) return -1;
        if constexpr (Dof == 16) return g4_16dof_renew_mit_batch(device_, commands, count, deadline_ns, previous_deadline_ns);
        else return g4_12dof_renew_mit_batch(device_, commands, count, deadline_ns, previous_deadline_ns);
    }

    int homing_mit_batch(const float* commands, int count, uint32_t active_mask, int64_t deadline_ns) {
        if (!device_) return -1;
        if constexpr (Dof == 16) return g4_16dof_homing_mit_batch(device_, commands, count, active_mask, deadline_ns);
        else return g4_12dof_homing_mit_batch(device_, commands, count, active_mask, deadline_ns);
    }

private:
    static void* open_driver(const char* port) {
        if constexpr (Dof == 16) return g4_16dof_open(port, 921600, default_hz);
        else return g4_12dof_open(port, 921600, default_hz);
    }
    static void* open_fresh_driver(const char* port) {
        if constexpr (Dof == 16) return g4_16dof_open_fresh(port, 921600, default_hz);
        else return g4_12dof_open_fresh(port, 921600, default_hz);
    }
    static int is_transport_lost(void* dev) {
        if constexpr (Dof == 16) return g4_16dof_transport_lost(dev);
        else return g4_12dof_transport_lost(dev);
    }
    static void close_driver(void* dev) {
        if constexpr (Dof == 16) g4_16dof_close(dev);
        else g4_12dof_close(dev);
    }

    const std::string port_;
    void* device_ = nullptr;
    static constexpr int64_t initial_delay = 250'000'000, max_delay = 2'000'000'000;
    int64_t retry_at_ = 0, delay_ = initial_delay;
    unsigned reconnects_ = 0;
    bool closed_ = false;
};

using GatewayTransport = GatewayTransportT<12>;
using GatewayTransport12 = GatewayTransportT<12>;
using GatewayTransport16 = GatewayTransportT<16>;
}  // namespace ve33
