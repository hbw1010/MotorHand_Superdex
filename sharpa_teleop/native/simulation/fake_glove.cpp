#include "simulation/fake_glove.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>
namespace motorhand::native::simulation {
namespace {
using Clock = std::chrono::steady_clock;
constexpr double pi = 3.14159265358979323846;
std::uint64_t now_ns() { return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count(); }
}

template <std::size_t G>
struct FakeGloveT<G>::Impl {
    const std::array<EncosMotorCalibration, G> cal;
    mutable std::mutex mutex;
    std::mutex close_mutex;
    std::condition_variable changed;
    std::thread producer;
    GloveFrameT<G> frame;
    GloveVec<G> target{}, nominal{};
    std::array<std::uint64_t, G> writes{};
    bool connected{}, closed{}, homed{}, force_enabled{};
    explicit Impl(std::array<EncosMotorCalibration, G> c) : cal(std::move(c)) {
        std::array<bool, G> seen{};
        for (const auto& m : cal) {
            const int raw = m.raw_motor_id - 1;
            if (raw < 0 || raw >= static_cast<int>(G) || seen[raw] || !std::isfinite(m.zero_deg) ||
                !std::isfinite(m.sign * m.scale) || m.sign * m.scale == 0 ||
                !std::isfinite(m.model_min_rad) || !std::isfinite(m.model_max_rad) ||
                m.model_min_rad > 0 || m.model_max_rad < 0 || m.model_min_rad >= m.model_max_rad ||
                (m.has_torque_limit && (!std::isfinite(m.raw_torque_limit_nm) || m.raw_torque_limit_nm <= 0)))
                throw std::invalid_argument("模拟手套校准无效");
            seen[raw] = true;
        }
    }
    void require_connected() const { if (!connected || closed) throw std::runtime_error("模拟手套未连接"); }
    void require_output(bool force) const {
        require_connected();
        if (!homed || (force && !force_enabled)) throw std::runtime_error("模拟手套未取得对应输出权");
    }
    void validate_position(const GloveVec<G>& q) const {
        for (std::size_t i = 0; i < G; ++i)
            if (!std::isfinite(q[i]) || q[i] < cal[i].model_min_rad || q[i] > cal[i].model_max_rad)
                throw std::invalid_argument("模拟手套目标超出标定限位");
    }
    void validate_torque(const GloveVec<G>& value) const {
        for (std::size_t i = 0; i < G; ++i)
            if (!std::isfinite(value[i]) || !cal[i].has_torque_limit ||
                std::abs(value[i]) > cal[i].raw_torque_limit_nm / std::abs(cal[i].sign * cal[i].scale) + 1e-10)
                throw std::invalid_argument("模拟手套名义力矩超出标定限额");
    }
    void produce() {
        std::unique_lock<std::mutex> lock(mutex);
        auto next = Clock::now();
        while (connected) {
            const auto stamp = now_ns();
            frame.motor_rad = target;
            for (std::size_t i = 0; i < G; ++i) {
                frame.raw_degrees[i] = cal[i].zero_deg + target[i] * 180 / pi / (cal[i].sign * cal[i].scale);
                frame.received_ns[i] = stamp;
            }
            frame.oldest_ns = stamp;
            changed.notify_all();
            next += std::chrono::milliseconds(20);
            changed.wait_until(lock, next, [&] { return !connected; });
        }
    }
    void confirm_write(std::uint64_t deadline) {
        const auto stamp = now_ns();
        if (!deadline || stamp >= deadline) throw std::runtime_error("模拟手套原始命令期限已过");
        writes.fill(stamp);
    }
};

template <std::size_t G>
FakeGloveT<G>::FakeGloveT(std::array<EncosMotorCalibration, G> c) : impl_(std::make_unique<Impl>(std::move(c))) {}
template <std::size_t G> FakeGloveT<G>::~FakeGloveT() { close(); }
template <std::size_t G>
void FakeGloveT<G>::connect() {
    auto& s = *impl_; std::unique_lock<std::mutex> lock(s.mutex);
    if (s.connected || s.closed) throw std::runtime_error("模拟手套不能重复连接");
    s.connected = true;
    try { s.producer = std::thread([&s] { s.produce(); }); } catch (...) { s.connected = false; throw; }
    s.changed.wait(lock, [&] { return s.frame.oldest_ns != 0; });
}
template <std::size_t G> GloveFrameT<G> FakeGloveT<G>::read() {
    auto& s = *impl_; std::lock_guard<std::mutex> lock(s.mutex); s.require_connected(); return s.frame;
}
template <std::size_t G> std::array<EncosMotorCalibration, G> FakeGloveT<G>::calibration() const { return impl_->cal; }
template <std::size_t G> std::array<std::uint64_t, G> FakeGloveT<G>::last_command_write_ns() const {
    std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->writes;
}
template <std::size_t G>
void FakeGloveT<G>::set_manual_target(const VecG& q) {
    auto& s = *impl_; std::lock_guard<std::mutex> lock(s.mutex); s.require_connected(); s.validate_position(q); s.target = q;
}
template <std::size_t G>
void FakeGloveT<G>::home() {
    auto& s = *impl_; std::unique_lock<std::mutex> lock(s.mutex); s.require_connected();
    s.target = {}; s.nominal = {}; s.force_enabled = false;
    s.changed.wait(lock, [&] { return !s.connected || s.frame.motor_rad == GloveVec<G>{}; });
    s.require_connected(); s.homed = true;
}
template <std::size_t G>
void FakeGloveT<G>::enable_force() {
    auto& s = *impl_; std::lock_guard<std::mutex> lock(s.mutex); s.require_output(false); s.force_enabled = true;
}
template <std::size_t G>
GloveVec<G> FakeGloveT<G>::write_position(const VecG& q, std::uint64_t deadline) {
    auto& s = *impl_; std::lock_guard<std::mutex> lock(s.mutex); s.require_output(false);
    s.validate_position(q); s.confirm_write(deadline);
    s.target = q; s.force_enabled = false; s.nominal = {}; return q;
}
template <std::size_t G>
GloveVec<G> FakeGloveT<G>::write_torque(const VecG& tau, std::uint64_t deadline) {
    auto& s = *impl_; std::lock_guard<std::mutex> lock(s.mutex); s.require_output(true);
    s.validate_torque(tau); s.confirm_write(deadline); s.nominal = tau; return tau;
}
template <std::size_t G>
DriverImpedanceSubmissionT<G> FakeGloveT<G>::write_impedance(const ImpedanceCmd& c, std::uint64_t deadline) {
    if (!std::isfinite(c.torque_margin) || c.torque_margin <= 0 || c.torque_margin > 1)
        throw std::invalid_argument("模拟手套阻抗预算系数无效");
    auto& s = *impl_; std::lock_guard<std::mutex> lock(s.mutex); s.require_output(true); s.validate_position(c.target_rad);
    DriverImpedanceSubmissionT<G> out; out.command = c; out.estimate_received_ns = s.frame.received_ns;
    for (std::size_t i = 0; i < G; ++i) {
        if (!std::isfinite(c.kp[i]) || c.kp[i] < 0 || !std::isfinite(c.kd[i]) || c.kd[i] < 0 || !std::isfinite(c.velocity_rad_s[i]))
            throw std::invalid_argument("模拟手套阻抗参数无效");
        out.pd_nm[i] = c.kp[i] * (c.target_rad[i] - s.frame.motor_rad[i]) - c.kd[i] * c.velocity_rad_s[i];
        double nominal = out.pd_nm[i] + c.feedforward_nm[i];
        const auto& axis = s.cal[i];
        const double budget = c.torque_margin * axis.raw_torque_limit_nm / std::abs(axis.sign * axis.scale);
        if (axis.has_torque_limit && std::isfinite(nominal) && std::abs(nominal) > budget) {
            const double scale = std::nextafter(budget, 0.) / std::abs(nominal);
            out.command.kp[i] *= scale;
            out.command.kd[i] *= scale;
            out.command.feedforward_nm[i] *= scale;
            out.pd_nm[i] *= scale;
            nominal *= scale;
            out.observation[i].torque_limited = true;
            out.observation[i].add(ImpedanceReason::torque_budget_limited);
        }
        out.friction_nm[i] = out.command.feedforward_nm[i];
        out.nominal_nm[i] = nominal;
    }
    s.validate_torque(out.nominal_nm); s.confirm_write(deadline); s.nominal = out.nominal_nm; return out;
}
template <std::size_t G>
GloveVec<G> FakeGloveT<G>::last_nominal_nm() const {
    std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->nominal;
}
template <std::size_t G>
void FakeGloveT<G>::disable() {
    auto& s = *impl_; std::lock_guard<std::mutex> lock(s.mutex); s.nominal = {}; s.force_enabled = false;
}
template <std::size_t G>
void FakeGloveT<G>::close() {
    auto& s = *impl_; std::lock_guard<std::mutex> close_lock(s.close_mutex);
    { std::lock_guard<std::mutex> lock(s.mutex); s.connected = false; s.closed = true; s.nominal = {}; s.force_enabled = false; }
    s.changed.notify_all(); if (s.producer.joinable()) s.producer.join();
}

template class FakeGloveT<12>;
template class FakeGloveT<16>;
}
