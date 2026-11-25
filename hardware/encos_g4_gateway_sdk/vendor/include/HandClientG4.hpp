#pragma once
#include "GatewayProtocol.hpp"
#include "PosixSerial.hpp"
#include <array>
#include <thread>
#include <mutex>
#include <atomic>
#include <map>
#include <chrono>
#include <utility>
#include <cerrno>

namespace lqs {
struct JointKey {
    uint8_t bus;
    uint16_t id;
    bool operator<(const JointKey& o) const {
        return (bus != o.bus) ? (bus < o.bus) : (id < o.id);
    }
};

class HandClientG4 {
public:
    HandClientG4() { ResetDefaultBuses(); }
    ~HandClientG4() { Disconnect(); }

    static constexpr size_t kMaxAxes = 16;

    uint8_t BusForId(uint16_t id) const {
        if (id >= 1 && id <= kMaxAxes) return bus_for_id_[id - 1].load(std::memory_order_relaxed);
        return 1;
    }

    size_t AxisCount() const { return axis_count_; }

    bool Connect(const std::string& port = "/dev/ttyUSB0", int baud = 921600, unsigned hz = 200,
                 bool query_only = false, bool wait_for_first_packets = true, size_t axis_count = 12) {
        Disconnect();
        if (!serial_.Open(port, baud)) return false;
        axis_count_ = (axis_count >= 1 && axis_count <= kMaxAxes) ? axis_count : 12;
        ResetDefaultBuses();
        transport_lost_ = false;
        last_reply_us_ = 0;
        query_only_ = query_only;
        target_hz_ = hz;
        running_ = true;
        io_thread_ = std::thread(&HandClientG4::IoLoop, this);
        if (wait_for_first_packets) std::this_thread::sleep_for(std::chrono::milliseconds(300));
        return true;
    }

    void Disconnect() {
        if (running_) {
            running_ = false;
            if (io_thread_.joinable()) io_thread_.join();
            if (!query_only_ && !transport_lost_) SendReleaseBatchAll();
        }
        serial_.Close();
    }

    bool TransportLost() const { return transport_lost_.load(); }
    uint64_t LastReplyUs() const { return last_reply_us_.load(std::memory_order_relaxed); }

    void SetServoCommand(uint8_t bus, uint16_t id, float deg, float spd = 50.0f, float cur = 1.0f) {
        MotorCommand cmd;
        cmd.enabled = true; cmd.is_mit_mode = false;
        cmd.target_angle = deg; cmd.servo_spd = spd; cmd.servo_cur = cur;
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        commands_[{bus, id}] = cmd;
    }

    void SetMitCommand(uint8_t bus, uint16_t id, float deg, float kp, float kd, float vel = 0.0f, float tor = 0.0f) {
        MotorCommand cmd;
        cmd.enabled = true; cmd.is_mit_mode = true;
        cmd.target_angle = deg; cmd.mit_kp = kp; cmd.mit_kd = kd; cmd.mit_vel = vel; cmd.mit_tor = tor;
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        commands_[{bus, id}] = cmd;
    }

    template<size_t N>
    void SetCommandBatch(const std::array<std::pair<JointKey, MotorCommand>, N>& batch) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        for (const auto& [key, command] : batch) commands_[key] = command;
    }

    bool SetCommandBatchBefore(const std::pair<JointKey, MotorCommand>* batch, size_t count, int64_t until_ns) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        if (std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() >= until_ns) return false;
        for (size_t i = 0; i < count; ++i) commands_[batch[i].first] = batch[i].second;
        return true;
    }

    template<size_t N>
    bool SetCommandBatchBefore(const std::array<std::pair<JointKey, MotorCommand>, N>& batch, int64_t until_ns) {
        return SetCommandBatchBefore(batch.data(), N, until_ns);
    }

    void DisableJoint(uint8_t bus, uint16_t id) {
        MotorCommand cmd;
        cmd.enabled = false;
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        commands_[{bus, id}] = cmd;
    }

    std::map<JointKey, HandSnapshot> GetSnapshots() {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        return latest_snapshots_;
    }

    // 项目扩展：时间来自RouteState::Accept解析位置反馈时，不在读取快照时更新。
    std::map<JointKey, TimedHandSnapshot> GetTimedSnapshots() {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        return latest_timed_snapshots_;
    }

private:
    PosixSerial serial_;
    bool query_only_ = false;
    std::atomic<bool> running_{false};
    std::atomic<bool> transport_lost_{false};
    std::atomic<uint64_t> last_reply_us_{0};
    unsigned target_hz_ = 200;
    std::thread io_thread_;
    std::mutex cmd_mutex_;
    std::map<JointKey, MotorCommand> commands_;
    std::mutex snapshot_mutex_;
    std::map<JointKey, HandSnapshot> latest_snapshots_;
    std::map<JointKey, TimedHandSnapshot> latest_timed_snapshots_;
    std::array<std::atomic<uint8_t>, kMaxAxes> bus_for_id_{};
    size_t axis_count_ = 12;

    void ResetDefaultBuses() {
        for (uint16_t id = 1; id <= 12; ++id) {
            bus_for_id_[id - 1].store(static_cast<uint8_t>(3 - (id - 1) / 4), std::memory_order_relaxed);
        }
        for (uint16_t id = 13; id <= kMaxAxes; ++id) {
            bus_for_id_[id - 1].store(static_cast<uint8_t>(4), std::memory_order_relaxed);
        }
    }

    void SendReleaseBatchAll() {
        std::array<GatewayWireFrame, kMaxAxes> rel_batch;
        const size_t n = std::min(axis_count_, kMaxAxes);
        for (size_t i = 0; i < n; ++i) {
            const uint16_t id = static_cast<uint16_t>(i + 1);
            rel_batch[i] = gateway::Release(BusForId(id), id);
            gateway::Seal(rel_batch[i]);
        }

        const size_t bytes = n * sizeof(GatewayWireFrame);
        for (int r = 0; r < 3; ++r) {
            bool term_err = false;
            serial_.WriteAll(rel_batch.data(), bytes, 5'000'000, 10'000'000, term_err);
            if (term_err) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void IoLoop() {
        gateway::Parser parser;
        uint8_t rx[1024];
        std::array<uint8_t, sizeof(GatewayWireFrame)> pending_tail{};
        size_t tail_len = 0;
        std::map<JointKey, gateway::RouteState> routes;
        const size_t active_axes = std::min(axis_count_, kMaxAxes);
        const uint8_t max_bus = active_axes > 12 ? 4 : 3;
        auto add_route = [&](uint8_t b, uint16_t id) { routes.emplace(JointKey{b, id}, gateway::RouteState(b, id)); };
        for (uint16_t id = 1; id <= active_axes; ++id) add_route(BusForId(id), id);

        const auto cycle = std::chrono::microseconds(1000000 / target_hz_);
        uint16_t sequence = 0;
        uint32_t tick_count = 0;

        auto publish_snapshots = [&]() {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            // Read/Accept之后取时间，避免age_ms减去未来收包时间而无符号下溢。
            const auto snapshot_us = gateway::NowUs();
            latest_snapshots_.clear();
            latest_timed_snapshots_.clear();
            for (const auto& [k, route] : routes) {
                auto timed = route.TimedSnapshot(snapshot_us);
                latest_snapshots_[k] = timed.state;
                latest_timed_snapshots_[k] = timed;
            }
        };

        auto drain_rx = [&](bool& published) -> bool {
            bool got_frame = false;
            while (running_) {
                const int n = serial_.Read(rx, sizeof(rx));
                if (n > 0) {
                    parser.Feed(rx, static_cast<size_t>(n), [&](const GatewayWireFrame& f) {
                        if (f.kind != 0x90 || f.bus < 1 || f.bus > max_bus || f.can_id < 1 || f.can_id > active_axes) return;
                        JointKey k{f.bus, f.can_id};
                        const auto now = gateway::NowUs();
                        auto it = routes.find(k);
                        if (it == routes.end()) {
                            // 动态发现新接线：若同 ID 在旧总线离线则迁移路由，若双总线同时在线则保留以触发重号告警。
                            for (auto old_it = routes.begin(); old_it != routes.end(); ) {
                                if (old_it->first.id == f.can_id) {
                                    const bool old_online = (old_it->second.feedback.valid & gateway::Position) &&
                                        now >= old_it->second.feedback.position_time &&
                                        now - old_it->second.feedback.position_time <= 100'000;
                                    if (!old_online) {
                                        std::lock_guard<std::mutex> lock(cmd_mutex_);
                                        commands_.erase(old_it->first);
                                        old_it = routes.erase(old_it);
                                        continue;
                                    }
                                }
                                ++old_it;
                            }
                            it = routes.emplace(k, gateway::RouteState(f.bus, f.can_id)).first;
                            bus_for_id_[f.can_id - 1].store(f.bus, std::memory_order_relaxed);
                        }
                        if (it->second.Accept(f, now)) {
                            last_reply_us_.store(now, std::memory_order_relaxed);
                            got_frame = true;
                        }
                    });
                    continue;
                }
                if (n < 0 && errno == EINTR) continue;
                if ((n < 0 && PosixSerial::IsTerminalErrno(errno)) || serial_.HasHangup()) {
                    transport_lost_ = true;
                    return false;
                }
                break;
            }
            if (got_frame) {
                publish_snapshots();
                published = true;
            }
            return true;
        };

        while (running_) {
            const auto t0 = std::chrono::steady_clock::now();
            const auto deadline = t0 + cycle;

            bool term_err = false;
            if (tail_len > 0) {
                const int w = serial_.WriteAll(pending_tail.data(), tail_len, 1'500'000, 3'500'000, term_err);
                if (term_err) {
                    transport_lost_ = true;
                    break;
                }
                if (w > 0) {
                    const size_t sent = static_cast<size_t>(w);
                    if (sent < tail_len) {
                        std::memmove(pending_tail.data(), pending_tail.data() + sent, tail_len - sent);
                        tail_len -= sent;
                    } else {
                        tail_len = 0;
                    }
                }
            }
            if (tail_len == 0) {
                std::array<GatewayWireFrame, kMaxAxes> tx_batch;
                size_t tx_count = 0;
                const auto now_us = gateway::NowUs();
                const uint8_t probe_step = static_cast<uint8_t>((tick_count / 2) % max_bus);
                ++tick_count;
                {
                    std::lock_guard<std::mutex> lock(cmd_mutex_);
                    for (auto& [k, route] : routes) {
                        if (tx_count >= tx_batch.size()) break;
                        MotorCommand c;
                        if (!query_only_ && commands_.count(k)) c = commands_[k];
                        uint8_t tx_bus = k.bus;
                        const bool online = (route.feedback.valid & gateway::Position) &&
                            now_us >= route.feedback.position_time &&
                            now_us - route.feedback.position_time <= 100'000;
                        // 离线且无有效控制命令的轴按 10 ms 节拍轮询 CAN 总线（12 轴 1..3，16 轴 1..4），自动发现真实挂载总线。
                        if (!c.enabled && !online && probe_step > 0) {
                            tx_bus = static_cast<uint8_t>(((k.bus - 1 + probe_step) % max_bus) + 1);
                        }
                        auto f = gateway::CommandFrame(tx_bus, k.id, c);
                        f.sequence = ++sequence;
                        gateway::Seal(f);
                        tx_batch[tx_count++] = f;
                    }
                }
                const size_t total = tx_count * sizeof(GatewayWireFrame);
                const int w = serial_.WriteAll(tx_batch.data(), total, 1'500'000, 3'500'000, term_err);
                if (term_err) {
                    transport_lost_ = true;
                    break;
                }
                if (w > 0 && static_cast<size_t>(w) < total) {
                    // 仅保留已发出半帧的尾部字节，确保下次写入保持 32 字节帧边界且不积压旧命令。
                    const size_t sent = static_cast<size_t>(w);
                    const size_t mod = sent % sizeof(GatewayWireFrame);
                    if (mod != 0) {
                        tail_len = sizeof(GatewayWireFrame) - mod;
                        std::memcpy(pending_tail.data(),
                                    reinterpret_cast<const uint8_t*>(tx_batch.data()) + sent, tail_len);
                    }
                }
            }

            bool published = false;
            if (!drain_rx(published)) break;

            while (running_) {
                const auto now = std::chrono::steady_clock::now();
                if (now >= deadline) break;
                const auto rem_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now).count();
                const int64_t wait_ns = std::min<int64_t>(rem_ns, 2'000'000);
                bool hangup = false;
                const bool readable = serial_.WaitReadableNs(wait_ns, hangup);
                if (readable) {
                    if (!drain_rx(published)) break;
                } else if (hangup) {
                    transport_lost_ = true;
                    break;
                }
            }
            if (transport_lost_.load()) break;
            if (!published) publish_snapshots();
        }
    }
};
}
