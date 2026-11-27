#pragma once
#include "GatewayProtocol16.hpp"
#include "PosixSerial16.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <map>
#include <mutex>
#include <thread>
#include <utility>

namespace lqs {

// 16-DOF STM32G4 (LQS-GW02 v0.5.0, ±20 N·m MIT) 专属 Linux 实时驱动：
// - 对齐官方 v0.5.0 规范：16 轴 (ID 1..16) 挂载于 CAN 1–3 总线；
//   因 GW02 板端周期轮询表仅支持 12 槽，16 轴采用主机直发 GW_CAN_TX (mode=2)，
//   且遵循 routes*(hz+10)<=2600 带宽上限，自动将目标频率上限钳制在 150 Hz (6667 us)；
// - 完整移植 12-DOF 本地实时优化：
//   O_CLOEXEC + WriteAll + WaitReadableNs(ppoll) + 32B 半帧保护 + TimedHandSnapshot
//   + CAN 1–3 动态路由自动发现 + 双总线重号检测 + SetCommandBatchBefore + 热插拔/断线检测。
class HandClientG4_16dof {
public:
    static constexpr size_t kAxisCount = 16;
    static constexpr size_t kMaxAxes = 16;
    static constexpr uint8_t kMaxBus = 3;
    static constexpr unsigned kDefaultHz = 150;
    static constexpr unsigned kMaxHz = 150;

    HandClientG4_16dof() { ResetDefaultBuses(); }
    ~HandClientG4_16dof() { Disconnect(); }

    uint8_t BusForId(uint16_t id) const {
        if (id >= 1 && id <= kAxisCount) return bus_for_id_[id - 1].load(std::memory_order_relaxed);
        return 1;
    }

    size_t AxisCount() const { return kAxisCount; }
    unsigned TargetHz() const { return target_hz_; }

    bool Connect(const std::string& port = "/dev/ttyUSB0", int baud = 921600, unsigned hz = kDefaultHz,
                 bool query_only = false, bool wait_for_first_packets = true) {
        Disconnect();
        if (!serial_.Open(port, baud)) return false;
        ResetDefaultBuses();
        transport_lost_ = false;
        last_reply_us_ = 0;
        query_only_ = query_only;
        // v0.5.0 16-DOF 串口与 CAN 带宽保护：16 轴固定上限 150 Hz（921600 bps 下 16*150*320 = 768 kbps）
        const unsigned clamped = hz == 0 ? kDefaultHz : std::clamp(hz, 50u, kMaxHz);
        target_hz_ = clamped;
        running_ = true;
        io_thread_ = std::thread(&HandClientG4_16dof::IoLoop, this);
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
    unsigned target_hz_ = kDefaultHz;
    std::thread io_thread_;
    std::mutex cmd_mutex_;
    std::map<JointKey, MotorCommand> commands_;
    std::mutex snapshot_mutex_;
    std::map<JointKey, HandSnapshot> latest_snapshots_;
    std::map<JointKey, TimedHandSnapshot> latest_timed_snapshots_;
    std::array<std::atomic<uint8_t>, kAxisCount> bus_for_id_{};

    void ResetDefaultBuses() {
        // 实测 16-DOF GW02 物理总线拓扑（CAN 1–3）：
        // CAN3: 1..4 (拇指); CAN2: 5..8, 13..14 (食指/中指屈伸+侧摆); CAN1: 9..12, 15..16 (无名指/小指屈伸+侧摆)
        // 离线轴仍保留 CAN 1–3 动态自动发现，改线后无需改代码即可自动锁定。
        for (uint16_t id = 1; id <= 12; ++id) {
            bus_for_id_[id - 1].store(static_cast<uint8_t>(3 - (id - 1) / 4), std::memory_order_relaxed);
        }
        bus_for_id_[12].store(2, std::memory_order_relaxed);  // ID 13 -> CAN2
        bus_for_id_[13].store(2, std::memory_order_relaxed);  // ID 14 -> CAN2
        bus_for_id_[14].store(1, std::memory_order_relaxed);  // ID 15 -> CAN1
        bus_for_id_[15].store(1, std::memory_order_relaxed);  // ID 16 -> CAN1
    }

    void SendStopPeriodicOnce(uint16_t& sequence) {
        // v0.5.0 规范：16 轴禁用板端 12 槽周期轮询（command 8 aux=0 停止、aux=2 清空），非阻塞发送不影响离线测试。
        std::array<GatewayWireFrame, 2> ctrl{};
        ctrl[0] = gateway::Frame(8);
        ctrl[0].auxiliary = 0;
        ctrl[0].sequence = ++sequence;
        gateway::Seal(ctrl[0]);
        ctrl[1] = gateway::Frame(8);
        ctrl[1].auxiliary = 2;
        ctrl[1].sequence = ++sequence;
        gateway::Seal(ctrl[1]);
        bool term_err = false;
        serial_.WriteAll(ctrl.data(), ctrl.size() * sizeof(GatewayWireFrame), 1'500'000, 3'500'000, term_err);
        if (term_err) transport_lost_ = true;
    }

    void SendReleaseBatchAll() {
        std::array<GatewayWireFrame, kAxisCount> rel_batch;
        for (size_t i = 0; i < kAxisCount; ++i) {
            const uint16_t id = static_cast<uint16_t>(i + 1);
            rel_batch[i] = gateway::Release(BusForId(id), id);
            gateway::Seal(rel_batch[i]);
        }
        const size_t bytes = kAxisCount * sizeof(GatewayWireFrame);
        for (int r = 0; r < 3; ++r) {
            bool term_err = false;
            serial_.WriteAll(rel_batch.data(), bytes, 5'000'000, 12'000'000, term_err);
            if (term_err) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(7));
        }
    }

    void IoLoop() {
        gateway::Parser parser;
        uint8_t rx[2048];
        std::array<uint8_t, sizeof(GatewayWireFrame)> pending_tail{};
        size_t tail_len = 0;
        std::map<JointKey, gateway::RouteState> routes;
        for (uint16_t id = 1; id <= kAxisCount; ++id) {
            const uint8_t b = BusForId(id);
            routes.emplace(JointKey{b, id}, gateway::RouteState(b, id));
        }

        const auto cycle = std::chrono::microseconds(1000000 / target_hz_);
        uint16_t sequence = 0;
        uint32_t tick_count = 0;

        // 启动时先关闭可能残留的 GW02 12 槽板端轮询，确保进入 16 轴主机直发模式（mode=2）。
        SendStopPeriodicOnce(sequence);
        if (transport_lost_.load()) return;

        auto publish_snapshots = [&]() {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
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
                        if (f.kind != 0x90 || f.bus < 1 || f.bus > kMaxBus || f.can_id < 1 || f.can_id > kAxisCount) return;
                        JointKey k{f.bus, f.can_id};
                        const auto now = gateway::NowUs();
                        auto it = routes.find(k);
                        if (it == routes.end()) {
                            // 动态发现 CAN 1–3 真实总线：旧总线离线则自动迁移路由，双总线同时在线则保留以触发重号告警。
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
                const int w = serial_.WriteAll(pending_tail.data(), tail_len, 1'500'000, 4'500'000, term_err);
                if (term_err) { transport_lost_ = true; break; }
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
                std::array<GatewayWireFrame, kAxisCount> tx_batch;
                size_t tx_count = 0;
                const auto now_us = gateway::NowUs();
                const uint8_t probe_step = static_cast<uint8_t>((tick_count / 2) % kMaxBus);
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
                        // 离线且无控制命令的轴按 ~13ms 节拍轮询 CAN 1–3，自动发现 16 轴真实物理总线。
                        if (!c.enabled && !online && probe_step > 0) {
                            tx_bus = static_cast<uint8_t>(((k.bus - 1 + probe_step) % kMaxBus) + 1);
                        }
                        auto f = gateway::CommandFrame(tx_bus, k.id, c);
                        f.sequence = ++sequence;
                        gateway::Seal(f);
                        tx_batch[tx_count++] = f;
                    }
                }
                const size_t total = tx_count * sizeof(GatewayWireFrame);
                const int w = serial_.WriteAll(tx_batch.data(), total, 1'500'000, 4'500'000, term_err);
                if (term_err) { transport_lost_ = true; break; }
                if (w > 0 && static_cast<size_t>(w) < total) {
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

}  // namespace lqs
