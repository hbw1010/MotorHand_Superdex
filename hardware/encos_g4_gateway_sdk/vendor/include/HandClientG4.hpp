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
    HandClientG4() = default;
    ~HandClientG4() { Disconnect(); }

    bool Connect(const std::string& port = "/dev/ttyUSB0", int baud = 921600, unsigned hz = 200,
                 bool query_only = false, bool wait_for_first_packets = true) {
        Disconnect();
        if (!serial_.Open(port, baud)) return false;
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

    void SetCommandBatch(const std::array<std::pair<JointKey, MotorCommand>, 12>& batch) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        for (const auto& [key, command] : batch) commands_[key] = command;
    }

    bool SetCommandBatchBefore(const std::array<std::pair<JointKey, MotorCommand>, 12>& batch, int64_t until_ns) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        if (std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() >= until_ns) return false;
        for (const auto& [key, command] : batch) commands_[key] = command;
        return true;
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

    void SendReleaseBatchAll() {
        static const JointKey all_joints[] = {
            {1, 9}, {1, 10}, {1, 11}, {1, 12},
            {2, 5}, {2, 6},  {2, 7},  {2, 8},
            {3, 1}, {3, 2},  {3, 3},  {3, 4}
        };

        std::array<GatewayWireFrame, 12> rel_batch;
        for (size_t i = 0; i < 12; ++i) {
            rel_batch[i] = gateway::Release(all_joints[i].bus, all_joints[i].id);
            gateway::Seal(rel_batch[i]);
        }

        for (int r = 0; r < 3; ++r) {
            bool term_err = false;
            serial_.WriteAll(rel_batch.data(), sizeof(rel_batch), 5'000'000, 10'000'000, term_err);
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
        auto add_route = [&](uint8_t b, uint16_t id) { routes.emplace(JointKey{b, id}, gateway::RouteState(b, id)); };
        for (uint16_t id = 9; id <= 12; ++id) add_route(1, id);
        for (uint16_t id = 5; id <= 8; ++id) add_route(2, id);
        for (uint16_t id = 1; id <= 4; ++id) add_route(3, id);

        const auto cycle = std::chrono::microseconds(1000000 / target_hz_);
        uint16_t sequence = 0;

        auto publish_snapshots = [&]() {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            // Read/Accept之后取时间，避免age_ms减去未来收包时间而无符号下溢。
            const auto snapshot_us = gateway::NowUs();
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
                        JointKey k{f.bus, f.can_id};
                        const auto it = routes.find(k);
                        const auto now = gateway::NowUs();
                        if (it != routes.end() && it->second.Accept(f, now)) {
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
                std::array<GatewayWireFrame, 12> tx_batch;
                size_t tx_count = 0;
                {
                    std::lock_guard<std::mutex> lock(cmd_mutex_);
                    for (auto& [k, route] : routes) {
                        MotorCommand c;
                        if (!query_only_ && commands_.count(k)) c = commands_[k];
                        auto f = gateway::CommandFrame(k.bus, k.id, c);
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
