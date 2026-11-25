// 厂商SDK的C ABI包装（vendor/include/HandClientG4.hpp）；时间快照扩展见vendor/LOCAL_CHANGES.md。
// for "驱动 2" (STM32G4 LQS-GW02 gateway).  Loaded from Python with ctypes.
//
// 在 SDK 之上保留指令期限 watchdog：控制器停止刷新指令时，先发送
// 零增益 MIT 卸力帧，再 DisableJoint，避免 SDK 持续发送过期目标。
// 位置与接收时间一起取自 SDK 的 GetTimedSnapshots，不依赖 watchdog 调度。
#include "HandClientG4.hpp"
#include "g4_api.h"

#include <time.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>
#include <thread>

namespace {
int64_t MonotonicNs() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);  // == Python time.monotonic_ns()
    return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

struct Pending {
    int64_t deadline_ns = 0;       // 0: no deadline
    int64_t disable_at_ns = 0;     // >0: limp MIT sent, DisableJoint at this time
};

struct Shim {
    lqs::HandClientG4 client;
    std::mutex mutex;
    std::map<lqs::JointKey, Pending> pending;
    std::atomic<bool> running{false};
    std::thread watchdog;
    uint64_t releases = 0;
    uint32_t disabled_mask = 0;

    void Limp(const lqs::JointKey& k, Pending& p, int64_t now) {
        client.SetMitCommand(k.bus, k.id, 0.f, 0.f, 0.f, 0.f, 0.f);
        p.deadline_ns = 0;
        p.disable_at_ns = now + 20000000;  // >= 3 SDK cycles of zero-gain MIT
        ++releases;
    }

    void Loop() {
        while (running) {
            const int64_t now = MonotonicNs();
            {
                std::lock_guard<std::mutex> lock(mutex);
                for (auto& [k, p] : pending) {
                    if (p.deadline_ns && now >= p.deadline_ns) Limp(k, p, now);
                    if (p.disable_at_ns && now >= p.disable_at_ns) {
                        client.DisableJoint(k.bus, k.id);
                        p.disable_at_ns = 0;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
};
}  // namespace

extern "C" {

uint32_t g4_joint_size() { return sizeof(G4Joint); }
int64_t g4_now_ns() { return MonotonicNs(); }

static void* OpenGateway(const char* port, int baud, unsigned hz, bool wait_for_first_packets, int axis_count = 12) {
    auto* shim = new Shim();
    if (!shim->client.Connect(port, baud, hz, false, wait_for_first_packets, static_cast<size_t>(axis_count))) {
        delete shim;
        return nullptr;
    }
    shim->running = true;
    shim->watchdog = std::thread(&Shim::Loop, shim);
    return shim;
}

void* g4_open(const char* port, int baud, unsigned hz) {
    return OpenGateway(port, baud, hz, true, 12);
}

void* g4_open_axes(const char* port, int baud, unsigned hz, int axis_count) {
    return OpenGateway(port, baud, hz, true, axis_count);
}

// 重连使用干净的 SDK/路由/看门狗状态，以实测新样本作为就绪依据。
void* g4_open_fresh(const char* port, int baud, unsigned hz) {
    return OpenGateway(port, baud, hz, false, 12);
}

void* g4_open_fresh_axes(const char* port, int baud, unsigned hz, int axis_count) {
    return OpenGateway(port, baud, hz, false, axis_count);
}

int g4_transport_lost(void* h) {
    if (!h) return true;
    const auto& client = static_cast<Shim*>(h)->client;
    if (client.TransportLost()) return true;
    // CH341 bulk 端点可能停止收发，但 tty 仍存在且 poll 没有 HUP。
    // 只有曾收到有效回包的会话才启用此检查，新句柄无回包时不会反复重开。
    constexpr uint64_t silence_us = 3'000'000;
    const auto last = client.LastReplyUs();
    const auto now_us = gateway::NowUs();
    return last && now_us > last && now_us - last >= silence_us;
}

// 仅读取位置。采集期间和关闭时均不发送电机控制帧。
void* g4_open_reader(const char* port) {
    auto* shim = new Shim();
    if (!shim->client.Connect(port, 921600, 200, true, true, 12)) {
        delete shim;
        return nullptr;
    }
    return shim;
}

void* g4_open_reader_axes(const char* port, int axis_count) {
    auto* shim = new Shim();
    if (!shim->client.Connect(port, 921600, 200, true, true, static_cast<size_t>(axis_count))) {
        delete shim;
        return nullptr;
    }
    return shim;
}

void g4_set_mit(void* h, uint32_t bus, uint32_t id, float deg, float kp, float kd, float vel,
                float tor, int64_t deadline_ns) {
    auto* shim = static_cast<Shim*>(h);
    std::lock_guard<std::mutex> lock(shim->mutex);
    shim->client.SetMitCommand(uint8_t(bus), uint16_t(id), deg, kp, kd, vel, tor);
    shim->pending[{uint8_t(bus), uint16_t(id)}] = Pending{deadline_ns, 0};
}

// 先校验完整批次，再一次提交给 IO 线程。返回 0 表示过期，-1 表示参数无效。
int g4_set_mit_batch(void* h, const float* commands, int count, int64_t deadline_ns) {
    return g4_renew_mit_batch(h, commands, count, deadline_ns, 0);
}

void g4_set_disabled_mask(void* h, uint32_t mask) {
    if (!h) return;
    auto* shim = static_cast<Shim*>(h);
    std::lock_guard<std::mutex> lock(shim->mutex);
    shim->disabled_mask = mask;
    const size_t n = shim->client.AxisCount();
    for (uint16_t id = 1; id <= n; ++id) {
        if (mask & (1u << (id - 1))) {
            const uint8_t bus = shim->client.BusForId(id);
            shim->pending.erase({bus, id});
            shim->client.DisableJoint(bus, id);
        }
    }
}

int g4_renew_mit_batch(void* h, const float* commands, int count, int64_t deadline_ns,
                      int64_t previous_deadline_ns) {
    if (!h || !commands || (count != 12 && count != 16)) return -1;
    auto* shim = static_cast<Shim*>(h);
    std::lock_guard<std::mutex> lock(shim->mutex);
    if (shim->client.TransportLost()) return -1;
    std::array<std::pair<lqs::JointKey, MotorCommand>, lqs::HandClientG4::kMaxAxes> batch;
    for (int axis = 0; axis < count; ++axis) {
        const float* fields = commands + axis * 5;
        for (int field = 0; field < 5; ++field) {
            if (!std::isfinite(fields[field])) return -1;
        }
        auto& [key, command] = batch[axis];
        const uint16_t id = static_cast<uint16_t>(axis + 1);
        key = {shim->client.BusForId(id), id};
        if (shim->disabled_mask & (1u << axis)) {
            command.enabled = false;
            command.is_mit_mode = false;
        } else {
            command.enabled = true;
            command.is_mit_mode = true;
            command.target_angle = fields[0];
            command.mit_kp = fields[1];
            command.mit_kd = fields[2];
            command.mit_vel = fields[3];
            command.mit_tor = fields[4];
        }
    }
    if (previous_deadline_ns) {
        for (int axis = 0; axis < count; ++axis) {
            if (shim->disabled_mask & (1u << axis)) continue;
            const auto& entry = batch[axis];
            auto it = shim->pending.find(entry.first);
            if (it == shim->pending.end() || it->second.deadline_ns != previous_deadline_ns) return 0;
        }
    }
    const auto submit_before = previous_deadline_ns ? std::min(previous_deadline_ns, deadline_ns) : deadline_ns;
    if (!shim->client.SetCommandBatchBefore(batch.data(), static_cast<size_t>(count), submit_before)) return 0;
    for (int axis = 0; axis < count; ++axis) {
        if (shim->disabled_mask & (1u << axis)) {
            shim->pending.erase(batch[axis].first);
            continue;
        }
        shim->pending[batch[axis].first] = Pending{deadline_ns, 0};
    }
    return 1;
}

int g4_homing_mit_batch(void* h, const float* commands, int count, uint32_t active_mask, int64_t deadline_ns) {
    if (!h || !commands || (count != 12 && count != 16)) return -1;
    auto* shim = static_cast<Shim*>(h);
    std::lock_guard<std::mutex> lock(shim->mutex);
    if (shim->client.TransportLost()) return -1;
    const uint32_t effective_mask = active_mask & ~shim->disabled_mask;
    std::array<std::pair<lqs::JointKey, MotorCommand>, lqs::HandClientG4::kMaxAxes> batch;
    for (int axis = 0; axis < count; ++axis) {
        const float* fields = commands + axis * 5;
        for (int field = 0; field < 5; ++field) {
            if (!std::isfinite(fields[field])) return -1;
        }
        auto& [key, command] = batch[axis];
        const uint16_t id = static_cast<uint16_t>(axis + 1);
        key = {shim->client.BusForId(id), id};
        if (effective_mask & (1u << axis)) {
            command.enabled = true;
            command.is_mit_mode = true;
            command.target_angle = fields[0];
            command.mit_kp = fields[1];
            command.mit_kd = fields[2];
            command.mit_vel = fields[3];
            command.mit_tor = fields[4];
        } else {
            command.enabled = false;
            command.is_mit_mode = false;
        }
    }
    if (!shim->client.SetCommandBatchBefore(batch.data(), static_cast<size_t>(count), deadline_ns)) return 0;
    for (int axis = 0; axis < count; ++axis) {
        if (effective_mask & (1u << axis)) {
            shim->pending[batch[axis].first] = Pending{deadline_ns, 0};
        } else {
            shim->pending.erase(batch[axis].first);
        }
    }
    return 1;
}

void g4_set_servo(void* h, uint32_t bus, uint32_t id, float deg, float spd, float cur,
                  int64_t deadline_ns) {
    auto* shim = static_cast<Shim*>(h);
    std::lock_guard<std::mutex> lock(shim->mutex);
    shim->client.SetServoCommand(uint8_t(bus), uint16_t(id), deg, spd, cur);
    shim->pending[{uint8_t(bus), uint16_t(id)}] = Pending{deadline_ns, 0};
}

// Zero-gain MIT for a few cycles, then back to read-only query.
void g4_release(void* h, uint32_t bus, uint32_t id) {
    auto* shim = static_cast<Shim*>(h);
    std::lock_guard<std::mutex> lock(shim->mutex);
    const uint8_t target_bus = bus == 0 ? shim->client.BusForId(uint16_t(id)) : uint8_t(bus);
    lqs::JointKey key{target_bus, uint16_t(id)};
    if (id >= 1 && id <= lqs::HandClientG4::kMaxAxes && (shim->disabled_mask & (1u << (id - 1)))) {
        shim->pending.erase(key);
        shim->client.DisableJoint(target_bus, uint16_t(id));
        return;
    }
    shim->Limp(key, shim->pending[key], MonotonicNs());
}

int g4_snapshots_with_current_time(void* h, G4Joint* out, int64_t* current_received_ns, int capacity) {
    auto* shim = static_cast<Shim*>(h);
    const auto snaps = shim->client.GetTimedSnapshots();
    int n = 0;
    for (const auto& [k, timed] : snaps) {
        if (n >= capacity) break;
        const auto& s = timed.state;
        if (current_received_ns) current_received_ns[n] = int64_t(timed.current_received_us) * 1000;
        G4Joint& j = out[n++];
        j.bus = s.bus; j.id = s.id; j.valid = s.valid; j.error = s.error;
        j.angle_deg = s.angle_deg; j.current_a = s.current_a; j.temp_c = s.temp_c;
        j.speed_rpm = s.speed_rpm; j.hz = s.hz; j.samples = s.samples;
        j.received_ns = int64_t(timed.position_received_us) * 1000;
        j.board_timestamp_us = s.board_timestamp_us; j.max_board_gap_us = s.max_board_gap_us;
        j.max_host_gap_us = s.max_host_gap_us; j.tx_failures = s.tx_failures;
    }
    return n;
}

int g4_snapshots(void* h, G4Joint* out, int capacity) {
    return g4_snapshots_with_current_time(h, out, nullptr, capacity);
}

uint64_t g4_release_count(void* h) {
    auto* shim = static_cast<Shim*>(h);
    std::lock_guard<std::mutex> lock(shim->mutex);
    return shim->releases;
}

// SDK Disconnect(): stops the IO thread and sends the vendor 3x safe release.
void g4_close(void* h) {
    auto* shim = static_cast<Shim*>(h);
    shim->running = false;
    if (shim->watchdog.joinable()) shim->watchdog.join();
    shim->client.Disconnect();
    delete shim;
}
}
