// 12-DOF STM32G4 (LQS-GW02 v0.4.0) 驱动的 C ABI 包装（vendor/include/HandClientG4_12dof.hpp）。
// 在 SDK 之上保留指令期限 watchdog、真实主机收包时间戳、断线检测与安全卸力。
#include "HandClientG4_12dof.hpp"
#include "g4_api_12dof.h"

#include <time.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>
#include <thread>

namespace {
int64_t MonotonicNs12() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

struct Pending12 {
    int64_t deadline_ns = 0;
    int64_t disable_at_ns = 0;
};

struct Shim12 {
    lqs::HandClientG4_12dof client;
    std::mutex mutex;
    std::map<lqs::JointKey, Pending12> pending;
    std::atomic<bool> running{false};
    std::thread watchdog;
    uint64_t releases = 0;
    uint32_t disabled_mask = 0;

    void Limp(const lqs::JointKey& k, Pending12& p, int64_t now) {
        client.SetMitCommand(k.bus, k.id, 0.f, 0.f, 0.f, 0.f, 0.f);
        p.deadline_ns = 0;
        p.disable_at_ns = now + 20000000;
        ++releases;
    }

    void Loop() {
        while (running) {
            const int64_t now = MonotonicNs12();
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

void* OpenGateway12(const char* port, int baud, unsigned hz, bool wait_for_first_packets) {
    auto* shim = new Shim12();
    if (!shim->client.Connect(port, baud, hz, false, wait_for_first_packets)) {
        delete shim;
        return nullptr;
    }
    shim->running = true;
    shim->watchdog = std::thread(&Shim12::Loop, shim);
    return shim;
}
}  // namespace

extern "C" {

uint32_t g4_12dof_joint_size(void) { return sizeof(G4Joint); }
uint32_t g4_12dof_axis_count(void) { return lqs::HandClientG4_12dof::kAxisCount; }
int64_t g4_12dof_now_ns(void) { return MonotonicNs12(); }

void* g4_12dof_open(const char* port, int baud, unsigned hz) {
    return OpenGateway12(port, baud, hz, true);
}
void* g4_12dof_open_fresh(const char* port, int baud, unsigned hz) {
    return OpenGateway12(port, baud, hz, false);
}
void* g4_12dof_open_reader(const char* port) {
    auto* shim = new Shim12();
    if (!shim->client.Connect(port, 921600, lqs::HandClientG4_12dof::kDefaultHz, true, true)) {
        delete shim;
        return nullptr;
    }
    return shim;
}

int g4_12dof_transport_lost(void* h) {
    if (!h) return true;
    const auto& client = static_cast<Shim12*>(h)->client;
    if (client.TransportLost()) return true;
    constexpr uint64_t silence_us = 3'000'000;
    const auto last = client.LastReplyUs();
    const auto now_us = gateway::NowUs();
    return last && now_us > last && now_us - last >= silence_us;
}

void g4_12dof_set_mit(void* h, uint32_t bus, uint32_t id, float deg, float kp, float kd, float vel,
                      float tor, int64_t deadline_ns) {
    auto* shim = static_cast<Shim12*>(h);
    std::lock_guard<std::mutex> lock(shim->mutex);
    const uint8_t b = bus == 0 ? shim->client.BusForId(uint16_t(id)) : uint8_t(bus);
    shim->client.SetMitCommand(b, uint16_t(id), deg, kp, kd, vel, tor);
    shim->pending[{b, uint16_t(id)}] = Pending12{deadline_ns, 0};
}

void g4_12dof_set_disabled_mask(void* h, uint32_t mask) {
    if (!h) return;
    auto* shim = static_cast<Shim12*>(h);
    std::lock_guard<std::mutex> lock(shim->mutex);
    shim->disabled_mask = mask;
    for (uint16_t id = 1; id <= lqs::HandClientG4_12dof::kAxisCount; ++id) {
        if (mask & (1u << (id - 1))) {
            const uint8_t bus = shim->client.BusForId(id);
            shim->pending.erase({bus, id});
            shim->client.DisableJoint(bus, id);
        }
    }
}

int g4_12dof_renew_mit_batch(void* h, const float* commands, int count, int64_t deadline_ns,
                             int64_t previous_deadline_ns) {
    if (!h || !commands || count != int(lqs::HandClientG4_12dof::kAxisCount)) return -1;
    auto* shim = static_cast<Shim12*>(h);
    std::lock_guard<std::mutex> lock(shim->mutex);
    if (shim->client.TransportLost()) return -1;
    std::array<std::pair<lqs::JointKey, MotorCommand>, lqs::HandClientG4_12dof::kAxisCount> batch;
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
    if (!shim->client.SetCommandBatchBefore(batch, submit_before)) return 0;
    for (int axis = 0; axis < count; ++axis) {
        if (shim->disabled_mask & (1u << axis)) {
            shim->pending.erase(batch[axis].first);
            continue;
        }
        shim->pending[batch[axis].first] = Pending12{deadline_ns, 0};
    }
    return 1;
}

int g4_12dof_set_mit_batch(void* h, const float* commands, int count, int64_t deadline_ns) {
    return g4_12dof_renew_mit_batch(h, commands, count, deadline_ns, 0);
}

int g4_12dof_homing_mit_batch(void* h, const float* commands, int count, uint32_t active_mask, int64_t deadline_ns) {
    if (!h || !commands || count != int(lqs::HandClientG4_12dof::kAxisCount)) return -1;
    auto* shim = static_cast<Shim12*>(h);
    std::lock_guard<std::mutex> lock(shim->mutex);
    if (shim->client.TransportLost()) return -1;
    const uint32_t effective_mask = active_mask & ~shim->disabled_mask;
    std::array<std::pair<lqs::JointKey, MotorCommand>, lqs::HandClientG4_12dof::kAxisCount> batch;
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
    if (!shim->client.SetCommandBatchBefore(batch, deadline_ns)) return 0;
    for (int axis = 0; axis < count; ++axis) {
        if (effective_mask & (1u << axis)) shim->pending[batch[axis].first] = Pending12{deadline_ns, 0};
        else shim->pending.erase(batch[axis].first);
    }
    return 1;
}

void g4_12dof_set_servo(void* h, uint32_t bus, uint32_t id, float deg, float spd, float cur, int64_t deadline_ns) {
    auto* shim = static_cast<Shim12*>(h);
    std::lock_guard<std::mutex> lock(shim->mutex);
    const uint8_t b = bus == 0 ? shim->client.BusForId(uint16_t(id)) : uint8_t(bus);
    shim->client.SetServoCommand(b, uint16_t(id), deg, spd, cur);
    shim->pending[{b, uint16_t(id)}] = Pending12{deadline_ns, 0};
}

void g4_12dof_release(void* h, uint32_t bus, uint32_t id) {
    auto* shim = static_cast<Shim12*>(h);
    std::lock_guard<std::mutex> lock(shim->mutex);
    const uint8_t target_bus = bus == 0 ? shim->client.BusForId(uint16_t(id)) : uint8_t(bus);
    lqs::JointKey key{target_bus, uint16_t(id)};
    if (id >= 1 && id <= lqs::HandClientG4_12dof::kAxisCount && (shim->disabled_mask & (1u << (id - 1)))) {
        shim->pending.erase(key);
        shim->client.DisableJoint(target_bus, uint16_t(id));
        return;
    }
    shim->Limp(key, shim->pending[key], MonotonicNs12());
}

int g4_12dof_snapshots_with_current_time(void* h, G4Joint* out, int64_t* current_received_ns, int capacity) {
    auto* shim = static_cast<Shim12*>(h);
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

int g4_12dof_snapshots(void* h, G4Joint* out, int capacity) {
    return g4_12dof_snapshots_with_current_time(h, out, nullptr, capacity);
}

uint64_t g4_12dof_release_count(void* h) {
    auto* shim = static_cast<Shim12*>(h);
    std::lock_guard<std::mutex> lock(shim->mutex);
    return shim->releases;
}

void g4_12dof_close(void* h) {
    auto* shim = static_cast<Shim12*>(h);
    shim->running = false;
    if (shim->watchdog.joinable()) shim->watchdog.join();
    shim->client.Disconnect();
    delete shim;
}

// 兼容别名（保留原有 12-DOF 测试与 ctypes 入口零回归）
uint32_t g4_joint_size(void) { return g4_12dof_joint_size(); }
int64_t g4_now_ns(void) { return g4_12dof_now_ns(); }
void* g4_open(const char* port, int baud, unsigned hz) { return g4_12dof_open(port, baud, hz); }
void* g4_open_fresh(const char* port, int baud, unsigned hz) { return g4_12dof_open_fresh(port, baud, hz); }
void* g4_open_reader(const char* port) { return g4_12dof_open_reader(port); }
int g4_transport_lost(void* h) { return g4_12dof_transport_lost(h); }
void g4_close(void* h) { g4_12dof_close(h); }
int g4_snapshots(void* h, G4Joint* out, int capacity) { return g4_12dof_snapshots(h, out, capacity); }
int g4_snapshots_with_current_time(void* h, G4Joint* out, int64_t* ct, int cap) { return g4_12dof_snapshots_with_current_time(h, out, ct, cap); }
void g4_set_mit(void* h, uint32_t bus, uint32_t id, float deg, float kp, float kd, float vel, float tor, int64_t d) { g4_12dof_set_mit(h, bus, id, deg, kp, kd, vel, tor, d); }
int g4_set_mit_batch(void* h, const float* c, int n, int64_t d) { return g4_12dof_set_mit_batch(h, c, n, d); }
int g4_renew_mit_batch(void* h, const float* c, int n, int64_t d, int64_t p) { return g4_12dof_renew_mit_batch(h, c, n, d, p); }
int g4_homing_mit_batch(void* h, const float* c, int n, uint32_t m, int64_t d) { return g4_12dof_homing_mit_batch(h, c, n, m, d); }
void g4_set_servo(void* h, uint32_t bus, uint32_t id, float deg, float spd, float cur, int64_t d) { g4_12dof_set_servo(h, bus, id, deg, spd, cur, d); }
void g4_set_disabled_mask(void* h, uint32_t mask) { g4_12dof_set_disabled_mask(h, mask); }
void g4_release(void* h, uint32_t bus, uint32_t id) { g4_12dof_release(h, bus, id); }
uint64_t g4_release_count(void* h) { return g4_12dof_release_count(h); }
}
