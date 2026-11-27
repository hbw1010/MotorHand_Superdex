#pragma once
#include "WinSerial.hpp"
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <utility>

class LqsGatewaySDK {
public:
    LqsGatewaySDK() : wake_(CreateEventW(nullptr, FALSE, FALSE, nullptr)) {
        if (!wake_) throw std::runtime_error("Cannot create worker event");
    }
    ~LqsGatewaySDK() { Disconnect(); CloseHandle(wake_); }
    bool Connect(const char* port, int baud = 921600) {
        Disconnect();
        try {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                routes_.clear(); diagnostics_ = {}; error_.clear(); desired_hz_ = 200;
                telemetry_enabled_ = false;
            }
            parser_ = {}; sequence_ = 0; periodic_ = false;
            serial_.Open(port, baud);
            auto ping = Rpc(gateway::Frame(1));
            if (ping.len != 8 || std::memcmp(ping.data, "LQS-GW02", 8))
                throw std::runtime_error("GW02 firmware required");
            PollControl(0); PollControl(2);
            for (uint8_t bus = 1; bus <= 3; ++bus) {
                for (uint16_t id = 1; id <= 16; ++id) {
                    gateway::RouteState candidate(bus, id);
                    for (int attempt = 0; attempt < 2 && !candidate.samples; ++attempt) {
                        scan_ = &candidate;
                        SendWait(gateway::Query(bus, id));
                        const auto until = gateway::NowUs() + 8000;
                        while (!candidate.samples && gateway::NowUs() < until) { Pump(); serial_.Wait(1); }
                        scan_ = nullptr;
                    }
                    if (candidate.samples) routes_.push_back(candidate);
                }
            }
            if (routes_.empty()) throw std::runtime_error("GW02 connected, no motor replies on CAN1-3 / ID1-16");
            if (routes_.size() > 16) throw std::runtime_error("GW02 supports at most 16 routes");
            if (routes_.size() <= 12) ConfigurePeriodic(200);
            else {
                PollControl(0); PollControl(2);
                desired_hz_ = 150; active_hz_ = 150;
                ResetMetrics();
                std::lock_guard<std::mutex> lock(mutex_);
                diagnostics_.mode = 2; diagnostics_.target_hz = 150;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                diagnostics_.connected = 1; diagnostics_.routes = uint32_t(routes_.size());
            }
            running_ = true;
            thread_ = std::thread(&LqsGatewaySDK::IoLoop, this);
            return true;
        } catch (const std::exception& error) {
            scan_ = nullptr; SetError(error.what());
            serial_.Close();
            return false;
        }
    }
    void Disconnect() {
        running_ = false; SetEvent(wake_);
        if (thread_.joinable()) thread_.join();
        serial_.Close();
        std::lock_guard<std::mutex> lock(mutex_);
        diagnostics_.connected = 0;
    }
    std::string LastError() const { std::lock_guard<std::mutex> lock(mutex_); return error_; }
    std::vector<std::pair<int,int>> GetDiscoveredRoutes() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<std::pair<int,int>> result;
        for (const auto& route : routes_) result.emplace_back(route.bus, route.id);
        return result;
    }
    bool SetCommand(int bus, int id, const MotorCommand& command) {
        if (!gateway::ValidCommand(command)) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        if (!diagnostics_.connected) return false;
        auto* route = Find(bus, id);
        if (!route) return false;
        if (command.enabled && !route->command.enabled && !route->Snapshot(gateway::NowUs()).online) return false;
        if (command.enabled && route->release.pending) return false;
        if (route->command.enabled && !command.enabled) route->release.Begin();
        route->command = command; SetEvent(wake_);
        return true;
    }
    void SetCommand(int id, const MotorCommand& command) { SetCommand(0, id, command); }
    bool GetFeedback(int id, MotorFeedback& feedback) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto* route = Find(0, id);
        if (!diagnostics_.connected || !route || !route->Snapshot(gateway::NowUs()).online) return false;
        feedback = route->feedback; return true;
    }
    int GetSnapshots(HandSnapshot* output, int capacity) const {
        if (!output || capacity <= 0) return 0;
        std::lock_guard<std::mutex> lock(mutex_);
        int count = std::min(capacity, int(routes_.size()));
        const auto now = gateway::NowUs();
        for (int i = 0; i < count; ++i) {
            output[i] = routes_[i].Snapshot(now);
            if (!diagnostics_.connected) { output[i].online = 0; output[i].hz = 0; }
        }
        return count;
    }
    float GetActualLoopHz() const {
        HandSnapshot items[16]{}; const int count = GetSnapshots(items, 16);
        float minimum = count ? items[0].hz : 0;
        for (int i = 1; i < count; ++i) minimum = std::min(minimum, items[i].hz);
        return minimum;
    }
    HandDiagnostics Diagnostics() const { std::lock_guard<std::mutex> lock(mutex_); return diagnostics_; }
    bool SetRate(unsigned hz) {
        if (hz < 100 || hz > 235) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        if (!diagnostics_.connected) return false;
        desired_hz_ = (routes_.size() > 12 && hz > 150) ? 150 : hz;
        SetEvent(wake_); return true;
    }
    bool SetTelemetry(bool enabled) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!diagnostics_.connected) return false;
        telemetry_enabled_ = enabled;
        diagnostics_.telemetry_enabled = enabled;
        SetEvent(wake_); return true;
    }
    void ResetMetrics() {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& route : routes_) {
            route.samples = route.window_start = route.window_samples = 0;
            route.max_board_gap = route.max_host_gap = route.host_over10 = route.board_over10 = 0;
            route.hz = 0;
        }
    }
private:
    WinSerial serial_;
    gateway::Parser parser_;
    mutable std::mutex mutex_;
    std::vector<gateway::RouteState> routes_;
    HandDiagnostics diagnostics_;
    std::string error_;
    std::thread thread_;
    HANDLE wake_;
    std::atomic<bool> running_{false};
    std::atomic<unsigned> desired_hz_{200};
    std::atomic<bool> telemetry_enabled_{false};
    uint16_t sequence_ = 0, expected_sequence_ = 0;
    uint8_t expected_kind_ = 0;
    bool reply_ready_ = false, periodic_ = false;
    unsigned active_hz_ = 200;
    uint32_t latest_board_us_ = 0;
    GatewayWireFrame reply_{};
    gateway::RouteState* scan_ = nullptr;
    const gateway::RouteState* Find(int bus, int id) const {
        const gateway::RouteState* found = nullptr;
        for (const auto& route : routes_) {
            if (route.id != id || (bus && route.bus != bus)) continue;
            if (found) return nullptr;
            found = &route;
        }
        return found;
    }
    gateway::RouteState* Find(int bus, int id) {
        return const_cast<gateway::RouteState*>(static_cast<const LqsGatewaySDK*>(this)->Find(bus,id));
    }
    void SetError(const std::string& error) {
        std::lock_guard<std::mutex> lock(mutex_);
        error_ = error; diagnostics_.connected = 0;
    }
    void Pump() {
        serial_.Pump([this](const uint8_t* bytes, size_t size) {
            parser_.Feed(bytes, size, [this](const GatewayWireFrame& frame) { Receive(frame); });
        });
        serial_.WriteBusy();
        std::lock_guard<std::mutex> lock(mutex_);
        diagnostics_.crc_errors = parser_.crc_errors;
        diagnostics_.invalid_frames = parser_.invalid_frames;
    }
    void Receive(const GatewayWireFrame& frame) {
        if (frame.kind == expected_kind_ && frame.sequence == expected_sequence_) {
            reply_ = frame; reply_ready_ = true;
        }
        const auto now = gateway::NowUs();
        if (scan_ && frame.len == 6 && frame.data[1] == 1) scan_->Accept(frame, now);
        std::lock_guard<std::mutex> lock(mutex_);
        if (frame.kind == 0x90) {
            latest_board_us_ = frame.timestamp_us;
            ++diagnostics_.received_can;
            auto* route = Find(frame.bus, frame.can_id);
            if (route && !route->Accept(frame, now)) ++parser_.invalid_frames;
        } else if (frame.kind == 0x92 || (frame.kind == 0x82 && frame.flags)) {
            ++diagnostics_.tx_failures;
            auto* route = Find(frame.bus, frame.can_id);
            if (route) {
                ++route->tx_failures;
                route->release.Failure(frame.sequence);
            }
        } else if (frame.kind == 0x83 && frame.bus == 0 && frame.can_id == 0) {
            uint32_t crc = 0, drops = 0;
            std::memcpy(&crc, frame.data, 4); std::memcpy(&drops, frame.data + 4, 4);
            diagnostics_.gateway_crc = crc; diagnostics_.gateway_drops = drops;
            diagnostics_.gateway_uart_errors = frame.auxiliary;
        }
    }
    void Prepare(GatewayWireFrame& frame) { frame.sequence = ++sequence_; gateway::Seal(frame); }
    void DrainWrite() {
        const auto deadline = gateway::NowUs() + 200000;
        while (serial_.WriteBusy()) {
            Pump();
            if (gateway::NowUs() > deadline) throw std::runtime_error("Serial write timeout");
            serial_.Wait(1);
        }
    }
    uint16_t SendWait(GatewayWireFrame frame) {
        DrainWrite(); Prepare(frame);
        if (!serial_.Send(&frame, 1)) throw std::runtime_error("Serial write busy");
        return frame.sequence;
    }
    GatewayWireFrame Rpc(GatewayWireFrame frame, unsigned retries = 3) {
        DrainWrite(); Prepare(frame);
        expected_kind_ = frame.kind | 0x80; expected_sequence_ = frame.sequence; reply_ready_ = false;
        serial_.Send(&frame, 1);
        const auto deadline = gateway::NowUs() + 250000;
        while (!reply_ready_) {
            Pump();
            if (gateway::NowUs() > deadline) throw std::runtime_error("Gateway RPC timeout, command " + std::to_string(frame.kind));
            serial_.Wait(1);
        }
        expected_kind_ = 0;
        if (reply_.flags == 3 && retries && (frame.kind == 7 || frame.kind == 8)) {
            const auto quiet_until = gateway::NowUs() + 12000;
            while (gateway::NowUs() < quiet_until) { Pump(); serial_.Wait(1); }
            return Rpc(frame, retries - 1);
        }
        if (reply_.flags) throw std::runtime_error("Gateway rejected command " + std::to_string(frame.kind) + " status " + std::to_string(reply_.flags));
        return reply_;
    }
    void PollControl(unsigned action) {
        auto frame = gateway::Frame(8); frame.auxiliary = action; Rpc(frame);
        if (action != 1) periodic_ = false;
    }
    void ConfigurePeriodic(unsigned hz) {
        PollControl(0); PollControl(2);
        const auto quiet_until = gateway::NowUs() + 12000;
        while (gateway::NowUs() < quiet_until) { Pump(); serial_.Wait(1); }
        for (size_t i = 0; i < routes_.size(); ++i) {
            auto frame = gateway::Query(routes_[i].bus, routes_[i].id);
            frame.kind = 7; frame.detail = uint16_t(i); Rpc(frame);
        }
        auto start = gateway::Frame(8); start.auxiliary = 1; start.len = 8;
        uint32_t period = 1000000 / hz;
        std::memcpy(start.data, &period, 4); Rpc(start);
        periodic_ = true; active_hz_ = hz;
        ResetMetrics();
        std::lock_guard<std::mutex> lock(mutex_);
        diagnostics_.mode = 1; diagnostics_.target_hz = hz;
    }
    void IoLoop() noexcept {
        try {
            uint64_t next_cycle = gateway::NowUs(), next_status = next_cycle;
            while (running_) {
                Pump();
                bool enabled = false, release = false;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    for (auto& route : routes_) {
                        if (route.release.Settled(gateway::NowUs())) {
                            if (route.release.failed) throw std::runtime_error("Soft release CAN failure, check physical motor state");
                            route.release.pending = false;
                        }
                        enabled |= route.command.enabled;
                        release |= route.release.pending;
                    }
                }
                if ((enabled || release) && periodic_) {
                    PollControl(0);
                    std::lock_guard<std::mutex> lock(mutex_);
                    diagnostics_.mode = 2; diagnostics_.target_hz = 200;
                    next_cycle = gateway::NowUs();
                }
                if (!enabled && !release && routes_.size() <= 12 && (!periodic_ || active_hz_ != desired_hz_)) ConfigurePeriodic(desired_hz_);
                const auto now = gateway::NowUs();
                const uint64_t cycle_us = routes_.size() > 12 ? (1000000ULL / desired_hz_.load()) : 5000ULL;
                if (!periodic_ && now >= next_cycle) {
                    const auto elapsed = (now - next_cycle) / cycle_us;
                    next_cycle += (elapsed + 1) * cycle_us;
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (routes_.size() > 12) { diagnostics_.mode = 2; diagnostics_.target_hz = desired_hz_.load(); }
                    diagnostics_.skipped_cycles += elapsed;
                    if (serial_.WriteBusy()) ++diagnostics_.skipped_cycles;
                    else {
                        std::array<GatewayWireFrame, 16> frames{};
                        size_t count = 0;
                        for (auto& route : routes_) {
                            frames[count] = route.release.NeedsSend() ? gateway::Release(route.bus, route.id) :
                                gateway::CommandFrame(route.bus, route.id, route.command);
                            Prepare(frames[count]);
                            if (route.release.NeedsSend()) route.release.Submitted(frames[count].sequence, now);
                            ++count;
                        }
                        serial_.Send(frames.data(), count);
                    }
                }
                const unsigned cur_hz = periodic_ ? active_hz_ : desired_hz_.load();
                if (telemetry_enabled_ && routes_.size() * (cur_hz + 10) <= 2600 && !serial_.WriteBusy()) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    // Send one supplement just after this route's fresh position response.
                    // A burst on a separate host timer can overlap the MCU's next query.
                    for (auto& route : routes_) {
                        if (!gateway::CanSupplement(route, latest_board_us_, now)) continue;
                        auto frame = gateway::Query(route.bus, route.id, route.telemetry_parameter);
                        Prepare(frame);
                        serial_.Send(&frame, 1);
                        route.telemetry_parameter = route.telemetry_parameter == 2 ? 3 : 2;
                        route.telemetry_due_us = now + 100000;
                        break;
                    }
                }
                if (now >= next_status && !serial_.WriteBusy()) {
                    SendWait(gateway::Frame(3));
                    next_status = now + 1000000;
                }
                serial_.Wait(1, wake_);
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                diagnostics_.connected = 0;
                for (auto& route : routes_)
                    if (route.command.enabled || route.release.pending) route.release.Begin();
            }
            for (unsigned attempt = 0; attempt < 3; ++attempt) {
                for (auto& route : routes_) {
                    if (!route.release.NeedsSend()) continue;
                    auto sequence = SendWait(gateway::Release(route.bus, route.id));
                    route.release.Submitted(sequence, gateway::NowUs());
                }
            }
            DrainWrite();
            const auto settle = gateway::NowUs() + 20000;
            while (gateway::NowUs() < settle) { Pump(); serial_.Wait(1); }
            PollControl(0); PollControl(2); DrainWrite();
            for (const auto& route : routes_)
                if (route.release.failed) throw std::runtime_error("Soft release CAN failure on close, check physical motor state");
        } catch (const std::exception& error) {
            SetError(error.what());
            std::lock_guard<std::mutex> lock(mutex_);
            ++diagnostics_.serial_errors;
        }
        serial_.Close();
        std::lock_guard<std::mutex> lock(mutex_);
        diagnostics_.connected = 0;
        for (auto& route : routes_) { route.command.enabled = false; route.release.pending = false; }
    }
};
