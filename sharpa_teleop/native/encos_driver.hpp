#pragma once

#include "types.hpp"
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace motorhand::native {

// Indexed in GLOVE_JOINTS order; torque caps are nominal, not measured output.
struct EncosMotorCalibration {
    int raw_motor_id{};
    double zero_deg{};
    double sign{};
    double scale{};
    double raw_min_deg{};
    double raw_max_deg{};
    double model_min_rad{};
    double model_max_rad{};
    double raw_torque_limit_nm{};
    bool has_torque_limit{};
};

template <std::size_t G = 12>
class EncosDriverT {
public:
    static constexpr std::size_t glove_dof = G;
    using VecG = GloveVec<G>;
    using Frame = GloveFrameT<G>;
    using ImpedanceCmd = DriverImpedanceCommandT<G>;
    using ImpedanceSub = DriverImpedanceSubmissionT<G>;

    explicit EncosDriverT(const std::string& mapping_path,
                          const std::string& force_limits_path = {});
#ifdef MOTORHAND_ENCOS_OFFLINE_TEST
    // Explicit PTY-only injection, unavailable in production builds.
    struct OfflineTestPorts { std::array<std::string, 3> paths; };
    EncosDriverT(const std::string& mapping_path, const std::string& force_limits_path,
                 const OfflineTestPorts& ports);
#endif
    ~EncosDriverT();
    EncosDriverT(const EncosDriverT&) = delete;
    EncosDriverT& operator=(const EncosDriverT&) = delete;

    void connect();
    Frame read();
    std::array<EncosMotorCalibration, G> calibration() const;
    std::array<std::uint64_t, G> last_command_write_ns() const;
    void home(double tolerance_rad = 0.15, double timeout_s = 45.0);
    void mark_homed();
    void enable_force();
    VecG write_position(const VecG& motor_rad, std::uint64_t deadline_ns);
    VecG write_torque(const VecG& nominal_nm, std::uint64_t deadline_ns);
    ImpedanceSub write_impedance(const ImpedanceCmd&, std::uint64_t deadline_ns);
    void disable();
    void close();

    void set_slot(std::chrono::microseconds slot);
    std::chrono::microseconds slot() const;
    void set_position_kp(double kp_nm_per_rad);
    double position_kp() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

using EncosDriver = EncosDriverT<12>;
using EncosDriver12 = EncosDriverT<12>;
using EncosDriver16 = EncosDriverT<16>;

}  // namespace motorhand::native
