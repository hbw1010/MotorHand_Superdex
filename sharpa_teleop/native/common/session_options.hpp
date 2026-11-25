#pragma once
#include "common/driver_impedance_config.hpp"
#include <atomic>
#include <cmath>
#include <stdexcept>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
namespace motorhand::native {
template <std::size_t G = 12>
struct ControlOptionsT {
    std::filesystem::path asset_root;
    std::filesystem::path glove_mapping;
    std::filesystem::path linear_feedback;
    bool glove_output = true;
    bool force_feedback = true;
    bool linear_force_feedback = false;
    bool driver_impedance_feedback = false;
    std::filesystem::path driver_impedance_config;
    std::optional<DriverImpedanceConfigT<G>> driver_impedance_override;
    double feedback_hardness = 0.3;
    bool left = false;
    bool zero_aligned_start = false;
    bool home_hand_on_enable = false;
    bool skip_glove_home = false;
    const std::atomic<bool>* stop_requested = nullptr;
    std::uint64_t transition_ns = 500'000'000;
    bool inverse_follows_command = true;
    double takeover_offset_motion_rad = 0.10;
    std::uint64_t takeover_offset_tau_ns = 0;
    double takeover_motor_deadband_rad = 0.0;
};
using ControlOptions = ControlOptionsT<12>;
using ControlOptions16 = ControlOptionsT<16>;

template <std::size_t G>
inline void validate_control_options(const ControlOptionsT<G>& options) {
    if (options.force_feedback && !options.glove_output)
        throw std::invalid_argument("force feedback requires glove output");
    if (options.zero_aligned_start && !options.glove_output)
        throw std::invalid_argument("zero-aligned start requires glove output for homing");
    if (options.driver_impedance_feedback && (!options.force_feedback || options.linear_force_feedback))
        throw std::invalid_argument("driver impedance requires contact feedback");
    if (!std::isfinite(options.feedback_hardness) || options.feedback_hardness < 0 || options.feedback_hardness > 1)
        throw std::invalid_argument("feedback hardness must be in [0,1]");
    if (!options.transition_ns) throw std::invalid_argument("transition duration must be positive");
    if (!std::isfinite(options.takeover_motor_deadband_rad) || options.takeover_motor_deadband_rad < 0)
        throw std::invalid_argument("takeover motor deadband must be finite and nonnegative");
}
}
