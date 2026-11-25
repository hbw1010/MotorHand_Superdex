#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace motorhand::native {

inline constexpr std::size_t tree_dof_for(std::size_t glove_dof) {
    return glove_dof == 16 ? 80 : 68;
}

template <std::size_t G> using GloveVec = std::array<double, G>;
template <std::size_t G> using TreeVec = std::array<double, tree_dof_for(G)>;
template <std::size_t G> using PointJacobianT = std::array<double, 63 * G>;
template <std::size_t G> using MotorJacobianT = std::array<double, G * G>;

using Vec12 = std::array<double, 12>;
using Vec16 = std::array<double, 16>;
using Vec20 = std::array<double, 20>;
using Vec68 = std::array<double, 68>;
using Vec80 = std::array<double, 80>;
using Points = std::array<double, 63>;
using PointJacobian = PointJacobianT<12>;
using MotorJacobian = MotorJacobianT<12>;

// 只读分支记录，固定大小，避免控制/提交线程分配原因字符串。
enum class ImpedanceReason : std::uint64_t {
    inactive_geometry = 1ULL << 0, invalid_input = 1ULL << 1, invalid_geometry = 1ULL << 2,
    unobservable_geometry = 1ULL << 3, singular_transmission = 1ULL << 4, invalid_contact_row = 1ULL << 5,
    inverse_failed = 1ULL << 6, contact_pd = 1ULL << 7, free_axis = 1ULL << 8, coupled_release = 1ULL << 9,
    friction_disabled = 1ULL << 10, outside_proxy = 1ULL << 11, zero_stiffness = 1ULL << 12,
    planner_release_guard = 1ULL << 13, proxy_offset_limited = 1ULL << 14, proxy_rate_limited = 1ULL << 15,
    soft_limit_target = 1ULL << 16, soft_limit_release = 1ULL << 17, torque_budget_limited = 1ULL << 18,
    friction_limited = 1ULL << 19, sdk_gain_limited = 1ULL << 20, sdk_zero_output = 1ULL << 21,
    sdk_direction_release = 1ULL << 22, sdk_budget_release = 1ULL << 23, sdk_output_accepted = 1ULL << 24
};
struct DriverAxisObservation {
    bool torque_limited{}, motion_limited{}, proxy_limited{};
    std::uint64_t reasons{};
    void add(ImpedanceReason reason) { reasons |= static_cast<std::uint64_t>(reason); }
    void merge(const DriverAxisObservation& other) {
        torque_limited |= other.torque_limited; motion_limited |= other.motion_limited;
        proxy_limited |= other.proxy_limited; reasons |= other.reasons;
    }
};
template <std::size_t N, std::size_t G = 12> struct CommonDriverImpedanceObservation {
    GloveVec<G> proxy_surface_rad{}, input_rad{}, filtered_velocity_rad_s{}; // 模型电机轴。
    std::array<std::uint64_t, G> source_ns{};
    std::array<int, G> blocking_direction{};
    std::array<bool, G> proxy_valid{};
    std::array<bool, N> impedance_contact{}; // 实际纳入本次局部逆解的机械手行。
    std::array<DriverAxisObservation, G> axes{};
};

using DriverImpedanceObservation = CommonDriverImpedanceObservation<20, 12>;

inline constexpr double default_impedance_torque_margin = .85;

// MIT命令均为GLOVE_JOINTS模型电机轴；velocity仅用于提交时名义估算。
template <std::size_t G = 12> struct DriverImpedanceCommandT {
    GloveVec<G> target_rad{}, kp{}, kd{}, feedforward_nm{}, velocity_rad_s{};
    std::array<int, G> blocked_direction{};
    std::array<std::uint64_t, G> velocity_received_ns{};
    double torque_margin = default_impedance_torque_margin;
};
template <std::size_t G = 12> struct DriverImpedanceSubmissionT {
    DriverImpedanceCommandT<G> command{}; // 量化后实际提交参数，仍以模型轴表示。
    GloveVec<G> nominal_nm{}, pd_nm{}, friction_nm{};
    std::array<std::uint64_t, G> estimate_received_ns{};
    std::array<DriverAxisObservation, G> observation{};
};
using DriverImpedanceCommand = DriverImpedanceCommandT<12>;
using DriverImpedanceSubmission = DriverImpedanceSubmissionT<12>;
using DriverImpedanceCommand16 = DriverImpedanceCommandT<16>;
using DriverImpedanceSubmission16 = DriverImpedanceSubmissionT<16>;

enum class ContactPhase { not_configured, free, arming, contact, releasing, waiting_geometry };

// 数组顺序分别固定为项目 GLOVE_JOINTS、机械手关节与 CAD 模型树顺序。
// 电机位置为模型轴相对角 rad；原始编码器位置为 degree。时间为单调时钟真实收包 ns。
template <std::size_t G = 12> struct GloveFrameT {
    GloveVec<G> raw_degrees{};
    GloveVec<G> motor_rad{};
    GloveVec<G> measured_current_a{};
    std::array<std::uint64_t, G> received_ns{};
    std::uint64_t oldest_ns{};
    std::array<std::uint64_t, G> current_received_ns{}; // 电流可独立补包；0表示未取得有效电流。
};
using GloveFrame = GloveFrameT<12>;
using GloveFrame16 = GloveFrameT<16>;

// points 是腕部平移后的 canonical 21 点，单位 m；雅可比按行主序。
template <std::size_t G = 12> struct GlovePoseT {
    GloveVec<G> joint_rad{};
    GloveVec<G> motor_rad{};
    TreeVec<G> tree_rad{};
    Points points_m{};
    PointJacobianT<G> point_jacobian{};
    MotorJacobianT<G> motor_jacobian{};
    double max_gap_m{};
};
using GlovePose = GlovePoseT<12>;
using GlovePose16 = GlovePoseT<16>;

struct HandFrame {
    Vec20 position_rad{};
    Vec20 velocity_rad_s{};
    Vec20 effort_a{};
    bool has_velocity{};
    bool has_effort{};
    std::uint64_t received_ns{};
};

}  // namespace motorhand::native
