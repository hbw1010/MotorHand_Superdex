#pragma once
#include "encos_driver.hpp"
#include "types.hpp"
#include <string>
namespace motorhand::native {
// 参数在模型电机轴中计算；配置的逐轴数组以原始电机ID排列。
template <std::size_t G = 12>
struct DriverImpedanceConfigT {
    double stiffness_min = 5, stiffness_max = 60, gain = 1, damping_s = .008, damping_max = .5;
    GloveVec<G> joint_gain = [] { GloveVec<G> v; v.fill(1.0); v[0] = 1.5; return v; }();
    GloveVec<G> kp_max = [] { GloveVec<G> v; v.fill(80.0); v[0] = 100.0; return v; }();
    double lookahead_s = .02, damping_off_velocity = .15, damping_on_velocity = .05;
    double torque_margin = default_impedance_torque_margin;
    double surface_rate = 1, inverse_regularization = .01, max_proxy_offset = .15, ramp_s = .15;
    double surface_acceleration = 0; // rad/s²。0 沿用仅限速的目标更新。
    double velocity_filter_hz = 12, velocity_deadband = .02, velocity_scale = .08, friction_gain = 1, friction_max = .6;
    GloveVec<G> coulomb_nm = [] { GloveVec<G> v; v.fill(.25); v[1] = .10; return v; }();
    double viscous = 0;
    bool friction_enabled = true;
};
using DriverImpedanceConfig = DriverImpedanceConfigT<12>;
using DriverImpedanceConfig16 = DriverImpedanceConfigT<16>;

template <std::size_t G = 12>
DriverImpedanceConfigT<G> load_driver_impedance_config_for(const std::string& path);

inline DriverImpedanceConfig load_driver_impedance_config(const std::string& path) {
    return load_driver_impedance_config_for<12>(path);
}

struct DriverAxisCommand { double target_rad{}, kp{}, kd{}, feedforward_nm{}; };
// 总PD预算含阻尼与编码余量；只保证当前状态下的名义输出，MIT没有硬力矩上限。
template <std::size_t G = 12>
DriverAxisCommand plan_driver_axis(double q, double dq, double surface, int direction, double stiffness,
    double torque_limit, double quantization_reserve, const DriverImpedanceConfigT<G>&, bool& damping_on,
    DriverAxisObservation* observation = nullptr);

template <std::size_t G = 12>
class MotorVelocityEstimatorT {
public:
    explicit MotorVelocityEstimatorT(double cutoff_hz);
    GloveVec<G> update(const GloveFrameT<G>&, std::uint64_t now_ns);
    void reset();
private:
    double cutoff_;
    GloveVec<G> q_{}, velocity_{};
    std::array<std::uint64_t, G> stamps_{};
};
using MotorVelocityEstimator = MotorVelocityEstimatorT<12>;
using MotorVelocityEstimator16 = MotorVelocityEstimatorT<16>;
}
