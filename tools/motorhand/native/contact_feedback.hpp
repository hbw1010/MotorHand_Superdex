#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace ve33 {
template <size_t Dof>
using AxesT = std::array<double, Dof>;
using Axes = AxesT<12>;
using Axes12 = AxesT<12>;
using Axes16 = AxesT<16>;

template <size_t Dof>
class ContactFeedbackT {
public:
    static constexpr size_t dof = Dof;
    using AxesType = AxesT<Dof>;
    using Curve = std::vector<std::array<double, 2>>;
    ContactFeedbackT(AxesType limits, AxesType stiffness, AxesType position_limits, double deadband, double contact_min,
                     double blend_end, double full_load, double motor_damping, double rebound_lookahead,
                     Curve anchors = {})
        : limits_(limits), stiffness_(stiffness), position_limits_(position_limits),
          deadband_(deadband), contact_min_(contact_min),
          blend_end_(blend_end), full_load_(full_load), motor_damping_(motor_damping),
          rebound_lookahead_(rebound_lookahead) {
        if (!anchors.empty()) validate(anchors, *std::min_element(limits_.begin(), limits_.end()));
        curves_.fill(anchors);
        friction_blend_.fill(1.);
    }

    void set_axis_curve(int motor_id, Curve anchors) {
        if (motor_id < 1 || motor_id > static_cast<int>(limits_.size()))
            throw std::invalid_argument("反馈轴编号超出范围");
        const auto axis = static_cast<std::size_t>(motor_id - 1);
        validate(anchors, limits_[axis]);
        // 渐入后的接触项幅值可落在 0 到 blend_end 之间的任一点。
        for (const auto& [x, y] : anchors) {
            if (x <= blend_end_ && y != x)
                throw std::invalid_argument("逐轴曲线的轻触段必须保持原尺度");
        }
        if (anchors.back()[0] < blend_end_ || std::abs(map_contact(anchors, blend_end_) - blend_end_) > 1e-12)
            throw std::invalid_argument("逐轴曲线的轻触段必须保持原尺度");
        curves_[axis] = std::move(anchors);
    }

    void set_friction(bool enabled, AxesType coulomb, AxesType viscous, double v_scale,
                      double v_deadband, double gain, AxesType max_nm) {
        if (!std::isfinite(v_scale) || !std::isfinite(v_deadband) || !std::isfinite(gain) ||
            v_scale <= 0. || v_deadband < 0. || gain < 0.)
            throw std::invalid_argument("摩擦力补偿参数无效");
        for (std::size_t i = 0; i < Dof; ++i) {
            if (!std::isfinite(coulomb[i]) || !std::isfinite(viscous[i]) || !std::isfinite(max_nm[i]) ||
                coulomb[i] < 0. || viscous[i] < 0. || max_nm[i] < 0.)
                throw std::invalid_argument("逐轴摩擦力补偿参数无效");
        }
        friction_enabled_ = enabled;
        friction_coulomb_ = coulomb;
        friction_viscous_ = viscous;
        friction_v_scale_ = std::max(1e-4, v_scale);
        friction_v_deadband_ = v_deadband;
        friction_gain_ = gain;
        friction_max_ = max_nm;
    }

    void reset() {
        slew_.fill(0.);
        filtered_.fill(0.);
        mapped_.fill(0.);
        damping_.fill(0.);
        friction_.fill(0.);
        friction_blend_.fill(1.);
        gate_.fill(false);
    }

    const AxesType& mapped() const { return mapped_; }
    const AxesType& damping() const { return damping_; }
    const AxesType& friction() const { return friction_; }
    const std::array<bool, Dof>& gate() const { return gate_; }

    std::tuple<AxesType, AxesType, int> plan(const AxesType& q, const AxesType& dq, const AxesType& reaction,
                                             const AxesType& sim_pos, const AxesType& sampled_pos,
                                             const AxesType& frame_weight, double ramp, double dt) {
        AxesType output{}, position{};
        int loaded_count = 0;
        const double alpha = -std::expm1(-2. * 3.14159265358979323846 * 15. * dt);
        const double alpha_fric = -std::expm1(-2. * 3.14159265358979323846 * 20. * dt);
        constexpr double rebound_vel_scale = 0.55;
        constexpr double press_damping = 0.90;
        constexpr double slew_up_idle = 50.;
        constexpr double slew_up_press = 115.;
        constexpr double slew_down_rate = 160.;
        constexpr double max_contact_kd = 0.28;
        constexpr double kd_hold_band = 0.6;
        constexpr double friction_fade_in_s = 0.08;
        const double clamped_ramp = std::clamp(ramp, 0., 1.);
        for (std::size_t i = 0; i < q.size(); ++i) {
            gate_[i] = std::abs(reaction[i]) >= contact_min_;
            if (!gate_[i]) {
                mapped_[i] = 0.;
                damping_[i] = 0.;
                slew_[i] = filtered_[i] = 0.;
                double target_fric = 0.;
                if (friction_enabled_ && friction_gain_ > 0. && friction_blend_[i] > 0.) {
                    const double abs_v = std::abs(dq[i]);
                    if (abs_v > friction_v_deadband_) {
                        const double moving = std::copysign(abs_v - friction_v_deadband_, dq[i]);
                        const double raw_f = friction_gain_ * (
                            friction_coulomb_[i] * std::tanh(moving / friction_v_scale_) +
                            friction_viscous_[i] * moving);
                        target_fric = std::clamp(raw_f, -friction_max_[i], friction_max_[i]) *
                            clamped_ramp * friction_blend_[i];
                    }
                }
                friction_[i] += alpha_fric * (target_fric - friction_[i]);
                if (target_fric == 0. && std::abs(friction_[i]) < 1e-4) {
                    friction_[i] = 0.;
                }
                friction_blend_[i] = std::min(1., friction_blend_[i] + dt / friction_fade_in_s);
                output[i] = friction_[i];
                continue;
            }
            ++loaded_count;
            friction_[i] = 0.;
            friction_blend_[i] = 0.;
            const double limit = limits_[i];
            const double load_torque = std::clamp(reaction[i], -limit, limit);
            const double onset = std::clamp((std::abs(load_torque) - contact_min_) / (blend_end_ - contact_min_), 0., 1.);
            const double blend = onset * onset * (3. - 2. * onset);
            const double raw_contact = load_torque * blend;
            const double mapped = map_contact(curves_[i], std::abs(raw_contact));
            const double contact = std::copysign(mapped, raw_contact);
            mapped_[i] = contact;
            const double sampled_error = sim_pos[i] - sampled_pos[i];
            const double direction = std::copysign(1., sampled_error);
            const double speed = std::clamp(dq[i], -2., 2.);
            const double live_gap = direction * (sim_pos[i] - q[i]) - rebound_lookahead_ * std::max(direction * speed, 0.);
            const double error = direction * std::min(std::abs(sampled_error), std::max(live_gap, 0.));
            const double load = std::clamp((std::abs(load_torque) - contact_min_) / (full_load_ - contact_min_), 0., 1.);
            const double position_effort = std::copysign(std::max(std::abs(error) - deadband_, 0.), error) *
                stiffness_[i] * load * load * (3. - 2. * load);
            if (position_effort * load_torque > 0.) {
                const double position_budget = std::min(position_limits_[i], limit - std::abs(contact));
                position[i] = std::copysign(std::min(std::abs(position_effort), position_budget),
                                            position_effort);
            }
            const double sign_r = std::copysign(1., load_torque);
            const double retreat_speed = std::max(0., sign_r * dq[i]);
            const double press_speed = std::max(0., -sign_r * dq[i]);
            // 用力顶入反抗输出时增大压入阻尼并加快建力斜率；放松回撤时平滑衰减推力抑制弹回。
            const double damp_coeff = (motor_damping_ > 0. && press_speed > 0.) ?
                std::max(motor_damping_, press_damping) : motor_damping_;
            double damped = contact + position[i] - blend * damp_coeff * speed;
            if (retreat_speed > 0.) {
                damped /= (1. + retreat_speed / rebound_vel_scale);
            }
            // 速度阻尼抑制真机回弹，接触力仍决定输出方向。
            const double target = damped * load_torque > 0. ?
                std::clamp(damped, -limit, limit) * std::clamp(ramp, 0., 1.) * frame_weight[i] : 0.;
            position[i] *= frame_weight[i];
            if (frame_weight[i] < 1.) {
                const double cap = limit * frame_weight[i];
                slew_[i] = std::clamp(slew_[i], -cap, cap);
                filtered_[i] = std::clamp(filtered_[i], -cap, cap);
            }
            if (filtered_[i] * load_torque < 0.) {
                slew_[i] = filtered_[i] = 0.;
            }
            const bool unloading = std::abs(target) < std::abs(slew_[i]) && target * slew_[i] >= 0.;
            const double press_blend = std::clamp((press_speed - 0.08) / 0.35, 0., 1.);
            const double rate = unloading ? slew_down_rate :
                (slew_up_idle + press_blend * (slew_up_press - slew_up_idle));
            slew_[i] += std::clamp(target - slew_[i], -rate * dt, rate * dt);
            filtered_[i] += alpha * (slew_[i] - filtered_[i]);
            // 电机驱动器本地零延迟阻尼（安全范围 <= 0.3）：握持微抖时双向稳住，明确张手退离时退出。
            const double excess_retreat = std::max(0., retreat_speed - kd_hold_band);
            const double kd_fade = 1. / (1. + excess_retreat / 0.4);
            damping_[i] = filtered_[i] == 0. ? 0. :
                max_contact_kd * std::min(1., std::abs(filtered_[i]) / 0.8) * frame_weight[i] * kd_fade;
            output[i] = filtered_[i];
        }
        return {output, position, loaded_count};
    }

private:
    static void validate(const Curve& anchors, double limit) {
        if (anchors.size() < 2 || anchors[0][0] != 0. || anchors[0][1] != 0.)
            throw std::invalid_argument("Invalid feedback curve origin");
        for (std::size_t i = 0; i < anchors.size(); ++i) {
            const auto [x, y] = anchors[i];
            if (!std::isfinite(x) || !std::isfinite(y) || x < 0. || y < 0. || y > limit ||
                (i && (x <= anchors[i - 1][0] || y < anchors[i - 1][1])))
                throw std::invalid_argument("Invalid feedback curve anchor");
        }
    }

    static double map_contact(const Curve& curve, double magnitude) {
        if (curve.empty()) return magnitude;
        const auto upper = std::upper_bound(curve.begin(), curve.end(), magnitude,
            [](double x, const auto& point) { return x < point[0]; });
        if (upper == curve.end()) return curve.back()[1];
        if (upper == curve.begin()) return upper->at(1);
        const auto& a = *(upper - 1);
        const auto& b = *upper;
        return a[1] + (magnitude - a[0]) * (b[1] - a[1]) / (b[0] - a[0]);
    }

    AxesType limits_, stiffness_, position_limits_;
    double deadband_, contact_min_, blend_end_, full_load_, motor_damping_;
    double rebound_lookahead_;
    bool friction_enabled_ = false;
    AxesType friction_coulomb_{}, friction_viscous_{}, friction_max_{};
    double friction_v_scale_ = 0.09, friction_v_deadband_ = 0.025, friction_gain_ = 1.;
    std::array<Curve, Dof> curves_;
    AxesType slew_{}, filtered_{}, mapped_{}, damping_{}, friction_{};
    AxesType friction_blend_{};
    std::array<bool, Dof> gate_{};
};

using ContactFeedback = ContactFeedbackT<12>;
using ContactFeedback12 = ContactFeedbackT<12>;
using ContactFeedback16 = ContactFeedbackT<16>;
} // namespace ve33
