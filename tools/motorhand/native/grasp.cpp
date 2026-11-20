#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>
#include <mochi_physics/cpp_api/mochi_scene.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace {
namespace nb = nanobind;
using mochi::Actor;
using mochi::real;
using Axes = std::array<double, 12>;
using Force = std::array<double, 3>;

void check(const mochi::Error& error) {
    if (!error.IsOK()) {
        throw std::runtime_error(error.GetDescription());
    }
}

class GraspPhysics {
public:
    GraspPhysics(mochi::Scene& scene, Actor& hand, std::vector<Actor*> links,
                 std::vector<int> tree, std::array<int, 12> motors, int palm_index)
        : scene_(scene), hand_(hand), links_(std::move(links)), tree_(std::move(tree)),
          motors_(motors), palm_index_(palm_index), pose_(hand.GetNumDofs()),
          velocity_(pose_.size()), target_(pose_.size()) {
        const auto valid_dof = [this](int i) { return i >= 0 && std::size_t(i) < pose_.size(); };
        if (!std::all_of(tree_.begin(), tree_.end(), valid_dof) ||
            !std::all_of(motors_.begin(), motors_.end(), valid_dof) || palm_index_ < -1 ||
            palm_index_ >= static_cast<int>(links_.size())) {
            throw std::invalid_argument("关节索引与 MotorHand 模型不匹配");
        }
    }

    void step(const std::vector<double>& target, double dt) {
        if (target.size() != target_.size() || !std::isfinite(dt) || dt <= 0. ||
            !std::all_of(target.begin(), target.end(), [](double q) {
                return std::isfinite(q) && std::abs(q) <= std::numeric_limits<real>::max();
            })) {
            throw std::invalid_argument("无效的固定腕部关节目标");
        }
        nb::gil_scoped_release release;
        std::copy(target.begin(), target.end(), target_.begin());
        mochi::Error error;
        hand_.ResetArticulatedTargetPose(target_, error);
        check(error);
        scene_.Step(dt);
    }

    std::tuple<bool, Axes, Axes, std::vector<double>> effort() {
        nb::gil_scoped_release release;
        Axes positions{}, speeds{};
        std::vector<double> tree_torque(tree_.size());
        mochi::Error error;
        hand_.GetArticulatedPose(pose_, error);
        hand_.GetArticulatedJointVelocities(velocity_, error);
        check(error);
        for (std::size_t i = 0; i < motors_.size(); ++i) {
            positions[i] = pose_[motors_[i]];
            speeds[i] = velocity_[motors_[i]];
        }
        if (scene_.GetSolverStats().convergenceStatus == mochi::ConvergenceStatus::Diverged) {
            return {false, positions, speeds, std::move(tree_torque)};
        }
        for (std::size_t link = 0; link < links_.size(); ++link) {
            if (static_cast<int>(link) == palm_index_) {
                continue;
            }
            const auto force = links_[link]->GetContactForceWorld(error);
            const auto moment = links_[link]->GetContactTorqueWorld(error);
            check(error);
            const std::array<double, 6> wrench{force[0], force[1], force[2], moment[0], moment[1], moment[2]};
            if (std::all_of(wrench.begin(), wrench.end(), [](double x) { return x == 0.; })) {
                continue;
            }
            const auto jac = links_[link]->GetArticulatedJacobian(error);
            check(error);
            if (jac.size() != 6 * pose_.size()) {
                throw std::runtime_error("SuperDex 关节雅可比尺寸不匹配");
            }
            for (std::size_t i = 0; i < tree_.size(); ++i) {
                for (std::size_t row = 0; row < wrench.size(); ++row) {
                    tree_torque[i] += wrench[row] * jac[row * pose_.size() + tree_[i]];
                }
            }
        }
        return {true, positions, speeds, std::move(tree_torque)};
    }

    std::vector<std::vector<Force>> contacts(const std::vector<Actor*>& surfaces) const {
        nb::gil_scoped_release release;
        std::vector<std::vector<Force>> result(surfaces.size());
        mochi::Error error;
        for (std::size_t surface = 0; surface < surfaces.size(); ++surface) {
            auto& row = result[surface];
            row.reserve(links_.size());
            for (auto* link : links_) {
                const auto force = link->GetContactForceFromActorWorld(surfaces[surface], error);
                check(error);
                row.push_back({force[0], force[1], force[2]});
            }
        }
        return result;
    }

private:
    mochi::Scene& scene_;
    Actor& hand_;
    std::vector<Actor*> links_;
    std::vector<int> tree_;
    std::array<int, 12> motors_;
    int palm_index_;
    std::vector<real> pose_, velocity_, target_;
};

class ContactFeedback {
public:
    ContactFeedback(Axes limits, Axes stiffness, Axes position_limits, double deadband, double contact_min,
                    double blend_end, double full_load, double motor_damping, double rebound_lookahead)
        : limits_(limits), stiffness_(stiffness), position_limits_(position_limits),
          deadband_(deadband), contact_min_(contact_min),
          blend_end_(blend_end), full_load_(full_load), motor_damping_(motor_damping),
          rebound_lookahead_(rebound_lookahead) {}

    std::tuple<Axes, Axes, int> plan(const Axes& q, const Axes& dq, const Axes& reaction,
                                    const Axes& sim_pos, const Axes& sampled_pos, const Axes& frame_weight,
                                    double ramp, double dt) {
        Axes position{};
        int loaded_count = 0;
        const double alpha = -std::expm1(-2. * 3.14159265358979323846 * 25. * dt);
        for (std::size_t i = 0; i < q.size(); ++i) {
            if (std::abs(reaction[i]) < contact_min_) {
                slew_[i] = filtered_[i] = 0.;
                continue;
            }
            ++loaded_count;
            const double limit = limits_[i];
            const double load_torque = std::clamp(reaction[i], -limit, limit);
            const double onset = std::clamp((std::abs(load_torque) - contact_min_) / (blend_end_ - contact_min_), 0., 1.);
            const double blend = onset * onset * (3. - 2. * onset);
            const double contact = load_torque * blend;
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
            const double damped = contact + position[i] - blend * motor_damping_ * speed;
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
            slew_[i] += std::clamp(target - slew_[i], -60. * dt, 60. * dt);
            filtered_[i] += alpha * (slew_[i] - filtered_[i]);
        }
        return {filtered_, position, loaded_count};
    }

private:
    Axes limits_, stiffness_, position_limits_;
    double deadband_, contact_min_, blend_end_, full_load_, motor_damping_;
    double rebound_lookahead_;
    Axes slew_{}, filtered_{};
};
} // namespace

NB_MODULE(_ve33_native, module) {
    module.attr("double_precision") = bool(MOCHI_USE_DOUBLE_PRECISION);
    nb::class_<GraspPhysics>(module, "GraspPhysics")
        .def(nb::init<mochi::Scene&, Actor&, std::vector<Actor*>, std::vector<int>, std::array<int, 12>, int>(),
             nb::keep_alive<1, 2>(), nb::keep_alive<1, 3>())
        .def("step", &GraspPhysics::step)
        .def("effort", &GraspPhysics::effort)
        .def("contacts", &GraspPhysics::contacts);
    nb::class_<ContactFeedback>(module, "ContactFeedback")
        .def(nb::init<Axes, Axes, Axes, double, double, double, double, double, double>())
        .def("plan", &ContactFeedback::plan);
}
