#include "feedback_bindings.hpp"
#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>
#include <mochi_physics/cpp_api/mochi_scene.h>
#include "kinematics.hpp"
#include "shared_link.hpp"
#include "grasp_motion.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
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

std::int64_t steady_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

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
          velocity_(pose_.size()), target_(pose_.size()), frame_(3 + 5 * motors.size() + 1) {
        const auto valid_dof = [this](int i) { return i >= 0 && std::size_t(i) < pose_.size(); };
        if (!std::all_of(tree_.begin(), tree_.end(), valid_dof) ||
            !std::all_of(motors_.begin(), motors_.end(), valid_dof) || palm_index_ < -1 ||
            palm_index_ >= static_cast<int>(links_.size())) {
            throw std::invalid_argument("关节索引与 MotorHand 模型不匹配");
        }
        for (std::size_t i = 0; i < motors_.size(); ++i) {
            const auto it = std::find(tree_.begin(), tree_.end(), motors_[i]);
            if (it == tree_.end()) throw std::invalid_argument("关节树中缺少电机轴");
            motor_rows_[i] = static_cast<std::size_t>(it - tree_.begin());
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
        advance_scene(dt);
    }

    // 每个子步读取一张完整输入快照。场景约束不动时，一次调用可连续完成预算内的子步。
    std::tuple<bool, double> step_contact(ve33::SharedLink& link, ve33::Kinematics& kin,
                                         double dt, std::int64_t budget_end, unsigned max_steps,
                                         bool publish_last, std::uint64_t revision) {
        nb::gil_scoped_release release;
        return advance_contact(link,kin,dt,budget_end,max_steps,publish_last,revision);
    }

    // 在创建 Mochi 场景的线程上运行。只在下一次场景命令/画面交接边界回到 Python。
    double run_until(ve33::SharedLink& link, ve33::Kinematics& kin, double dt,
                     std::int64_t period, std::int64_t view_deadline, std::uint64_t revision) {
        if (period<=0 || period>100'000'000 || view_deadline-steady_ns()>100'000'000)
            throw std::invalid_argument("无效的物理调度预算");
        nb::gil_scoped_release release;
        do {
            if (next_tick_ns_ > steady_ns()) {
                using namespace std::chrono;
                std::this_thread::sleep_until(steady_clock::time_point(nanoseconds(next_tick_ns_)));
            }
            ve33::Intent intent;
            if (link.read_intent(intent) && intent.revision != static_cast<std::int64_t>(revision)) {
                frame_[2] = 0.;
                link.invalidate_force();
                return 0.;
            }
            const auto now=steady_ns();
            next_tick_ns_=now+period;
            advance_contact(link,kin,dt,next_tick_ns_,4,true,revision);
        } while (std::max(steady_ns(),next_tick_ns_) < view_deadline);
        return last_target_[0];
    }

    void set_root_target(int trans_offset, int rot_offset, const std::array<double, 6>& root) {
        if (trans_offset < 0 || rot_offset < 0 ||
            static_cast<std::size_t>(trans_offset + 3) > pose_.size() ||
            static_cast<std::size_t>(rot_offset + 3) > pose_.size() ||
            !std::all_of(root.begin(), root.end(), [](double x) {
                return std::isfinite(x) && std::abs(x) <= std::numeric_limits<real>::max();
            })) {
            throw std::invalid_argument("无效的浮动腕部目标");
        }
        has_root_target_ = true;
        root_trans_offset_ = trans_offset;
        root_rot_offset_ = rot_offset;
        root_target_ = root;
    }

    bool publish_contact(ve33::SharedLink& link, ve33::Kinematics& kin, double stamp,
                         const Axes& sampled_input, const Axes& encoder_sample, std::uint64_t revision) {
        nb::gil_scoped_release release;
        const auto solved = steady_ns();
        const auto finite = [](const auto& a) {
            return std::all_of(a.begin(), a.end(), [](double x) { return std::isfinite(x); });
        };
        bool valid = std::isfinite(stamp) && stamp > 0. && stamp <= static_cast<double>(solved) &&
                     finite(sampled_input) && finite(encoder_sample);
        Axes positions{}, speeds{}, torque{};
        if (valid) {
            try {
                auto [usable, q, dq, tree_torque] = sample_effort();
                positions = q;
                speeds = dq;
                valid = usable && finite(q) && finite(dq) && finite(tree_torque);
                if (valid) {
                    torque = kin.project(tree_torque, q);
                    valid = finite(torque);
                }
            } catch (const std::runtime_error&) {
                valid = false;
            } catch (const std::invalid_argument&) {
                valid = false;
            }
        }
        frame_[0] = std::isfinite(stamp) ? stamp : 0.;
        frame_[1] = static_cast<double>(solved);
        auto cursor = frame_.begin() + 3;
        cursor = std::copy(torque.begin(), torque.end(), cursor);
        cursor = std::copy(speeds.begin(), speeds.end(), cursor);
        cursor = std::copy(positions.begin(), positions.end(), cursor);
        cursor = std::copy(sampled_input.begin(), sampled_input.end(), cursor);
        cursor = std::copy(encoder_sample.begin(), encoder_sample.end(), cursor);
        frame_.back() = static_cast<double>(revision);
        frame_[2] = valid;
        link.publish_force(frame_);
        const auto published = steady_ns();
        last_timing_ms_ = {valid ? (solved - frame_[0]) / 1e6 : -1., 0., (published - solved) / 1e6};
        return valid;
    }

    void set_motion(std::vector<Actor*> rigid,std::vector<Actor*> fixed,std::vector<Actor*> surfaces,
                    std::vector<Actor*> soft,int center_node,Force center,Force axis,double angle) {
        motion_=std::make_unique<ve33::GraspMotion>(
            scene_,std::move(rigid),std::move(fixed),std::move(surfaces),std::move(soft),
            center_node,center,axis,angle);
    }
    void clear_motion() {motion_.reset();}
    void motion_command(bool locked,double angle,const std::vector<double>& drag) {
        if (!motion_) throw std::runtime_error("没有本地抓握物体");
        motion_->command(locked,angle,drag);
    }
    Force motion_center() const {
        if (!motion_) throw std::runtime_error("没有本地抓握物体");
        return motion_->center();
    }
private:
    std::tuple<bool,double> advance_contact(ve33::SharedLink& link,ve33::Kinematics& kin,
                                         double dt,std::int64_t budget_end,unsigned max_steps,
                                         bool publish_last,std::uint64_t revision) {
        if (max_steps < 1 || max_steps > 4 || !std::isfinite(dt) || dt <= 0.)
            throw std::invalid_argument("无效的物理子步参数");
        mochi::Error error;
        hand_.GetArticulatedPose(target_, error);
        check(error);
        std::vector<double> sampled;
        for (unsigned step = 1; step <= max_steps; ++step) {
            if (motion_) motion_->step(dt);
            const auto began = steady_ns();
            bool valid = false;
            if (link.try_read_target(sampled)) {
                if (sampled.size() != 1 + tree_.size() + motors_.size())
                    throw std::invalid_argument("编码器共享快照长度与关节树不匹配");
                valid = sampled[0] > 0. && sampled[0] <= static_cast<double>(began) &&
                        std::all_of(sampled.begin()+1, sampled.end(), [](double x) {
                            return std::isfinite(x) && std::abs(x) <= std::numeric_limits<real>::max();
                        });
                if (valid) {
                    last_target_ = std::move(sampled);
                    for (std::size_t i = 0; i < tree_.size(); ++i)
                        target_[tree_[i]] = static_cast<real>(last_target_[1+i]);
                } else {
                    // 未收到完整快照时不施加可能撕裂的目标，也不给上一目标续期。
                    if (last_target_.empty()) last_target_.resize(sampled.size());
                    last_target_[0] = 0.;
                }
            } else if (!last_target_.empty() && last_target_[0] > 0. &&
                       last_target_[0] <= static_cast<double>(began)) {
                // 极罕见的 seqlock 并发写入抢占：沿用上一有效目标及其原始时戳，避免误发无效帧触发保护。
                valid = true;
            } else {
                if (last_target_.empty()) last_target_.resize(1 + tree_.size() + motors_.size());
                last_target_[0] = 0.;
            }
            if (has_root_target_) {
                for (int i = 0; i < 3; ++i) {
                    target_[root_trans_offset_ + i] = static_cast<real>(root_target_[i]);
                    target_[root_rot_offset_ + i] = static_cast<real>(root_target_[3 + i]);
                }
            }
            advance_scene(dt);
            const auto solved = steady_ns();
            const auto step_ns = std::max<std::int64_t>(0, solved - began);
            const bool finish_early = publish_last && (
                (budget_end > 0 && solved + step_ns > budget_end) ||
                (valid && (solved - static_cast<std::int64_t>(last_target_[0])) >= 35'000'000));
            if (solved < budget_end && !finish_early && !(step == max_steps && publish_last)) continue;
            Axes positions{}, speeds{}, torque{};
            if (valid) {
                try {
                    auto [usable, q, dq, tree_torque] = sample_effort();
                    positions = q;
                    speeds = dq;
                    const auto finite = [](const auto& a) {
                        return std::all_of(a.begin(), a.end(), [](double x) { return std::isfinite(x); });
                    };
                    valid = usable && finite(q) && finite(dq) && finite(tree_torque);
                    if (valid) {
                        torque = kin.project(tree_torque, q);
                        valid = finite(torque);
                    }
                } catch (const std::runtime_error&) {
                    valid = false;
                } catch (const std::invalid_argument&) {
                    valid = false;
                }
            }
            frame_[0] = last_target_[0];
            frame_[1] = static_cast<double>(solved);
            auto cursor = frame_.begin() + 3;
            cursor = std::copy(torque.begin(), torque.end(), cursor);
            cursor = std::copy(speeds.begin(), speeds.end(), cursor);
            cursor = std::copy(positions.begin(), positions.end(), cursor);
            for (const auto row : motor_rows_) *cursor++ = last_target_[1 + row];
            for (std::size_t i = 0; i < motors_.size(); ++i) {
                *cursor++ = last_target_[1 + tree_.size() + i];
            }
            frame_.back() = static_cast<double>(revision);
            frame_[2] = valid;
            link.publish_force(frame_);
            const auto published = steady_ns();
            last_timing_ms_ = {valid ? (began - frame_[0]) / 1e6 : -1.,
                               (solved - began) / 1e6, (published - solved) / 1e6};
            return {true, frame_[0]};
        }
        return {false, last_target_[0]};
    }

public:
    const std::vector<double>& last_target() const { return last_target_; }
    double last_contact_frame_ns() const { return frame_[2] == 1. ? frame_[1] : 0.; }
    const std::array<double, 3>& last_timing_ms() const { return last_timing_ms_; }

private:
    void advance_scene(double dt) {
        mochi::Error error;
        hand_.ResetArticulatedTargetPose(target_, error);
        check(error);
        scene_.Step(dt);
    }

public:
    std::tuple<bool, Axes, Axes, std::vector<double>> effort() {
        nb::gil_scoped_release release;
        return sample_effort();
    }

private:
    std::tuple<bool, Axes, Axes, std::vector<double>> sample_effort() {
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

public:
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
    std::array<std::size_t, 12> motor_rows_{};
    std::vector<double> last_target_, frame_;
    std::array<double, 3> last_timing_ms_{};
    std::int64_t next_tick_ns_{};
    bool has_root_target_ = false;
    int root_trans_offset_ = 0, root_rot_offset_ = 0;
    std::array<double, 6> root_target_{};
    std::unique_ptr<ve33::GraspMotion> motion_;
};

} // namespace

void bind_native_loop(nanobind::module_&);
NB_MODULE(_ve33_native, module) {
    module.attr("runtime_api") = 5;
    bind_kinematics(module);
    bind_native_loop(module);
    module.attr("double_precision") = bool(MOCHI_USE_DOUBLE_PRECISION);
    nb::class_<GraspPhysics>(module, "GraspPhysics")
        .def(nb::init<mochi::Scene&, Actor&, std::vector<Actor*>, std::vector<int>, std::array<int, 12>, int>(),
             nb::keep_alive<1, 2>(), nb::keep_alive<1, 3>())
        .def("step", &GraspPhysics::step)
        .def("step_contact", &GraspPhysics::step_contact)
        .def("run_until", &GraspPhysics::run_until)
        .def("set_root_target", &GraspPhysics::set_root_target)
        .def("publish_contact", &GraspPhysics::publish_contact)
        .def("set_motion",&GraspPhysics::set_motion)
        .def("clear_motion",&GraspPhysics::clear_motion)
        .def("motion_command",&GraspPhysics::motion_command)
        .def("motion_center",&GraspPhysics::motion_center)
        .def("last_target", &GraspPhysics::last_target)
        .def("last_contact_frame_ns", &GraspPhysics::last_contact_frame_ns)
        .def("last_timing_ms", &GraspPhysics::last_timing_ms)
        .def("effort", &GraspPhysics::effort)
        .def("contacts", &GraspPhysics::contacts);
    ve33::bind_feedback(module);

}
