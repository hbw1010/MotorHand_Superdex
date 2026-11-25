#pragma once
#include "common/hand.hpp"
#include "types.hpp"

#include <array>
#include <limits>
#include <memory>
#include <string>

namespace motorhand::native {

template <std::size_t G> class GloveModelT;

template <std::size_t G = 12> struct InverseResultT {
    GlovePoseT<G> pose{};
    double task_error_max_mm{};
    double task_error_rms_mm{};
    int iterations{};
    bool success{};
    int optimizer_status{};  // NLopt status, or zero if optimization was not started.
    int optimizer_attempts{};
    std::array<int, 3> attempt_status{};
    bool kkt_converged{};
    double kkt_stationarity = std::numeric_limits<double>::infinity();
    double kkt_complementarity = std::numeric_limits<double>::infinity();
    double objective_change = std::numeric_limits<double>::infinity();
    std::string failure_reason;
};
using InverseResult = InverseResultT<12>;
using InverseResult16 = InverseResultT<16>;

template <std::size_t N, std::size_t G = 12> class CommonInverseRetarget {
public:
    CommonInverseRetarget(GloveModelT<G>& glove, HandModel<N>& hand,
                          std::array<std::array<double, 2>, G> motor_bounds, bool scale_aware = true);
    ~CommonInverseRetarget();
    CommonInverseRetarget(const CommonInverseRetarget&) = delete;
    CommonInverseRetarget& operator=(const CommonInverseRetarget&) = delete;
    InverseResultT<G> solve(const JointArray<N>& actual_hand, const GlovePoseT<G>& measured_seed);

private:
    struct Optimizer;
    std::unique_ptr<Optimizer> optimizer_;
    GloveModelT<G>& glove_;
    HandModel<N>& hand_;
    std::array<std::array<double, 2>, G> motor_bounds_;
    std::array<double, 21> scales_{};
    GlovePoseT<G> warm_pose_{};
    Points warm_target_{};
    GloveVec<G> warm_anchor_{};
    bool has_warm_pose_ = false;
};

}  // namespace motorhand::native
