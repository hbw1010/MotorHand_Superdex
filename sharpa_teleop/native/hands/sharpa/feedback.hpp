#pragma once
#include "common/feedback.hpp"
#include "encos_driver.hpp"
#include "identity.hpp"
#include <filesystem>
namespace motorhand::native::sharpa {
// 仅使用SDK报告关节力矩；偏置/符号是反馈参数，绝不写入设备（并列支持 12-DOF 与 16-DOF 手套）。
template <std::size_t G = 12>
class FeedbackT final : public HandFeedback<22, G> {
public:
    using VecG = GloveVec<G>;
    FeedbackT(const std::filesystem::path& config_path,
              const std::filesystem::path& wuji_config_dir, bool linear_mode,
              const std::array<EncosMotorCalibration, G>&,
              std::string expected_source = torque_source,
              std::uint64_t startup_delay_ns = 0);
    void set_motor_limits(const VecG&, const VecG&) override;
    void reset(std::uint64_t now_ns) override;
    FeedbackResult<22, G> step(const JointArray<22>&, const HandState<22>&,
                               const GloveFrameT<G>&, const GlovePoseT<G>&,
                               const RetargetJacobian<22, G>&, const VecG&,
                               std::uint64_t, std::uint64_t) override;
    void acknowledge_torque(const VecG&) override;

private:
    void clear_axis(int);
    void clear_contact();
    JointArray<22> reference_{}, bias_{}, sign_{}, on_{}, off_{}, error_on_{},
        error_off_{}, stalled_{}, stiffness_{}, damping_{}, load_scale_{};
    JointArray<22> filtered_{}, last_position_{}, last_target_{}, contact_output_{};
    std::array<bool, 22> contact_{}, candidate_{}, releasing_{};
    std::array<int, 22> direction_{}, candidate_direction_{};
    std::array<std::uint64_t, 22> candidate_ns_{}, release_ns_{};
    VecG cap_{}, ack_cap_{}, slew_{}, lower_{}, upper_{}, history_{}, pending_output_{};
    double gain_{}, reflection_{}, ramp_{}, dwell_{}, release_dwell_{},
        filter_hz_{}, feedback_age_{}, jac_age_{}, jac_pose_delta_{},
        opening_deadband_{}, output_filter_hz_{}, max_gap_{}, relative_speed_limit_{};
    std::string expected_source_;
    bool linear_{}, limits_set_{}, reset_set_{}, have_source_{}, have_hand_{},
        pending_{}, geometry_active_{};
    std::uint64_t reset_ns_{}, source_ns_{}, hand_ns_{}, history_ns_{}, pending_ns_{};
    std::uint64_t startup_delay_ns_{}, mode_started_ns_{};
};

using Feedback = FeedbackT<12>;
using Feedback12 = FeedbackT<12>;
using Feedback16 = FeedbackT<16>;
} // namespace motorhand::native::sharpa
