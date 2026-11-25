#pragma once
#include "common/hand.hpp"
namespace motorhand::native {
template <std::size_t N, std::size_t G = 12> struct FeedbackResult {
    JointArray<N> virtual_hand_nm{}; // Software impedance, not measured actuator torque.
    JointArray<N> load_fraction{}; // Device-normalized proxy, not contact force.
    GloveVec<G> model_motor_nm{};  // Nominal model-axis torque.
    std::array<bool, N> contact{};
    // contact含释放迟滞；只有此逐轴mask允许施加阻抗。回开/释放立即撤销。
    std::array<bool, N> impedance_contact{};
    std::array<int, N> contact_direction{};
    bool active{};           // Inputs and local geometry are valid, not a contact indication.
    bool linear{};
    ContactPhase phase{ContactPhase::not_configured};
    GloveVec<G> projected_motor_nm{}; // 未爬升/限幅/斜率处理的模型电机目标，仅用于遥测。
    JointArray<N> filtered_load{};
    double reflection_gain{};
    RetargetJacobian<N, G> hand_to_motor{}; // GxN行主序，未乘反射增益。
    JointArray<N> projection_load{};
    std::uint64_t projection_received_ns{};
};

template <std::size_t N, std::size_t G = 12> class HandFeedback {
public:
    virtual ~HandFeedback() = default;
    virtual void set_motor_limits(const GloveVec<G>&, const GloveVec<G>&) = 0;
    virtual void reset(std::uint64_t now_ns) = 0;
    virtual FeedbackResult<N, G> step(const JointArray<N>&, const HandState<N>&, const GloveFrameT<G>&,
        const GlovePoseT<G>&, const RetargetJacobian<N, G>&, const GloveVec<G>&,
        std::uint64_t, std::uint64_t) = 0;
    virtual void acknowledge_torque(const GloveVec<G>&) = 0;
};
}
