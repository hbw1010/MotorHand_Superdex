#pragma once
#include "common/session_options.hpp"
#include "common/hand.hpp"
#include "common/glove_device.hpp"
#include "common/feedback.hpp"
namespace motorhand::native {
template <std::size_t N, std::size_t G = 12> struct ControlTick {
    HandState<N> hand{};
    GloveFrameT<G> glove{};
    JointArray<N> hand_target{};
    GloveVec<G> glove_target{};  // MIT model position in inference; nominal torque in takeover.
    bool takeover{};
    bool inverse_fresh{};
    bool force_geometry_valid{};
    std::uint64_t target_deadline_ns{};
    GlovePoseT<G> glove_pose{};
    JointArray<N> virtual_hand_nm{};  // Software virtual torque, not a measurement.
    std::array<bool, N> contact{};
    ContactPhase contact_phase = ContactPhase::not_configured;
    bool force_commanded{};
    std::uint64_t jacobian_source_ns{};
    std::uint64_t glove_output_source_ns{};
    std::uint64_t cycle_started_ns{};
    std::uint64_t hand_submit_ns{};
    std::uint64_t glove_submit_ns{};
    std::uint64_t cycle_completed_ns{};
    bool linear_force{};
    JointArray<N> load_fraction{};
    HandState<N> feedback_input_hand{};
    JointArray<N> feedback_input_target{};
    std::uint64_t feedback_request_id{};
    GloveVec<G> glove_feedforward_nm{};
    GloveVec<G> glove_projected_nm{}, glove_requested_nm{};
    JointArray<N> filtered_load{};
    double contact_reflection_gain{};
    RetargetJacobian<N, G> hand_to_motor{};
    JointArray<N> projection_load{};
    std::uint64_t projection_received_ns{};
    bool driver_impedance{};
    GloveVec<G> glove_friction_nm{}, glove_driver_pd_nm{};
    GloveVec<G> glove_driver_target_rad{}, glove_driver_kp{}, glove_driver_kd{};
    double feedback_hardness{};
    std::array<std::uint64_t, G> glove_driver_estimate_received_ns{}, glove_driver_velocity_received_ns{};
    CommonDriverImpedanceObservation<N, G> glove_impedance_observation{};
    std::uint64_t contact_evaluated_ns{}, impedance_planned_ns{};
};

template <std::size_t N, std::size_t G = 12> class ControlSession {
public:
    ControlSession(ControlOptionsT<G>, std::unique_ptr<HandDevice<N>>, std::unique_ptr<HandModel<N>>,
                   std::unique_ptr<HandFeedback<N, G>> feedback = nullptr,
                   std::unique_ptr<GloveDeviceT<G>> injected_glove = {});
    ~ControlSession();
    ControlSession(const ControlSession&) = delete;
    ControlSession& operator=(const ControlSession&) = delete;
    const HandDescriptor<N>& connect();
    const HandDescriptor<N>& descriptor() const;
    HandState<N> read_hand_state();
    void enable();
    ControlTick<N, G> step_inference(const JointArray<N>&, std::uint64_t source_ns);
    ControlTick<N, G> step_takeover();
    void request_takeover(bool);
    void stop();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
