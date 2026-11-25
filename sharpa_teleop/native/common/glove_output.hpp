#pragma once
#include "common/hand.hpp"
#include "common/glove_device.hpp"
#include "common/feedback.hpp"
#include "common/jacobian_worker.hpp"
#include "types.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace motorhand::native {
template <std::size_t G> class EncosDriverT;
template <std::size_t N, std::size_t G> class CommonDriverImpedancePlanner;

enum class GloveOutputMode { position, force };
template <std::size_t N, std::size_t G = 12> struct CommonGloveOutputResult {
    GloveOutputMode mode{};
    std::uint64_t request_id{};
    std::uint64_t source_ns{};
    std::uint64_t jacobian_source_ns{};
    std::uint64_t last_write_ns{};
    GloveVec<G> applied{};
    FeedbackResult<N, G> force{};
    bool force_computed{};
    HandState<N> feedback_input_hand{};
    JointArray<N> feedback_input_target{};
    bool driver_impedance{};
    DriverImpedanceSubmissionT<G> impedance{};
    CommonDriverImpedanceObservation<N, G> impedance_observation{};
    std::uint64_t contact_evaluated_ns{}, impedance_planned_ns{};
};

template <std::size_t N, std::size_t G = 12> class CommonGloveOutputWorker {
public:
    CommonGloveOutputWorker(GloveDeviceT<G>& driver, HandFeedback<N, G>* force,
                            CommonDriverImpedancePlanner<N, G>* impedance = nullptr);
    CommonGloveOutputWorker(EncosDriverT<G>& driver, HandFeedback<N, G>* force,
                            CommonDriverImpedancePlanner<N, G>* impedance = nullptr);
    ~CommonGloveOutputWorker();
    CommonGloveOutputWorker(const CommonGloveOutputWorker&) = delete;
    CommonGloveOutputWorker& operator=(const CommonGloveOutputWorker&) = delete;

    std::uint64_t submit_position(const GloveVec<G>& motor_rad, std::uint64_t source_ns,
                                  std::uint64_t deadline_ns);
    std::uint64_t submit_force(const JointArray<N>& target, const HandState<N>& hand,
                               const GloveFrameT<G>& glove, const GlovePoseT<G>& pose,
                               const std::optional<CommonJacobianSnapshot<N, G>>& jacobian,
                               std::uint64_t deadline_ns);
    std::optional<CommonGloveOutputResult<N, G>> poll();

    void enter_force();
    void enter_position(std::uint64_t deadline_ns);
    void close();
private:
    struct Impl;
    std::unique_ptr<GloveDeviceT<G>> compatibility_driver_;
    std::unique_ptr<Impl> impl_;
};
} // namespace motorhand::native
