#pragma once
#include "common/driver_impedance_config.hpp"
#include "common/feedback.hpp"
namespace motorhand::native {
template <std::size_t N, std::size_t G = 12> struct CommonDriverImpedancePlan {
    DriverImpedanceCommandT<G> command{};
    GloveVec<G> nominal_nm{}, proxy_surface_rad{};
    CommonDriverImpedanceObservation<N, G> observation{};
    bool valid{};
};
template <std::size_t N, std::size_t G = 12> class CommonDriverImpedancePlanner {
public:
    CommonDriverImpedancePlanner(DriverImpedanceConfigT<G>, double hardness,
                                 const std::array<EncosMotorCalibration, G>&);
    void reset(std::uint64_t now_ns);
    CommonDriverImpedancePlan<N, G> step(const JointArray<N>& final_target, const HandState<N>&,
                                         const GloveFrameT<G>&, const GlovePoseT<G>&,
                                         const RetargetJacobian<N, G>& final_jacobian,
                                         const FeedbackResult<N, G>&, std::uint64_t now_ns);
private:
    DriverImpedanceConfigT<G> config_;
    double hardness_;
    std::array<EncosMotorCalibration, G> calibration_;
    MotorVelocityEstimatorT<G> velocity_;
    std::array<int, G> direction_{};
    std::array<bool, G> damping_{};
    GloveVec<G> surface_{}, surface_velocity_{};
    std::uint64_t reset_ns_{}, source_ns_{};
};
}
