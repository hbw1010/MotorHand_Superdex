#pragma once
#include "common/hand.hpp"
#include "types.hpp"
#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>

namespace motorhand::native {

template <std::size_t N, std::size_t G = 12> struct CommonJacobianSnapshot {
    RetargetJacobian<N, G> jacobian{}; // N x G, row-major
    GloveVec<G> reference_joint_q12{};
    std::uint64_t source_ns{}; // Original glove receive time, never completion time.
    std::uint64_t request_id{};
};

// Four persistent threads each own a separate CAD model and retarget optimizer.
// Thread-safe submit/poll; latest pending request replaces earlier pending work.
template <std::size_t N, std::size_t G = 12> class CommonJacobianWorker {
public:
    CommonJacobianWorker(const std::filesystem::path& asset_root,
                         HandModelFactory<N> hand_factory, bool left, double step_rad);
    ~CommonJacobianWorker();
    CommonJacobianWorker(const CommonJacobianWorker&) = delete;
    CommonJacobianWorker& operator=(const CommonJacobianWorker&) = delete;

    std::uint64_t submit(const GlovePoseT<G>& pose, std::uint64_t source_ns);
    std::optional<CommonJacobianSnapshot<N, G>> poll();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace motorhand::native
