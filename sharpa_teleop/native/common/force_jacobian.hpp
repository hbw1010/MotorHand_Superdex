#pragma once
#include "common/hand.hpp"
#include "ve33_model.hpp"

#include <filesystem>
#include <optional>
#include <vector>

namespace motorhand::native {

// One instance per worker thread; never shares the forward control optimizer.
template <std::size_t N, std::size_t G = 12> class CommonForceJacobianColumns {
public:
    CommonForceJacobianColumns(const std::filesystem::path& asset_root,
                               HandModelFactory<N> hand_factory, bool left);

    std::vector<double> compute(const GloveVec<G>& q, const std::vector<int>& columns,
                                double step_rad);
    std::vector<double> compute(const GlovePoseT<G>& base_pose,
                                const std::vector<int>& columns, double step_rad);

private:
    std::vector<double> compute_centre(const GloveVec<G>& q, const GlovePoseT<G>& centre,
                                       const std::vector<int>& columns, double step_rad);
    GloveModelT<G> model_;
    std::unique_ptr<HandModel<N>> retarget_;
    std::optional<GlovePoseT<G>> seed_;
};

}  // namespace motorhand::native
