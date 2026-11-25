#pragma once

#include "types.hpp"

#include <array>
#include <filesystem>
#include <memory>

namespace motorhand::native {

// CAD joint and motor coordinates are radians; points are canonical metres.
// Evaluations are pure; pass the previous valid pose to follow its branch.
// 并列支持 12-DOF (G=12, Tree=68) 与 16-DOF (G=16, Tree=80)。
template <std::size_t G = 12>
class GloveModelT {
public:
    static constexpr std::size_t glove_dof = G;
    static constexpr std::size_t tree_dof = tree_dof_for(G);
    using VecG = GloveVec<G>;
    using VecTree = TreeVec<G>;
    using Pose = GlovePoseT<G>;

    explicit GloveModelT(const std::filesystem::path& asset_root, bool left);
    ~GloveModelT();
    GloveModelT(GloveModelT&&) noexcept;
    GloveModelT& operator=(GloveModelT&&) noexcept;
    GloveModelT(const GloveModelT&) = delete;
    GloveModelT& operator=(const GloveModelT&) = delete;

    Pose zero();
    Pose evaluate_motors(const VecG& motors, const Pose* seed = nullptr);
    Pose evaluate_joints(const VecG& joints, const Pose* seed = nullptr);
    Pose evaluate_tree(const VecTree& tree);
    std::array<std::array<double, 2>, G> active_joint_bounds(const Pose& pose);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

using Ve33Model = GloveModelT<12>;
using Hand12Model = GloveModelT<12>;
using Hand16Model = GloveModelT<16>;

}  // namespace motorhand::native
