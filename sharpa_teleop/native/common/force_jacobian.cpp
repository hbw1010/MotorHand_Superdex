#include "common/force_jacobian.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace motorhand::native {
namespace {
template <std::size_t G>
void validate_request(const GloveVec<G>& q, const std::vector<int>& columns, double step_rad) {
    for (double value : q)
        if (!std::isfinite(value)) throw std::invalid_argument("non-finite glove joint");
    if (!std::isfinite(step_rad) || step_rad <= 0)
        throw std::invalid_argument("Jacobian step must be positive and finite");
    std::array<bool, G> seen{};
    for (int column : columns) {
        if (column < 0 || column >= static_cast<int>(G) || seen[column])
            throw std::invalid_argument("Jacobian columns must be distinct indices in [0, G)");
        seen[column] = true;
    }
}
}  // namespace

template <std::size_t N, std::size_t G>
CommonForceJacobianColumns<N, G>::CommonForceJacobianColumns(const std::filesystem::path& asset_root,
                                                             HandModelFactory<N> hand_factory,
                                                             bool left)
    : model_(asset_root, left), retarget_(hand_factory()) {}

template <std::size_t N, std::size_t G>
std::vector<double> CommonForceJacobianColumns<N, G>::compute(const GloveVec<G>& q,
                                                              const std::vector<int>& columns,
                                                              double step_rad) {
    validate_request<G>(q, columns, step_rad);
    if (columns.empty()) return {};
    const GlovePoseT<G> centre = model_.evaluate_joints(q, seed_ ? &*seed_ : nullptr);
    return compute_centre(q, centre, columns, step_rad);
}

template <std::size_t N, std::size_t G>
std::vector<double> CommonForceJacobianColumns<N, G>::compute(const GlovePoseT<G>& base_pose,
                                                              const std::vector<int>& columns,
                                                              double step_rad) {
    validate_request<G>(base_pose.joint_rad, columns, step_rad);
    if (columns.empty()) return {};
    const GlovePoseT<G> centre = model_.evaluate_tree(base_pose.tree_rad);
    for (std::size_t i = 0; i < G; ++i) {
        if (std::abs(centre.joint_rad[i] - base_pose.joint_rad[i]) > 1e-8)
            throw std::invalid_argument("glove pose joints do not match CAD tree");
    }
    return compute_centre(base_pose.joint_rad, centre, columns, step_rad);
}

template <std::size_t N, std::size_t G>
std::vector<double> CommonForceJacobianColumns<N, G>::compute_centre(const GloveVec<G>& q,
                                                                     const GlovePoseT<G>& centre,
                                                                     const std::vector<int>& columns,
                                                                     double step_rad) {
    retarget_->reset();
    retarget_->set_initial_hint(nullptr);
    const JointArray<N> hint = retarget_->solve(centre.points_m);
    retarget_->set_initial_hint(&hint);

    std::vector<double> result(N * columns.size());
    for (std::size_t k = 0; k < columns.size(); ++k) {
        GloveVec<G> plus = q, minus = q;
        plus[columns[k]] += step_rad;
        minus[columns[k]] -= step_rad;
        const auto evaluate_probe = [&](const GloveVec<G>& joints, double signed_step_rad) {
            try {
                return model_.evaluate_joints(joints, &centre);
            } catch (const std::runtime_error& error) {
                if constexpr (N == 22) {
                    const int axis = columns[k];
                    throw std::runtime_error("Sharpa力反馈雅可比扰动点闭链失败，手套模型关节 " +
                        std::to_string(axis + 1) + "，中心角 " + std::to_string(q[axis]) +
                        " rad，扰动 " + std::to_string(signed_step_rad) + " rad：" + error.what());
                } else {
                    throw;
                }
            }
        };
        const auto plus_pose = evaluate_probe(plus, step_rad);
        const auto minus_pose = evaluate_probe(minus, -step_rad);
        retarget_->reset();
        const JointArray<N> plus_hand = retarget_->solve(plus_pose.points_m);
        retarget_->reset();
        const JointArray<N> minus_hand = retarget_->solve(minus_pose.points_m);
        for (std::size_t row = 0; row < N; ++row) {
            const double value = (plus_hand[row] - minus_hand[row]) / (2.0 * step_rad);
            if (!std::isfinite(value)) throw std::runtime_error("non-finite retarget Jacobian");
            result[row * columns.size() + k] = value;
        }
    }
    seed_ = centre;
    return result;
}

template class CommonForceJacobianColumns<20, 12>;
template class CommonForceJacobianColumns<22, 12>;
template class CommonForceJacobianColumns<22, 16>;
}  // namespace motorhand::native
