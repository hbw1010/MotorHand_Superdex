#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

namespace nanobind { class dict; class module_; }

namespace ve33 {
// 模型只在启动时由 CAD 数据构建。控制、显示与摆放槽分别保存上次有效分支。
template <size_t Dof>
class KinematicsT {
public:
    static constexpr size_t dof = Dof;
    using Motors = std::array<double, Dof>;
    explicit KinematicsT(const nanobind::dict& model);
    ~KinematicsT();
    KinematicsT(const KinematicsT&) = delete;
    KinematicsT& operator=(const KinematicsT&) = delete;

    std::vector<double> solve(const Motors& angles, int slot = 0);
    void copy_slot(int source, int destination);
    std::vector<double> derivative();   // CAD 关节行，电机 J1–J{Dof} 列，按行排列
    Motors project(const std::vector<double>& tree_torque, const Motors& sim_motor_positions);
    double last_gap_m() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

using Kinematics = KinematicsT<12>;
using Kinematics12 = KinematicsT<12>;
using Kinematics16 = KinematicsT<16>;
}
void bind_kinematics(nanobind::module_& module);
