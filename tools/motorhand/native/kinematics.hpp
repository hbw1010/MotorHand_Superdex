#pragma once

#include <array>
#include <memory>
#include <vector>

namespace nanobind { class dict; class module_; }

namespace ve33 {
// 模型只在启动时由 CAD 数据构建。控制、显示与摆放槽分别保存上次有效分支。
class Kinematics {
public:
    using Motors = std::array<double, 12>;
    explicit Kinematics(const nanobind::dict& model);
    ~Kinematics();
    Kinematics(const Kinematics&) = delete;
    Kinematics& operator=(const Kinematics&) = delete;

    std::vector<double> solve(const Motors& angles, int slot = 0);
    void copy_slot(int source, int destination);
    std::vector<double> derivative();   // CAD 关节行，电机 J1–J12 列，按行排列
    Motors project(const std::vector<double>& tree_torque, const Motors& sim_motor_positions);
    double last_gap_m() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
void bind_kinematics(nanobind::module_& module);
