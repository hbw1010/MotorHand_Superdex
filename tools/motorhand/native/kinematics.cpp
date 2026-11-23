// VE33 闭链：前向树变换、点约束雅可比、分支连续求解与虚功投影。
#include "kinematics.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/vector.h>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>

namespace nb = nanobind;
namespace ve33 {
namespace {
using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;
using Vec = Eigen::VectorXd;
using Mat = Eigen::MatrixXd;
using RowMat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Indices = std::vector<int>;
using Triples = std::vector<std::array<double, 3>>;

struct Joint {
    int parent, kind;  // 0 固定，1 转动，2 平移
    M3 r;
    V3 p, axis;
};
struct Witness {
    int a, b;
    V3 pa, pb;
};
struct Block {
    Indices witnesses, dependent, motor_ids, motor_joints;
    std::vector<double> lower, upper;
};
struct Geometry {
    explicit Geometry(size_t n, size_t w)
        : r(n + 1, M3::Identity()), p(n + 1, V3::Zero()), axis(n), a(w), b(w) {}
    std::vector<M3> r;
    std::vector<V3> p, axis, a, b;
};

void require(bool valid, const char* error) {
    if (!valid) throw std::invalid_argument(error);
}
bool valid_indices(const Indices& ids, size_t n) {
    return std::all_of(ids.begin(), ids.end(), [n](int j) {return j >= 0 && size_t(j) < n;});
}
template<typename T> T field(const nb::dict& data, const char* key) {
    return nb::cast<T>(data[key]);
}
Mat least_squares(const Mat& jac, const Mat& rhs, double cutoff) {
    Eigen::JacobiSVD<Mat> svd(jac, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(cutoff);
    return svd.solve(rhs);
}
}

struct Kinematics::Impl {
    std::mutex lock;
    std::vector<Joint> joints;
    std::vector<Witness> witnesses;
    std::vector<Block> blocks;
    std::vector<std::vector<bool>> ancestor;
    Indices tree, motors, tree_row;
    std::array<std::array<double,2>,12> limits;
    double tolerance, last_gap = 0.;
    std::array<Vec,3> slots;
    RowMat jacobian;
    bool jac_valid = false;
    Motors projected_pose{};
    bool projection_valid = false;

    explicit Impl(const nb::dict& d) {
        auto parent = field<Indices>(d, "parent"), kind = field<Indices>(d, "kind");
        const size_t n = parent.size();
        auto rotations = field<std::vector<std::array<double,9>>>(d, "rotation");
        auto positions = field<Triples>(d, "position"), axes = field<Triples>(d, "axis");
        require(n && kind.size() == n && positions.size() == n && axes.size() == n && rotations.size() == n,
                "闭链模型关节维数错误");
        ancestor.assign(n + 1, std::vector<bool>(n));
        for (size_t i = 0; i < n; ++i) {
            require(parent[i] >= 0 && (size_t(parent[i]) < i || size_t(parent[i]) == n) &&
                    kind[i] >= 0 && kind[i] <= 2, "闭链模型拓扑错误");
            joints.push_back({parent[i], kind[i],
                Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(rotations[i].data()),
                Eigen::Map<V3>(positions[i].data()), Eigen::Map<V3>(axes[i].data())});
            const auto& j = joints.back();
            require(j.r.allFinite() && j.p.allFinite() && j.axis.allFinite(), "无效关节几何");
            ancestor[i] = ancestor[parent[i]];
            ancestor[i][i] = true;
        }
        auto la = field<Indices>(d,"link_a"), lb = field<Indices>(d,"link_b");
        auto pa = field<Triples>(d,"point_a"), pb = field<Triples>(d,"point_b");
        require(!la.empty() && la.size() == lb.size() && la.size() == pa.size() && la.size() == pb.size() &&
                valid_indices(la,n+1) && valid_indices(lb,n+1), "闭链约束维数错误");
        for (size_t i = 0; i < la.size(); ++i) {
            witnesses.push_back({la[i], lb[i], Eigen::Map<V3>(pa[i].data()), Eigen::Map<V3>(pb[i].data())});
            require(witnesses.back().pa.allFinite() && witnesses.back().pb.allFinite(), "无效闭链约束点");
        }
        tree = field<Indices>(d,"tree");
        motors = field<Indices>(d,"motors");
        require(!tree.empty() && motors.size()==12 && valid_indices(tree,n) && valid_indices(motors,n), "闭链轴序错误");
        tree_row.assign(n, -1);
        for (size_t row = 0; row < tree.size(); ++row) {
            require(tree_row[tree[row]] == -1, "重复的闭链输出关节");
            tree_row[tree[row]] = int(row);
        }
        limits = field<decltype(limits)>(d,"limits");
        for (size_t i = 0; i < motors.size(); ++i)
            require(tree_row[motors[i]] >= 0 && std::isfinite(limits[i][0]) && std::isfinite(limits[i][1]) &&
                    limits[i][0] <= limits[i][1], "电机轴或限位错误");
        tolerance = field<double>(d, "tolerance");
        require(tolerance > 0. && std::isfinite(tolerance), "无效闭链容差");
        for (nb::handle item : d["groups"]) {
            auto group = nb::cast<nb::dict>(item);
            Block b{field<Indices>(group,"witnesses"), field<Indices>(group,"dependent"),
                    field<Indices>(group,"motors"), {}, field<std::vector<double>>(group,"lower"),
                    field<std::vector<double>>(group,"upper")};
            require(!b.witnesses.empty() && !b.dependent.empty() && !b.motor_ids.empty() &&
                    valid_indices(b.witnesses,witnesses.size()) && valid_indices(b.dependent,n) &&
                    valid_indices(b.motor_ids,12) && b.lower.size() == b.dependent.size() &&
                    b.upper.size() == b.lower.size(), "无效的闭链约束分组");
            for (size_t k=0; k<b.dependent.size(); ++k)
                require(tree_row[b.dependent[k]] >= 0 && b.lower[k] <= b.upper[k], "从动关节限位错误");
            for (int id : b.motor_ids) b.motor_joints.push_back(motors[id]);
            blocks.push_back(std::move(b));
        }
        require(!blocks.empty(), "缺少闭链分组");
        for (auto& slot : slots) slot = Vec::Zero(n);
    }

    void forward(const Vec& q, Geometry& g) const {
        for (size_t i = 0; i < joints.size(); ++i) {
            const auto& j = joints[i];
            int p = j.parent;
            M3 base = g.r[p] * j.r;
            g.axis[i] = base * j.axis;
            g.p[i] = g.r[p] * j.p + g.p[p];
            if (j.kind == 2) g.p[i] += g.axis[i] * q[i];
            if (j.kind == 1) g.r[i] = base * Eigen::AngleAxisd(q[i], j.axis);
            else g.r[i] = base;
        }
        for (size_t i = 0; i < witnesses.size(); ++i) {
            const auto& w = witnesses[i];
            g.a[i] = g.r[w.a] * w.pa + g.p[w.a];
            g.b[i] = g.r[w.b] * w.pb + g.p[w.b];
        }
    }

    double gap(const Geometry& g) const {
        double squared = 0.;
        for (size_t i = 0; i < witnesses.size(); ++i) {
            double x = (g.a[i] - g.b[i]).squaredNorm();
            if (!std::isfinite(x)) return std::numeric_limits<double>::infinity();
            squared = std::max(squared, x);
        }
        return std::sqrt(squared);
    }

    Mat block_jac(const Block& block, const Indices& cols, const Geometry& g) const {
        Mat jac = Mat::Zero(3 * block.witnesses.size(), cols.size());
        for (size_t r = 0; r < block.witnesses.size(); ++r) {
            int i = block.witnesses[r];
            const auto& w = witnesses[i];
            for (size_t c = 0; c < cols.size(); ++c) {
                int j = cols[c];
                auto term = jac.block<3,1>(3*r,c);
                if (ancestor[w.a][j])
                    term += joints[j].kind == 2 ? g.axis[j] : g.axis[j].cross(g.a[i]-g.p[j]);
                if (ancestor[w.b][j])
                    term -= joints[j].kind == 2 ? g.axis[j] : g.axis[j].cross(g.b[i]-g.p[j]);
            }
        }
        return jac;
    }

    bool advance(Vec& state, const Motors& target, const std::vector<const Block*>& changed,
                 Geometry& g, int depth) const {
        Vec q = state;
        for (size_t i = 0; i < motors.size(); ++i) q[motors[i]] = target[i];
        auto active = changed;
        for (int iteration = 0; iteration < 25 && !active.empty(); ++iteration) {
            forward(q,g);
            std::vector<const Block*> remaining;
            for (const Block* block : active) {
                Vec rhs(3*block->witnesses.size());
                double worst = 0.;
                for (size_t r=0; r<block->witnesses.size(); ++r) {
                    int w = block->witnesses[r];
                    rhs.segment<3>(r*3) = g.b[w] - g.a[w];
                    worst = std::max(worst, rhs.segment<3>(r*3).norm());
                }
                if (worst <= .05*tolerance) continue;
                Vec step = least_squares(block_jac(*block,block->dependent,g),rhs,1e-10);
                if (!step.allFinite()) return false;
                for (size_t k=0; k<block->dependent.size(); ++k) {
                    int j=block->dependent[k];
                    q[j] = std::clamp(q[j] + step[k], block->lower[k], block->upper[k]);
                }
                remaining.push_back(block);
            }
            active = std::move(remaining);
        }
        forward(q,g);
        bool continuous = true;
        for (const Block* b : changed)
            for (int j : b->dependent) continuous &= std::abs(q[j]-state[j]) <= .2;
        if (continuous && gap(g) <= tolerance) {
            state = std::move(q);
            return true;
        }
        Motors middle;
        double delta = 0.;
        for (size_t i=0; i<motors.size(); ++i) {
            double start = state[motors[i]];
            middle[i] = (start + target[i]) / 2.;
            delta = std::max(delta,std::abs(start-target[i]));
        }
        return depth < 8 && delta >= 1e-5 &&
            advance(state,middle,changed,g,depth+1) && advance(state,target,changed,g,depth+1);
    }

    void solve(const Motors& target, int slot) {
        Vec& stored = slots.at(slot);
        Motors start;
        double delta = 0.;
        for (size_t i=0; i<motors.size(); ++i) {
            require(std::isfinite(target[i]), "闭链输入含非有限角度");
            start[i] = stored[motors[i]];
            delta = std::max(delta,std::abs(target[i]-start[i]));
        }
        require(delta <= 240.*.02, "闭链目标增量过大");
        std::vector<const Block*> changed;
        for (const auto& b : blocks) {
            if (std::any_of(b.motor_ids.begin(), b.motor_ids.end(),
                            [&](int i){return std::abs(target[i]-start[i])>1e-12;}))
                changed.push_back(&b);
        }
        Geometry g(joints.size(),witnesses.size());
        if (!changed.empty()) {
            Vec q = stored; // 完整 continuation 成功后才发布新分支。
            int count = std::max(1,int(std::ceil(delta/.02)));
            for (int k=1; k<=count; ++k) {
                Motors goal;
                for (size_t i=0; i<motors.size(); ++i)
                    goal[i] = start[i] + (target[i]-start[i])*double(k)/count;
                require(advance(q,goal,changed,g,0), "目标超出闭链装配分支");
            }
            stored = std::move(q);
            if (slot == 0) {jac_valid = false; projection_valid = false;}
        } else forward(stored,g);
        last_gap = gap(g);
    }

    const RowMat& derivative() {
        if (jac_valid) return jacobian;
        Geometry g(joints.size(),witnesses.size());
        forward(slots[0],g);
        RowMat result = RowMat::Zero(tree.size(),motors.size());
        for (const auto& b : blocks) {
            const Mat a = block_jac(b,b.dependent,g);
            const Mat dq = least_squares(a,-block_jac(b,b.motor_joints,g),1e-9);
            require(dq.allFinite(), "闭链导数无效");
            for (size_t k=0; k<b.motor_ids.size(); ++k) {
                const int col = b.motor_ids[k];
                result(tree_row[motors[col]],col) = 1.;
                for (size_t r=0; r<b.dependent.size(); ++r)
                    result(tree_row[b.dependent[r]],col) = dq(r,k);
            }
        }
        jacobian = std::move(result);
        jac_valid = true;
        return jacobian;
    }
};

Kinematics::Kinematics(const nb::dict& model) : impl_(std::make_unique<Impl>(model)) {}
Kinematics::~Kinematics() = default;

std::vector<double> Kinematics::solve(const Motors& q, int slot) {
    std::lock_guard guard(impl_->lock);
    impl_->solve(q,slot);
    std::vector<double> result;
    result.reserve(impl_->tree.size());
    for (int j : impl_->tree) result.push_back(impl_->slots.at(slot)[j]);
    return result;
}
void Kinematics::copy_slot(int src,int dst) {
    std::lock_guard guard(impl_->lock);
    impl_->slots.at(dst) = impl_->slots.at(src);
    if (dst == 0) {impl_->jac_valid = false; impl_->projection_valid = false;}
}
std::vector<double> Kinematics::derivative() {
    std::lock_guard guard(impl_->lock);
    const auto& jac = impl_->derivative();
    return {jac.data(),jac.data()+jac.size()};
}
double Kinematics::last_gap_m() const {
    std::lock_guard guard(impl_->lock);
    return impl_->last_gap;
}
Kinematics::Motors Kinematics::project(const std::vector<double>& torque, const Motors& q) {
    std::lock_guard guard(impl_->lock);
    require(torque.size()==impl_->tree.size() &&
            std::all_of(torque.begin(),torque.end(),[](double x){return std::isfinite(x);}),
            "闭链投影输入力矩无效");
    Motors clamped;
    double delta = 0.;
    for (size_t i=0; i<q.size(); ++i) {
        require(std::isfinite(q[i]),"闭链投影角度无效");
        clamped[i] = std::clamp(q[i],impl_->limits[i][0],impl_->limits[i][1]);
        delta = std::max(delta,std::abs(clamped[i]-impl_->projected_pose[i]));
    }
    if (std::all_of(torque.begin(),torque.end(),[](double x){return x==0.;})) return {};
    if (!impl_->projection_valid || delta > .001) {
        impl_->solve(clamped,0);
        impl_->derivative();
        impl_->projected_pose = clamped;
        impl_->projection_valid = true;
    }
    Motors result;
    Eigen::Map<Vec>(result.data(),12) =
        impl_->derivative().transpose() * Eigen::Map<const Vec>(torque.data(),torque.size());
    return result;
}
} // namespace ve33

void bind_kinematics(nb::module_& m) {
    using K = ve33::Kinematics;
    nb::class_<K>(m,"Kinematics")
        .def(nb::init<const nb::dict&>())
        .def("solve",&K::solve,nb::arg("angles"),nb::arg("slot")=0,nb::call_guard<nb::gil_scoped_release>())
        .def("copy_slot",&K::copy_slot)
        .def("derivative",&K::derivative,nb::call_guard<nb::gil_scoped_release>())
        .def("project",&K::project,nb::call_guard<nb::gil_scoped_release>())
        .def_prop_ro("last_gap_m",&K::last_gap_m);
}
