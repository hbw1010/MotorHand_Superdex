// 固定腕部场景的物体约束由操作场景的物理线程独占。
// 此对象必须先于其引用的 actor 和 scene 销毁。
#pragma once

#include <mochi_physics/cpp_api/mochi_scene.h>
#include <mochi_physics/cpp_api/mochi_constraint.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <vector>

namespace ve33 {
class GraspMotion {
    using V = Eigen::Vector3d;
    using Q = Eigen::Quaterniond;
    using A = mochi::Actor;
    using Pin = mochi::Constraint;
    using Triple = std::array<double, 3>;
    static V vec(mochi::Real3 x) { return {x[0], x[1], x[2]}; }
    static V vec(const Triple& x) { return {x[0], x[1], x[2]}; }
    static mochi::Real3 native(const V& v) {
        return {mochi::real(v.x()), mochi::real(v.y()), mochi::real(v.z())};
    }
    static Q quat(const mochi::Quaternion& q) {
        const auto v = q.ToReal4();
        return {v[3], v[0], v[1], v[2]};
    }
    static mochi::Quaternion native(const Q& rot) {
        return mochi::Quaternion(mochi::real(rot.x()),mochi::real(rot.y()),
                                 mochi::real(rot.z()),mochi::real(rot.w()));
    }
    static void check(const mochi::Error& error) {
        if (!error.IsOK()) throw std::runtime_error(error.GetDescription());
    }
    struct Position { Pin* pin; V offset; };
    struct Rotation { Pin* pin; Q target; };
    mochi::Scene& scene_;
    std::vector<A*> rigid_, fixed_, surfaces_;
    A* soft_;
    int center_node_;
    V center_, axis_;
    double angle_, target_angle_;
    bool locked_{}, free_{};
    std::optional<V> drag_;
    std::vector<Position> positions_, rim_;
    std::vector<Rotation> rotations_;
    V soft_offset_{V::Zero()};

    template<class T> void clear(std::vector<T>& pins) {
        for (auto& p : pins) scene_.DestroyConstraint(p.pin);
        pins.clear();
    }
    void release() { clear(rim_); clear(rotations_); clear(positions_); }

    mochi::Span<const mochi::real> soft_nodes() const {
        mochi::Error error;
        auto nodes = soft_->GetNodePositionsLocal(error);
        if (!error.IsOK() || !nodes.size()) nodes = soft_->GetMesh().coordinates;
        if (center_node_ < 0 || 3 * static_cast<size_t>(center_node_) + 2 >= nodes.size())
            throw std::runtime_error("软体中心节点无效");
        return nodes;
    }
    static V node(mochi::Span<const mochi::real> nodes, size_t i) {
        return {nodes[3*i],nodes[3*i+1],nodes[3*i+2]};
    }
    Pin* soft_pin(int index, const V& point, double damping) {
        mochi::DeformableNodePositionConstraintParams p;
        p.actor=soft_->GetHandle(); p.nodeIndex=index; p.position=native(point);
        p.stiffness=4000.; p.damping=mochi::real(damping);
        mochi::Error error;
        auto* pin=scene_.CreateDeformableNodePositionConstraint(p,error);
        check(error);
        return pin;
    }
    void find_center() {
        V lo=V::Constant(std::numeric_limits<double>::infinity()), hi=-lo;
        mochi::Error error;
        for (auto* s:surfaces_) {
            const auto box=s->GetAabbWorld(error);
            check(error);
            lo=lo.cwiseMin(vec(box.GetMin())); hi=hi.cwiseMax(vec(box.GetMax()));
        }
        if (lo.allFinite() && hi.allFinite()) center_=(lo+hi)*.5;
    }
    void pin_object() {
        mochi::Error error;
        for (auto* actor : rigid_) {
            const auto pose=actor->GetRootTransform();
            mochi::RigidPivotPositionConstraintParams p;
            p.actor=actor->GetHandle(); p.targetPosition=pose.GetTranslation();
            p.stiffness=4000.; p.damping=12.;
            auto* pin=scene_.CreateRigidPivotPositionConstraint(p,error); check(error);
            positions_.push_back({pin,vec(p.targetPosition)-center_});
            mochi::RigidPivotRotationConstraintParams r;
            r.actor=p.actor; r.targetRotation=pose.GetRotation().ToRotationVector();
            r.stiffness=1.; r.damping=.03;
            auto* twist=scene_.CreateRigidPivotRotationConstraint(r,error); check(error);
            rotations_.push_back({twist,quat(pose.GetRotation())});
        }
        if (soft_) {
            auto p=soft_->GetRootTransform();
            const V point=vec(p.GetTranslation()) + quat(p.GetRotation())*node(soft_nodes(),center_node_);
            soft_offset_=point-center_;
            positions_.push_back({soft_pin(center_node_,point,1.),soft_offset_});
        }
    }
    void pin_rim() {
        const auto nodes=soft_nodes();
        const Q orientation=quat(soft_->GetRootTransform().GetRotation());
        const V c=node(nodes,center_node_);
        std::vector<V> offsets, radial;
        for (size_t i=0;i<nodes.size()/3;++i) {
            V v=orientation*(node(nodes,i)-c);
            offsets.push_back(v); radial.push_back(v-axis_*axis_.dot(v));
        }
        const auto biggest=std::max_element(radial.begin(),radial.end(),[](const V& a,const V& b){
            return a.squaredNorm()<b.squaredNorm();
        });
        const auto across=std::max_element(radial.begin(),radial.end(),[&](const V& a,const V& b){
            return biggest->cross(a).squaredNorm()<biggest->cross(b).squaredNorm();
        });
        for (const auto it : {biggest,across}) {
            const auto i=it-radial.begin();
            rim_.push_back({soft_pin(static_cast<int>(i),center_+soft_offset_+offsets[i],8.),offsets[i]});
        }
    }
public:
    GraspMotion(mochi::Scene& scene, std::vector<A*> rigid, std::vector<A*> fixed,
                std::vector<A*> surfaces, std::vector<A*> soft, int node_id,
                const Triple& center, const Triple& axis, double angle)
        :scene_(scene),rigid_(std::move(rigid)),fixed_(std::move(fixed)),surfaces_(std::move(surfaces)),
         soft_(soft.empty()?nullptr:soft.front()),center_node_(node_id),center_(vec(center)),
         axis_(vec(axis).normalized()),angle_(angle),target_angle_(angle) {
        if (soft.size()>1 || !axis_.allFinite() || !center_.allFinite() || !std::isfinite(angle))
            throw std::invalid_argument("无效抓握物体或旋转轴");
        positions_.reserve(rigid_.size()+1); rotations_.reserve(rigid_.size()); rim_.reserve(2);
    }
    ~GraspMotion(){release();}
    void command(bool locked,double target,const std::vector<double>& drag) {
        if (!std::isfinite(target) || (!drag.empty() && drag.size()!=3))
            throw std::invalid_argument("无效物体目标");
        locked_=locked; target_angle_=target;
        if (drag.empty()) drag_.reset();
        else {
            V v{drag[0],drag[1],drag[2]};
            if (!v.allFinite()) throw std::invalid_argument("无效拖动坐标");
            drag_=v;
        }
    }
    Triple center() const {return {center_.x(),center_.y(),center_.z()};}
    void step(double dt) {
        const double error=target_angle_-angle_;
        const double delta=std::abs(error)<=.0001?error:
            std::clamp(error*-std::expm1(-dt/.05),-150.*dt,150.*dt);
        if (!delta && !locked_ && !drag_) {release();free_=true;return;}
        if (free_) {find_center();free_=false;}
        if (positions_.empty()) pin_object();
        if (!delta) clear(rim_);
        if (!drag_ && !delta) return;

        const V previous=center_;
        if (drag_) {
            const V move=*drag_-center_;
            center_+=move*std::min(1.,.35*dt/std::max(move.norm(),1e-9));
        }
        if (delta && soft_ && rim_.empty()) pin_rim();
        Q turn(Eigen::AngleAxisd(delta*std::numbers::pi/180.,axis_));
        angle_+=delta;
        mochi::Error err;
        if (delta) {
            for (auto& r:rotations_) {
                r.target=(turn*r.target).normalized();
                r.pin->SetTargetRotation(native(r.target),err);check(err);
            }
        }
        for (auto& p:positions_) {
            if (delta) p.offset=turn*p.offset;
            p.pin->SetTargetPosition(native(center_+p.offset),err);check(err);
        }
        for (auto* actor:fixed_) {
            const auto pose=actor->GetRootTransform();
            const auto target=mochi::TransformRT(native((turn*quat(pose.GetRotation())).normalized()),
                native(center_+turn*(vec(pose.GetTranslation())-previous)));
            actor->SetRootTransform(target,err);check(err);
        }
        if (soft_) {
            soft_offset_=positions_.back().offset;
            for (auto& p:rim_) {
                p.offset=turn*p.offset;
                p.pin->SetTargetPosition(native(center_+soft_offset_+p.offset),err);check(err);
            }
        }
    }
};
} // namespace ve33
