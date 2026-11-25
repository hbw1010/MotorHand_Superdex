"""从现有 CAD 配置装入 C++ 闭链模型（支持 12-DOF 与 16-DOF 并列实例化）。"""
import numpy as np

from .kinematics import GloveKinematics, KinematicsError
from .native_runtime import load_native


class NativeGloveKinematics:
    _slot_ids = {"target": 0, "visual": 1, "placement": 2}

    def __init__(self, model_dir, mapping, *, native_cls=None):
        # 两条后端共用 CAD 解析及轴索引，避免维护第二套模型常量。
        reference = GloveKinematics(model_dir, mapping)
        fast = reference._fast
        self.joint_names = reference.joint_names
        self.independent = reference.independent
        self.dof = len(mapping.motors)
        model = {
            "parent": fast.parent.tolist(),
            "kind": (fast.revolute.astype(int) + 2 * fast.prismatic.astype(int)).tolist(),
            "rotation": fast.t_rot.reshape(-1, 9).tolist(),
            "position": fast.t_pos.tolist(), "axis": fast.axis.tolist(),
            "link_a": fast.w_la.tolist(), "point_a": fast.w_pa.tolist(),
            "link_b": fast.w_lb.tolist(), "point_b": fast.w_pb.tolist(),
            "tree": [fast.index[n] for n in self.joint_names],
            "motors": [fast.index[m.model_joint] for m in mapping.motors],
            "limits": [m.model_rad_limits() for m in mapping.motors],
            "tolerance": fast.tolerance,
            "groups": [],
        }
        for group, (witnesses, _, _, deps, lower, upper) in fast.blocks.items():
            motor_ids = [m.motor_id - 1 for m in mapping.motors if m.model_joint.split("_")[0] == group]
            model["groups"].append({"witnesses": witnesses.tolist(), "dependent": deps.tolist(),
                                    "motors": motor_ids, "lower": lower.tolist(), "upper": upper.tolist()})
        if native_cls is None:
            native = load_native()
            native_cls = native.Kinematics16 if self.dof == 16 else native.Kinematics
        self.native = native_cls(model)

    def full_pose(self, motor_rad, model, mapping, *, slot="target"):
        try:
            return np.asarray(self.native.solve(motor_rad, self._slot_ids[slot]))
        except (RuntimeError, ValueError, TypeError) as exc:
            raise KinematicsError(str(exc)) from exc

    def copy_slot(self, src, dst):
        self.native.copy_slot(self._slot_ids[src], self._slot_ids[dst])

    def tree_derivative(self):
        result = np.asarray(self.native.derivative()).reshape(len(self.joint_names), self.dof)
        result.setflags(write=False)
        return result

    def project_tree(self, tree_torque, motor_positions):
        try:
            return np.asarray(self.native.project(tree_torque, motor_positions))
        except (RuntimeError, ValueError, TypeError) as exc:
            raise KinematicsError(str(exc)) from exc

    @property
    def last_gap_m(self):
        return self.native.last_gap_m
