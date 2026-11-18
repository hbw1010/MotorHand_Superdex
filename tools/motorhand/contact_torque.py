"""将物理引擎实际接触力按虚功投影到 VE33 的独立电机轴。"""

import numpy as np


class ContactTorqueProjector:
    def __init__(self, links, kin, mapping, tree_indices, motor_indices):
        self.kin = kin
        self.mapping = mapping
        self.tree_indices = tree_indices
        self.motor_indices = motor_indices
        limits = np.array([motor.model_rad_limits() for motor in mapping.motors])
        self.low, self.high = limits.T
        self.links = [actor for name, actor in links if name != "base_link"]
        self.q = None

    def project(self, pose):
        tree_torque = np.zeros(len(self.tree_indices))
        loaded = False
        for link in self.links:
            # 引擎已累加各接触点的合力及质心力矩。场景的接触层只允许右手碰物体。
            wrench = np.array([*link.get_contact_force_world(), *link.get_contact_torque_world()])
            if not np.isfinite(wrench).all():
                raise ValueError("仿真接触力不可用")
            if not np.any(wrench):
                continue
            jacobian = np.asarray(link.get_articulated_jacobian()).reshape(6, -1)
            tree_torque += wrench @ jacobian[:, self.tree_indices]
            loaded = True
        if not loaded:
            return np.zeros(12)
        # 机械限位的数值余量不应把 CAD 闭链求解推到标定区间以外。
        q = np.clip(np.asarray(pose)[self.motor_indices], self.low, self.high)
        # 约 0.06° 以内复用闭链导数，避免静止接触的小抖动反复求解闭链。
        if self.q is None or np.max(np.abs(q - self.q)) > .001:
            self.kin.full_pose(q, None, self.mapping)
            self.q = q
        return tree_torque @ self.kin.tree_derivative()
