"""12-DOF (VE40) 专属并列运行时工厂：直接绑定 C++ 12 自由度实例化。"""
from __future__ import annotations

from .feedback import (
    RETREAT_BLEND_END_RAD,
    RETREAT_BLEND_START_RAD,
    build_contact_kernel,
    contact_feedback_gains,
    feedback_curves,
)
from .hand_profiles import HAND_12DOF, HandRuntimeBase
from .native_kinematics import NativeGloveKinematics
from .native_runtime import load_native


class HandRuntime12(HandRuntimeBase):
    spec = HAND_12DOF

    def create_kinematics(self, mapping):
        native = load_native()
        return NativeGloveKinematics(self.spec.model_dir, mapping, native_cls=native.Kinematics12)

    def create_shared_link(self, path, tree_size: int = 0):
        native = load_native()
        return native.SharedLink12(str(path), int(tree_size))

    def create_feedback_control(self, axes, profile: str = "identity", config=None):
        native = load_native()
        cfg = config if config is not None else self.load_feedback_config()
        imp = cfg["driver_impedance"]
        gains = contact_feedback_gains(axes, config=cfg)
        curves = feedback_curves(profile, config=cfg, dof=self.spec.dof)
        kernel = build_contact_kernel(
            native,
            gains,
            curves,
            cfg.get("friction_comp"),
            config=cfg,
            kernel_cls=native.ContactFeedback12,
            dof=self.spec.dof,
        )
        ratios = [axes[i + 1][1].sign * axes[i + 1][1].scale * cfg["torque_sign"][i] for i in range(self.spec.dof)]
        motor_limit = min(20.0, max(cfg["torque_limit_nm"])) * imp["torque_margin"]
        return native.FeedbackControl12(
            kernel,
            ratios,
            motor_limit,
            int(cfg["command_ttl_s"] * 1e9),
            round(1e9 / cfg["control_hz"]),
            cfg["feedback_ramp_s"],
            cfg["velocity_filter_hz"],
            RETREAT_BLEND_START_RAD,
            RETREAT_BLEND_END_RAD,
        )

    def create_native_loop(self, kin, control, link, port, axes, *, disabled_mask: int = 0,
                           preserve_zero_on_reconnect: bool = False, config=None):
        native = load_native()
        cfg = config if config is not None else self.load_feedback_config()
        motors = [motor for _, motor in axes.values()]
        lo, hi = zip(*(motor.model_rad_limits() for motor in motors))
        return native.NativeLoop12(
            kin.native,
            control,
            link,
            "" if not port else str(port),
            [m.sign * m.scale for m in motors],
            lo,
            hi,
            self.spec.demo_amplitude,
            round(1e9 / cfg["control_hz"]),
            disabled_mask,
            bool(preserve_zero_on_reconnect),
        )

    def create_grasp_physics(self, scene, actor, links, tree_indices, motor_indices, palm_index: int):
        native = load_native()
        return native.GraspPhysics12(scene, actor, links, tree_indices, motor_indices, palm_index)
