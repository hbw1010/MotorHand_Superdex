"""12-DOF (VE40) 与 16-DOF 并列手部型号规格与工厂调度。"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

from .mapping import (
    MODEL_MOTOR_JOINTS,
    MODEL_MOTOR_JOINTS_16,
    MOTOR_FINGER,
    MOTOR_FINGER_16,
    GloveMapping,
)

ROOT = Path(__file__).resolve().parents[2]
G4_12DOF_DIR = ROOT / "hardware/encos_g4_gateway_12dof"
G4_16DOF_DIR = ROOT / "hardware/encos_g4_gateway_16dof"
SHARPA_DIR = ROOT / "sharpa_teleop"


@dataclass(frozen=True)
class HandModelSpec:
    model_id: str
    label: str
    dof: int
    tree_dof: int
    model_dir: Path
    bot_path: Path
    driver_dir: Path
    shim_library: Path
    gateway_hz: int
    mapping_path: Path
    feedback_path: Path
    default_zero_path: Path
    motor_joints: tuple[str, ...]
    motor_fingers: tuple[str, ...]
    demo_amplitude: tuple[float, ...]
    sharpa_retarget_yaml: Path
    sharpa_feedback_json: Path
    sharpa_impedance_json: Path

    @property
    def force_frame_size(self) -> int:
        return 3 + 5 * self.dof + 1

    @property
    def display_packet_size(self) -> int:
        return 6 + 2 * self.dof


HAND_12DOF = HandModelSpec(
    model_id="12dof",
    label="12-DOF (VE40)",
    dof=12,
    tree_dof=68,
    model_dir=ROOT / "assets/hand_motor_ve40/hand_motor_ve40_R",
    bot_path=ROOT / "assets/hand_motor_ve40/hand_motor_ve40_R/hand_motor_ve40_right_closed_chain.superdex_bot",
    driver_dir=G4_12DOF_DIR,
    shim_library=G4_12DOF_DIR / "build/libg4_shim_12dof.so",
    gateway_hz=200,
    mapping_path=G4_12DOF_DIR / "config/right_sim_mapping_12dof.json",
    feedback_path=G4_12DOF_DIR / "config/right_sim_feedback_12dof.json",
    default_zero_path=Path.home() / ".config/superdex/ve33-g4-right-zero.json",
    motor_joints=MODEL_MOTOR_JOINTS,
    motor_fingers=MOTOR_FINGER,
    demo_amplitude=(0.20, 0.15, 0.60, 0.60, 0.80, 1.20, 0.80, 1.20, 0.80, 1.20, 0.80, 1.20),
    sharpa_retarget_yaml=SHARPA_DIR / "config/retargeting/sharpa/right.yaml",
    sharpa_feedback_json=SHARPA_DIR / "config/feedback/sharpa-right.json",
    sharpa_impedance_json=SHARPA_DIR / "config/feedback/wuji-driver-impedance-right.json",
)

HAND_16DOF = HandModelSpec(
    model_id="16dof",
    label="16-DOF",
    dof=16,
    tree_dof=80,
    model_dir=ROOT / "assets/hand_motor_16dof_R",
    bot_path=ROOT / "assets/hand_motor_16dof_R/hand_motor_16dof_right_closed_chain.superdex_bot",
    driver_dir=G4_16DOF_DIR,
    shim_library=G4_16DOF_DIR / "build/libg4_shim_16dof.so",
    gateway_hz=150,
    mapping_path=G4_16DOF_DIR / "config/right_sim_mapping_16dof.json",
    feedback_path=G4_16DOF_DIR / "config/right_sim_feedback_16dof.json",
    default_zero_path=Path.home() / ".config/superdex/motorhand-16dof-right-zero.json",
    motor_joints=MODEL_MOTOR_JOINTS_16,
    motor_fingers=MOTOR_FINGER_16,
    demo_amplitude=(
        0.20, 0.15, 0.60, 0.60,
        0.80, 1.20, 0.80, 1.20, 0.80, 1.20, 0.80, 1.20,
        0.28, 0.12, -0.12, -0.28,
    ),
    sharpa_retarget_yaml=SHARPA_DIR / "config/retargeting/sharpa/right_16dof.yaml",
    sharpa_feedback_json=SHARPA_DIR / "config/feedback/sharpa-right-16dof.json",
    sharpa_impedance_json=SHARPA_DIR / "config/feedback/wuji-driver-impedance-right-16dof.json",
)

HAND_PROFILES: dict[str, HandModelSpec] = {
    HAND_12DOF.model_id: HAND_12DOF,
    HAND_16DOF.model_id: HAND_16DOF,
}

AVAILABLE_HAND_MODELS: tuple[dict[str, Any], ...] = tuple(
    {"id": spec.model_id, "label": spec.label, "dof": spec.dof}
    for spec in (HAND_12DOF, HAND_16DOF)
)


def normalize_hand_model(model_id: str | None) -> str:
    key = (model_id or "12dof").strip().lower()
    if key not in HAND_PROFILES:
        raise ValueError(f"未知手部型号：{model_id!r}（仅支持 12dof / 16dof）")
    return key


def get_hand_profile(model_id: str | None = "12dof") -> HandModelSpec:
    return HAND_PROFILES[normalize_hand_model(model_id)]


class HandRuntimeBase:
    """12-DOF 与 16-DOF 并列运行时的统一抽象基类。"""

    spec: HandModelSpec

    def load_mapping(self, path: str | Path | None = None) -> GloveMapping:
        return GloveMapping.load(path or self.spec.mapping_path, motor_joints=self.spec.motor_joints)

    def load_feedback_config(self) -> dict[str, Any]:
        return json.loads(self.spec.feedback_path.read_text(encoding="utf-8"))

    def validate_mapping(self, mapping: GloveMapping) -> None:
        if mapping.raw.get("hand", "right") != "right" or not mapping.raw.get("calibrated"):
            raise ValueError("需要已标定的右手轴序、方向和比例")
        if len(mapping.motors) != self.spec.dof:
            raise ValueError(f"期望 {self.spec.dof} 轴映射，实际为 {len(mapping.motors)} 轴")
        semantics = {
            x["motor"]: x["name"].removesuffix("_joint")
            for x in json.loads((self.spec.model_dir / "config/joint_space.json").read_text())["independent"]
        }
        import numpy as np

        for m in mapping.motors:
            if (
                semantics.get(m.model_joint) != m.joint
                or m.sign not in (-1.0, 1.0)
                or m.scale <= 0
                or not np.isfinite([m.scale, m.zero_deg, m.min_deg, m.max_deg]).all()
                or m.min_deg >= m.max_deg
            ):
                raise ValueError(f"轴 {m.motor_id} 的标定与 {self.spec.label} 模型不一致")

    def disabled_mask(self, config: dict[str, Any] | None = None) -> int:
        cfg = config if config is not None else self.load_feedback_config()
        mask = 0
        for mid in cfg.get("disabled_motor_ids", ()):
            if isinstance(mid, int) and 1 <= mid <= self.spec.dof:
                mask |= 1 << (mid - 1)
        return mask

    def demo_axes_limits(self, axes: dict[int, tuple[str, Any]]) -> list[list[float]]:
        return [list(axes[mid][1].model_rad_limits()) for mid in axes]


def get_hand_runtime(model_id: str | None = "12dof"):
    key = normalize_hand_model(model_id)
    if key == "12dof":
        from .runtime_12dof import HandRuntime12

        return HandRuntime12()
    from .runtime_16dof import HandRuntime16

    return HandRuntime16()
