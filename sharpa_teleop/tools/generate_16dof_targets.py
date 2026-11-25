#!/usr/bin/env python3
"""从 assets/hand_motor_16dof_R 生成 sharpa_teleop 独立的 16-DOF 运动学资产、joint_space.json 与 retarget_targets.json。"""
from __future__ import annotations

import json
from pathlib import Path
import shutil
import sys

import pinocchio as pin

ROOT = Path(__file__).resolve().parents[2]
SHARPA_ROOT = ROOT / "sharpa_teleop"
SRC_16 = ROOT / "assets/hand_motor_16dof_R"
DST_16 = SHARPA_ROOT / "assets/hand_motor_16dof/hand_motor_16dof_R"
REF_TEMPLATE = SHARPA_ROOT / "assets/hand_motor_ve33/hand_motor_ve33_R/config/retarget_targets.json"

sys.path.insert(0, str(SHARPA_ROOT / "tools"))
from generate_ve40_targets import build_retarget_targets


def build_joint_space_16dof() -> dict:
    return {
        "schema_version": 1,
        "side": "R",
        "dof": 16,
        "angle_unit": "rad",
        "zero": "相对于 2026-11-25 16DOF CAD 零位的相邻指节转角",
        "positive_direction": "使用 axis_joint 的局部 +Z 右手定则物理轴，前 12 轴与 vE.4.0 一致，13–16 为四指 MCP 侧摆",
        "independent": [
            {"name": "thumb_cmc_flex_joint", "parent": "base_link", "child": "thumb_cmc_flex_link", "axis_joint": "thumb_cmc_flex_motor_joint", "motor": "thumb_cmc_flex_motor_joint", "label": "拇指 CMC 屈伸"},
            {"name": "thumb_cmc_abd_joint", "parent": "thumb_cmc_flex_link", "child": "thumb_cmc_abd_link", "axis_joint": "thumb_cmc_abd_motor_joint", "motor": "thumb_cmc_abd_motor_joint", "label": "拇指 CMC 侧摆"},
            {"name": "thumb_mcp_joint", "parent": "thumb_cmc_abd_link", "child": "thumb_mcp_link", "axis_joint": "thumb_mcp_output_joint", "motor": "thumb_mcp_motor_joint", "label": "拇指 MCP 屈伸"},
            {"name": "thumb_ip_joint", "parent": "thumb_mcp_link", "child": "thumb_ip_link", "axis_joint": "thumb_ip_output_joint", "motor": "thumb_ip_motor_joint", "label": "拇指 IP 屈伸"},
            {"name": "index_mcp_flex_joint", "parent": "index_mount_link", "child": "index_mcp_link", "axis_joint": "index_mcp_output_joint", "motor": "index_mcp_motor_joint", "label": "食指 MCP 屈伸"},
            {"name": "index_pip_joint", "parent": "index_mcp_link", "child": "index_pip_link", "axis_joint": "index_pip_output_joint", "motor": "index_pip_dip_motor_joint", "label": "食指 PIP 屈伸"},
            {"name": "middle_mcp_flex_joint", "parent": "middle_mount_link", "child": "middle_mcp_link", "axis_joint": "middle_mcp_output_joint", "motor": "middle_mcp_motor_joint", "label": "中指 MCP 屈伸"},
            {"name": "middle_pip_joint", "parent": "middle_mcp_link", "child": "middle_pip_link", "axis_joint": "middle_pip_output_joint", "motor": "middle_pip_dip_motor_joint", "label": "中指 PIP 屈伸"},
            {"name": "ring_mcp_flex_joint", "parent": "ring_mount_link", "child": "ring_mcp_link", "axis_joint": "ring_mcp_output_joint", "motor": "ring_mcp_motor_joint", "label": "无名指 MCP 屈伸"},
            {"name": "ring_pip_joint", "parent": "ring_mcp_link", "child": "ring_pip_link", "axis_joint": "ring_pip_output_joint", "motor": "ring_pip_dip_motor_joint", "label": "无名指 PIP 屈伸"},
            {"name": "pinky_mcp_flex_joint", "parent": "pinky_mount_link", "child": "pinky_mcp_link", "axis_joint": "pinky_mcp_output_joint", "motor": "pinky_mcp_motor_joint", "label": "小指 MCP 屈伸"},
            {"name": "pinky_pip_joint", "parent": "pinky_mcp_link", "child": "pinky_pip_link", "axis_joint": "pinky_pip_output_joint", "motor": "pinky_pip_dip_motor_joint", "label": "小指 PIP 屈伸"},
            {"name": "index_mcp_abd_joint", "parent": "base_link", "child": "index_mount_link", "axis_joint": "index_mcp_abd_output_joint", "motor": "index_mcp_abd_motor_joint", "label": "食指 MCP 侧摆"},
            {"name": "middle_mcp_abd_joint", "parent": "base_link", "child": "middle_mount_link", "axis_joint": "middle_mcp_abd_output_joint", "motor": "middle_mcp_abd_motor_joint", "label": "中指 MCP 侧摆"},
            {"name": "ring_mcp_abd_joint", "parent": "base_link", "child": "ring_mount_link", "axis_joint": "ring_mcp_abd_output_joint", "motor": "ring_mcp_abd_motor_joint", "label": "无名指 MCP 侧摆"},
            {"name": "pinky_mcp_abd_joint", "parent": "base_link", "child": "pinky_mount_link", "axis_joint": "pinky_mcp_abd_output_joint", "motor": "pinky_mcp_abd_motor_joint", "label": "小指 MCP 侧摆"},
        ],
        "dependent": [
            {"name": "index_dip_joint", "parent": "index_pip_link", "child": "index_dip_link", "axis_joint": "index_dip_output_joint", "label": "食指 DIP 联动"},
            {"name": "middle_dip_joint", "parent": "middle_pip_link", "child": "middle_dip_link", "axis_joint": "middle_dip_output_joint", "label": "中指 DIP 联动"},
            {"name": "ring_dip_joint", "parent": "ring_pip_link", "child": "ring_dip_link", "axis_joint": "ring_dip_output_joint", "label": "无名指 DIP 联动"},
            {"name": "pinky_dip_joint", "parent": "pinky_pip_link", "child": "pinky_dip_link", "axis_joint": "pinky_dip_output_joint", "label": "小指 DIP 联动"},
        ],
    }


def main() -> None:
    (DST_16 / "config").mkdir(parents=True, exist_ok=True)
    shutil.copy2(SRC_16 / "hand_tree_only.urdf", DST_16 / "hand_tree_only.urdf")
    shutil.copy2(SRC_16 / "config/loop_closures.yaml", DST_16 / "config/loop_closures.yaml")

    js = build_joint_space_16dof()
    # 校验所有 joint_space 提到的 link/joint 都存在于 URDF 中
    m = pin.buildModelFromUrdf(str(SRC_16 / "hand_tree_only.urdf"))
    for entry in js["independent"]:
        for key in ("parent", "child"):
            assert m.existFrame(entry[key]), f"Missing link {entry[key]}"
        for key in ("axis_joint", "motor"):
            assert m.existJointName(entry[key]), f"Missing joint {entry[key]}"
    (DST_16 / "config/joint_space.json").write_text(
        json.dumps(js, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    ref_template = json.loads(REF_TEMPLATE.read_text(encoding="utf-8"))
    targets = build_retarget_targets(SRC_16, ref_template)
    targets["model_version"] = "hand_motor_16dof_R"
    (DST_16 / "config/retarget_targets.json").write_text(
        json.dumps(targets, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(f"已生成 16-DOF 运动学配置：{DST_16}")
    print("pad_report:", targets["pad_report"])


if __name__ == "__main__":
    main()
