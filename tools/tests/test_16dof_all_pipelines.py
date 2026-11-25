"""16-DOF 手部全三条链路（固定腕部、VR 桌面、Sharpa 遥操）及 12/16 型号在线切换端到端测试。"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import unittest
from urllib import request as urlreq

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from motorhand.hand_profiles import get_hand_runtime


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def _get_json(port: int, path: str, timeout: float = 2.5) -> dict:
    with urlreq.urlopen(f"http://127.0.0.1:{port}{path}", timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def _post_json(port: int, path: str, payload: dict, timeout: float = 6.0) -> tuple[int, dict]:
    body = json.dumps(payload).encode("utf-8")
    req = urlreq.Request(
        f"http://127.0.0.1:{port}{path}",
        data=body,
        headers={"Content-Type": "application/json", "Origin": f"http://127.0.0.1:{port}"},
        method="POST",
    )
    with urlreq.urlopen(req, timeout=timeout) as r:
        raw = r.read()
        return r.status, (json.loads(raw.decode("utf-8")) if raw else {})


def _wait_for(fn, timeout: float = 25.0, interval: float = 0.25):
    deadline = time.monotonic() + timeout
    last_err = None
    while time.monotonic() < deadline:
        try:
            val = fn()
            if val:
                return val
        except Exception as exc:
            last_err = exc
        time.sleep(interval)
    raise AssertionError(f"等待条件超时 ({timeout}s), last_err={last_err}")


class Hand16AllPipelinesTest(unittest.TestCase):
    def test_parallel_12_and_16_kinematics_and_virtual_work(self):
        rt12 = get_hand_runtime("12dof")
        rt16 = get_hand_runtime("16dof")
        mp12 = rt12.load_mapping()
        mp16 = rt16.load_mapping()
        rt12.validate_mapping(mp12)
        rt16.validate_mapping(mp16)

        k12 = rt12.create_kinematics(mp12)
        k16 = rt16.create_kinematics(mp16)
        self.assertEqual(len(k12.joint_names), 68)
        self.assertEqual(len(k16.joint_names), 80)

        pose16 = np.zeros(16)
        pose16[4:6] = [0.35, 0.25]
        pose16[12:16] = [0.25, -0.15, 0.15, -0.25]
        tree16 = np.asarray(k16.native.solve(pose16.tolist(), 0), dtype=float)
        self.assertEqual(tree16.shape, (80,))
        gap = float(k16.native.last_gap_m)
        self.assertLess(gap, 1e-7)
        deriv = np.asarray(k16.native.derivative(), dtype=float).reshape(80, 16)
        self.assertTrue(np.all(np.isfinite(deriv)))
        # 四指侧摆列在树关节雅可比上应有明显灵敏度（虚功力矩投影通道畅通）
        for col in range(12, 16):
            self.assertGreater(float(np.linalg.norm(deriv[:, col])), 0.5)

    def test_superdex_16dof_closed_chain_cycles_and_j13_j16_direction(self):
        import yaml
        from scipy.spatial.transform import Rotation
        import superdex.physics as sdp
        import superdex.robotics as sdr

        rt = get_hand_runtime("16dof")
        mp = rt.load_mapping()
        kin = rt.create_kinematics(mp)
        tree_names = list(kin.joint_names)
        lc = yaml.safe_load((rt.spec.model_dir / "config/loop_closures.yaml").read_text(encoding="utf-8"))

        sdp.initialize(num_worker_threads=2)
        scene = sdp.create_scene("test_16dof_closed_chain")
        scene.set_gravity([0.0, 0.0, 0.0])
        ctx = sdr.create_context()
        prefab = sdr.load_bot_prefab_from_file(str(rt.spec.bot_path))
        prefab.joints[0].type = sdp.ArticulatedJointType.HARD
        for link in prefab.links:
            link.layer = "RightHand"
        scene.enable_layer_contact_symmetric("RightHand", "RightHand", False)
        bot = sdr.create_bot(scene, prefab, ctx)
        actor = bot.get_articulated_actor()
        info = actor.get_articulated_shape_info()
        offsets = {
            n: d.offset for n, d in zip(info.joint_names, info.dof_info)
            if n in tree_names and d.rot_size == 1
        }
        link_actors = {n: scene.get_actor(h) for n, h in zip(info.link_names, actor.get_nested_link_actors())}

        def link_T(name: str) -> np.ndarray:
            tr = link_actors[name].get_root_transform()
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat(tr.rotation).as_matrix()
            T[:3, 3] = np.array(tr.translation, dtype=float)
            return T

        anchors = []
        for cl in lc["closures"]:
            p0 = np.array(cl["point"], dtype=float)
            pa = (np.linalg.inv(link_T(cl["link_a"])) @ np.r_[p0, 1.0])[:3]
            pb = (np.linalg.inv(link_T(cl["link_b"])) @ np.r_[p0, 1.0])[:3]
            anchors.append((cl["link_a"], cl["link_b"], pa, pb))

        params = sdp.PoseControllerParams(len(info.link_names))
        motor_set = {m.model_joint for m in mp.motors}
        for i, n in enumerate(list(info.joint_names)[:len(info.link_names)]):
            if n in motor_set:
                params.joint_tracking[i] = sdp.PoseTrackingParams(stiffness=0.6, damping=0.005, saturation=0.2)
            elif info.dof_info[i].rot_size == 1:
                params.joint_tracking[i] = sdp.PoseTrackingParams(stiffness=0.09, damping=0.002, saturation=0.25)
        actor.add_articulated_pose_controller(params)

        motors = [0.0] * 12 + [0.25, 0.25, 0.25, 0.25]
        tree_target = np.asarray(kin.native.solve(motors, 0), dtype=float)
        pose = sdp.DynamicArrayReal(actor.get_num_dofs())
        actor.get_articulated_pose(pose)
        for jn, val in zip(tree_names, tree_target):
            pose[offsets[jn]] = float(val)
        actor.reset_articulated_target_pose(pose)
        for _ in range(120):
            scene.step(0.002)

        actual = sdp.DynamicArrayReal(actor.get_num_dofs())
        actor.get_articulated_pose(actual)
        for f in ("index", "middle", "ring", "pinky"):
            oi = offsets[f"{f}_mcp_abd_output_joint"]
            ti = tree_names.index(f"{f}_mcp_abd_output_joint")
            self.assertGreater(float(actual[oi]), 0.20, f"{f} 侧摆输出方向应与正输入一致")
            self.assertAlmostEqual(float(actual[oi]), float(tree_target[ti]), delta=0.02)

        max_gap_mm = max(
            float(np.linalg.norm((link_T(la) @ np.r_[pa, 1.0])[:3] - (link_T(lb) @ np.r_[pb, 1.0])[:3]) * 1000.0)
            for la, lb, pa, pb in anchors
        )
        self.assertLess(max_gap_mm, 0.25, f"32 处闭链最大物理脱节应 < 0.25 mm，实际 {max_gap_mm:.4f} mm")

    def test_live_web_three_pipelines_and_12_16_switching(self):
        port = _free_port()
        env = os.environ.copy()
        env["OPENBLAS_NUM_THREADS"] = "1"
        proc = subprocess.Popen(
            [
                sys.executable,
                str(ROOT / "tools/ve33_g4_view.py"),
                "--local-grasp",
                "--demo",
                "--quest-port",
                str(port),
                "--hand-model",
                "12dof",
            ],
            cwd=str(ROOT),
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            # 1) 初始 12-DOF 固定腕部抓握
            st12 = _wait_for(
                lambda: (
                    c if (c := _get_json(port, "/api/v1/control")).get("hand_model") == "12dof"
                    and len(c.get("rows", [])) == 12 else None
                ),
                timeout=30.0,
            )
            self.assertEqual(st12["dof"], 12)
            sc12 = _wait_for(
                lambda: s if len((s := _get_json(port, "/api/v1/scene")).get("right_hand", {})) == 73 else None,
                timeout=15.0,
            )
            self.assertEqual(sc12["hand_model"], "12dof")

            # 2) 在线切换到 16-DOF 固定腕部抓握，并验证 J13–J16 交互式全轴控制
            code, _ = _post_json(port, "/api/v1/hand-model", {"hand_model": "16dof"})
            self.assertIn(code, (200, 204))
            st16 = _wait_for(
                lambda: (
                    c if (c := _get_json(port, "/api/v1/control")).get("hand_model") == "16dof"
                    and len(c.get("rows", [])) == 16 else None
                ),
                timeout=30.0,
            )
            self.assertEqual(st16["dof"], 16)
            self.assertEqual(len(st16["demo_axes_limits"]), 16)
            sc16 = _wait_for(
                lambda: s if len((s := _get_json(port, "/api/v1/scene")).get("right_hand", {})) == 81 else None,
                timeout=15.0,
            )
            self.assertEqual(sc16["hand_model"], "16dof")

            spread = [0.0] * 12 + [0.30, 0.10, -0.10, -0.30]
            _post_json(port, "/api/v1/demo-axes", {"active": True, "values": spread})
            st_spread = _wait_for(
                lambda: (
                    c if len((c := _get_json(port, "/api/v1/control")).get("rows", [])) == 16
                    and all(abs(c["rows"][12 + i]["input_rad"] - spread[12 + i]) < 1e-3 for i in range(4))
                    else None
                ),
                timeout=10.0,
            )
            self.assertTrue(st_spread["demo_axes_active"])

            # 3) 切换到 VR 桌面多物体链路（保持 16-DOF 81 刚体与 16 轴控制）
            _post_json(port, "/api/v1/control", {"action": "switch_task", "task": "tabletop"})
            st_vr = _wait_for(
                lambda: (
                    c if (c := _get_json(port, "/api/v1/control")).get("task") == "tabletop"
                    and c.get("hand_model") == "16dof"
                    and len(c.get("rows", [])) == 16 else None
                ),
                timeout=30.0,
            )
            self.assertEqual(st_vr["dof"], 16)
            sc_vr = _wait_for(
                lambda: s if len((s := _get_json(port, "/api/v1/scene")).get("right_hand", {})) == 81
                and "test_box" in s.get("objects", {}) else None,
                timeout=15.0,
            )
            self.assertEqual(sc_vr["hand_model"], "16dof")

            # 4) 切换到 Sharpa 遥操作链路（继承 16-DOF，启动 sim 并验证 J13–J16 驱动 Sharpa MCP_AA）
            _post_json(port, "/api/v1/control", {"action": "switch_task", "task": "sharpa_real"})
            st_sh = _wait_for(
                lambda: (
                    c if (c := _get_json(port, "/api/v1/control")).get("task") == "sharpa_real"
                    and c.get("hand_model") == "16dof" else None
                ),
                timeout=25.0,
            )
            self.assertEqual(st_sh["dof"], 16)
            sh_scene = _wait_for(
                lambda: (
                    s if (s := _get_json(port, "/api/v1/sharpa-scene")).get("ready")
                    and s.get("glove_model") == "16dof" else None
                ),
                timeout=20.0,
            )
            glove_bodies = [k for k in sh_scene["bodies"] if k.startswith("glove_")]
            sharpa_bodies = [k for k in sh_scene["bodies"] if k.startswith("sharpa_")]
            self.assertEqual(len(glove_bodies), 81)
            self.assertEqual(len(sharpa_bodies), 23)

            # 启动 16-DOF Sharpa 离线仿真
            _post_json(port, "/api/v1/sharpa", {
                "action": "start",
                "mode": "sim",
                "hand": "right",
                "hardness": 0.3,
                "rate": 50,
                "with_object": False,
            })
            _wait_for(
                lambda: (
                    f if (f := _get_json(port, "/api/v1/sharpa").get("frame"))
                    and len(f.get("glove_motor_rad", [])) == 16
                    and len(f.get("position_rad", [])) == 22 else None
                ),
                timeout=15.0,
            )

            # 通过 /api/v1/demo-axes 发送正/负四指侧摆，验证 Sharpa 4 个 MCP_AA 关节真实跟随张合
            aa_idx = (6, 10, 14, 19)
            pos_axes = [0.0] * 12 + [0.25, 0.20, 0.20, 0.25]
            neg_axes = [0.0] * 12 + [-0.25, -0.20, -0.20, -0.25]
            _post_json(port, "/api/v1/demo-axes", {"active": True, "values": pos_axes})
            f_pos = _wait_for(
                lambda: (
                    f if (f := _get_json(port, "/api/v1/sharpa").get("frame"))
                    and f.get("demo_axes_active")
                    and all(abs(f["glove_motor_rad"][12 + i] - pos_axes[12 + i]) < 0.02 for i in range(4))
                    else None
                ),
                timeout=8.0,
            )
            time.sleep(0.45)
            f_pos = _get_json(port, "/api/v1/sharpa")["frame"]

            _post_json(port, "/api/v1/demo-axes", {"active": True, "values": neg_axes})
            _wait_for(
                lambda: (
                    f if (f := _get_json(port, "/api/v1/sharpa").get("frame"))
                    and all(abs(f["glove_motor_rad"][12 + i] - neg_axes[12 + i]) < 0.02 for i in range(4))
                    else None
                ),
                timeout=8.0,
            )
            time.sleep(0.45)
            f_neg = _get_json(port, "/api/v1/sharpa")["frame"]

            spans_deg = [
                abs(f_pos["hand_target_rad"][i] - f_neg["hand_target_rad"][i]) * 180.0 / math.pi
                for i in aa_idx
            ]
            self.assertGreater(sum(spans_deg) / 4.0, 12.0, f"Sharpa 四指 MCP_AA 侧摆跨度偏小: {spans_deg}")

            # 5) 在 Sharpa 页面内直接切回 12-DOF，再切回固定腕部，验证零回归
            _post_json(port, "/api/v1/hand-model", {"hand_model": "12dof"})
            _wait_for(
                lambda: c if (c := _get_json(port, "/api/v1/control")).get("hand_model") == "12dof" else None,
                timeout=25.0,
            )
            _post_json(port, "/api/v1/control", {"action": "switch_task", "task": "local_grasp"})
            st_back = _wait_for(
                lambda: (
                    c if (c := _get_json(port, "/api/v1/control")).get("task") == "local_grasp"
                    and c.get("hand_model") == "12dof"
                    and len(c.get("rows", [])) == 12 else None
                ),
                timeout=30.0,
            )
            self.assertEqual(st_back["dof"], 12)
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=3.0)


if __name__ == "__main__":
    unittest.main()
