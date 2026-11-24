"""验证左手与右手共用空间、丢帧后清除，以及力的方向和单位。"""
from pathlib import Path
import sys
import time
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ve33_quest import IDENTITY, XR_TO_WORLD, WristBridge


class XRDiagnosticsTest(unittest.TestCase):
    def test_left_hand_shares_right_wrist_alignment_and_clears_on_new_space(self):
        bridge = WristBridge()
        msg = {"space": "xr", "seq": 1, "pose": [1., 2., 3., 0., 0., 0., 1.], "align": True,
               "left": {"kind": "hand", "points": [[.9, 2.1, 2.8]] * 16}}
        bridge.receive(msg)
        self.assertIsNone(bridge.left_hand())
        bridge.wrist()
        self.assertEqual(bridge.left_hand()["kind"], "hand")
        np.testing.assert_allclose(bridge.left_hand()["points"], [[.2, .1, .1]] * 16)
        bridge.receive({"space": "reset", "seq": 1, "pose": None})
        self.assertIsNone(bridge.left_hand())

    def test_invalid_or_expired_left_probes_are_unavailable(self):
        bridge = WristBridge()
        pose = [0., 0., 0., 0., 0., 0., 1.]
        with self.assertRaises(ValueError):
            bridge.receive({"space": "xr", "seq": 1, "pose": pose,
                            "left": {"kind": "controller", "points": [[float("nan"), 0, 0]]}})
        bridge.receive({"space": "xr", "seq": 1, "pose": pose, "align": True,
                        "left": {"kind": "controller", "points": [[.10, 0, -.20]]}})
        bridge.wrist()
        np.testing.assert_allclose(bridge.left_hand()["points"], [[.2, -.1, 0]])
        self.assertIsNone(bridge.left_hand(time.monotonic() + .5))

    def test_scene_reset_rebases_live_wrist_and_discards_old_left_probes(self):
        bridge = WristBridge()
        identity = [0., 0., 0., 0., 0., 0., 1.]
        moved = [.06, .04, -.08, .0, .0, .0, 1.]
        left = {"kind": "controller", "points": [[.1, .1, -.1]]}
        bridge.receive({"space": "xr", "seq": 1, "pose": identity, "align": True})
        bridge.wrist()
        bridge.receive({"space": "xr", "seq": 2, "pose": moved, "left": left})
        self.assertGreater(np.linalg.norm(bridge.wrist()[0][:3]), .05)
        self.assertIsNotNone(bridge.left_hand())
        bridge.reset_scene()
        np.testing.assert_allclose(bridge.wrist()[0], identity)
        self.assertIsNone(bridge.left_hand())
        bridge.receive({"space": "xr", "seq": 3, "pose": moved, "left": left})
        np.testing.assert_allclose(bridge.wrist()[0], identity)
        self.assertIsNotNone(bridge.left_hand())

    def test_controller_lever_arm_offset_eliminates_rotation_arc_and_aligns_world(self):
        from scipy.spatial.transform import Rotation

        with self.assertRaises(ValueError):
            WristBridge(controller_offset=(0., 0.))
        with self.assertRaises(ValueError):
            WristBridge(controller_offset=(float("nan"), 0., 0.))
        with self.assertRaises(ValueError):
            WristBridge(controller_offset=(0., 0., 1.5))

        offset = np.array([0.015, -0.02, 0.09])
        bridge = WristBridge(controller_offset=offset)
        # 任意支架安装姿态角 R_ctrl_ref（绕 XYZ 倾斜安装），验证对齐时自动消去安装倾角
        r_ctrl_ref = Rotation.from_euler("xyz", [28., -41., 15.], degrees=True)
        hand_xr_ref = np.array([0.3, 1.1, -0.4])
        ctrl_xr_ref = hand_xr_ref + XR_TO_WORLD.T @ offset

        bridge.receive({
            "space": "pico", "seq": 1, "align": True,
            "pose": [*ctrl_xr_ref, *r_ctrl_ref.as_quat()],
            "left": {"kind": "controller", "points": [hand_xr_ref.tolist()]},
        })
        root, status, reg, _ = bridge.wrist()
        self.assertEqual(status, "右腕跟踪中")
        np.testing.assert_allclose(root, IDENTITY, atol=1e-12)
        np.testing.assert_allclose(reg["xr_from_world"][:3], hand_xr_ref, atol=1e-12)
        # 左手触碰真实手套 base_link 原点时，在世界坐标系中恰为 [0, 0, 0]
        np.testing.assert_allclose(bridge.left_hand()["points"], [[0., 0., 0.]], atol=1e-12)

        # 真实手套绕 base_link 原地做任意三维旋转 + 平移，计算刚性连接的手柄在 XR 空间的位姿
        r_hand_world = Rotation.from_euler("ZYX", [55., -35., 40.], degrees=True)
        t_hand_world = np.array([0.12, -0.08, 0.05])
        r_delta_xr = Rotation.from_matrix(XR_TO_WORLD.T @ r_hand_world.as_matrix() @ XR_TO_WORLD)
        r_ctrl_now = r_delta_xr * r_ctrl_ref
        hand_xr_now = hand_xr_ref + XR_TO_WORLD.T @ t_hand_world
        ctrl_xr_now = hand_xr_now + XR_TO_WORLD.T @ r_hand_world.apply(offset)

        bridge.receive({"space": "pico", "seq": 2, "pose": [*ctrl_xr_now, *r_ctrl_now.as_quat()]})
        root_now, _, _, _ = bridge.wrist()
        np.testing.assert_allclose(root_now[:3], t_hand_world, atol=1e-12)
        rot_err = (Rotation.from_quat(root_now[3:]) * r_hand_world.inv()).magnitude()
        self.assertLess(rot_err, 1e-12)

        # 纯原地旋转（t_hand_world = 0）时，手套原点位移严格为 0，不会甩出圆弧
        ctrl_xr_spin = hand_xr_ref + XR_TO_WORLD.T @ r_hand_world.apply(offset)
        bridge.receive({"space": "pico", "seq": 3, "pose": [*ctrl_xr_spin, *r_ctrl_now.as_quat()]})
        root_spin, _, _, _ = bridge.wrist()
        np.testing.assert_allclose(root_spin[:3], [0., 0., 0.], atol=1e-12)

    def test_motion_pivot_calibration_recovers_offset_and_rejects_bad_motion(self):
        import json
        from tempfile import TemporaryDirectory
        from urllib.request import Request, urlopen
        from scipy.spatial.transform import Rotation
        from ve33_quest import wrist_server

        true_offset = np.array([0.012, -0.018, 0.105])
        r_ctrl_ref = Rotation.from_euler("xyz", [32., -25., 18.], degrees=True)
        hand_xr_ref = np.array([0.1, 1.0, -0.5])
        ctrl_xr_ref = hand_xr_ref + XR_TO_WORLD.T @ true_offset

        bridge = WristBridge()
        bridge.receive({"space": "pico", "seq": 1, "align": True,
                        "pose": [*ctrl_xr_ref, *r_ctrl_ref.as_quat()]})
        bridge.wrist()

        # 仅绕单轴旋转（滚转）必须被拒绝，因为沿转轴方向偏移不可观
        bridge.start_offset_calibration()
        seq = 2
        for deg in np.linspace(-50., 50., 25):
            r_hand = Rotation.from_euler("x", deg, degrees=True)
            r_ctrl = Rotation.from_matrix(XR_TO_WORLD.T @ r_hand.as_matrix() @ XR_TO_WORLD) * r_ctrl_ref
            ctrl_xr = hand_xr_ref + XR_TO_WORLD.T @ r_hand.apply(true_offset)
            bridge.receive({"space": "pico", "seq": seq, "pose": [*ctrl_xr, *r_ctrl.as_quat()]})
            seq += 1
        res_1axis = bridge.finish_offset_calibration()
        self.assertTrue(res_1axis["active"])
        self.assertFalse(res_1axis["ready"])
        np.testing.assert_allclose(res_1axis["offset_m"], [0., 0., 0.])

        # 多轴原地旋转（带 1.5 mm 随机抖动 + 任意非零固定支点）应精确反推出真实偏移
        rng = np.random.default_rng(42)
        pivot_xr = hand_xr_ref + np.array([0.15, -0.08, 0.22])
        bridge.start_offset_calibration()
        angles = [
            *[(a, 0., 0.) for a in np.linspace(-45., 45., 12)],
            *[(0., b, 0.) for b in np.linspace(-40., 40., 12)],
            *[(a, b, c) for a, b, c in zip(np.linspace(-35., 35., 12), np.linspace(30., -30., 12), np.linspace(-25., 25., 12))],
        ]
        for ax, ay, az in angles:
            r_hand = Rotation.from_euler("xyz", [ax, ay, az], degrees=True)
            r_ctrl = Rotation.from_matrix(XR_TO_WORLD.T @ r_hand.as_matrix() @ XR_TO_WORLD) * r_ctrl_ref
            jitter = rng.normal(0., 0.0012, size=3)
            ctrl_xr = pivot_xr + jitter + XR_TO_WORLD.T @ r_hand.apply(true_offset)
            bridge.receive({"space": "pico", "seq": seq, "pose": [*ctrl_xr, *r_ctrl.as_quat()]})
            seq += 1

        # 通过 HTTP 控制接口完成标定并保存到临时配置文件
        class DummyControl:
            grasp = None
            record_ready = None
            def read(self):
                return {"fixed_wrist": False}
            def command(self, msg):
                raise AssertionError(f"unexpected command: {msg}")

        with TemporaryDirectory() as tmp:
            import socket
            with socket.socket() as sock:
                sock.bind(("127.0.0.1", 0))
                port = sock.getsockname()[1]
            cfg = Path(tmp) / "feedback.json"
            cfg.write_text(json.dumps({"hardness": 0.92, "wrist_tracker_offset_m": [0., 0., 0.]}))
            with wrist_server(bridge, port, control=DummyControl(), offset_config_path=cfg):
                def post(payload):
                    req = Request(f"http://127.0.0.1:{port}/api/v1/control",
                                  data=json.dumps(payload).encode(),
                                  headers={"Content-Type": "application/json"})
                    with urlopen(req) as resp:
                        return json.loads(resp.read().decode())

                finished = post({"action": "finish_offset_cal"})
                self.assertFalse(finished["active"])
                np.testing.assert_allclose(finished["offset_m"], true_offset, atol=0.003)
                self.assertLess(finished["rms_mm"], 4.0)
                for key in ("up", "down", "left", "right"):
                    self.assertGreaterEqual(finished["directions"][key], 0.99)

                saved = post({"action": "save_offset_cal"})
                saved_json = json.loads(cfg.read_text())
                np.testing.assert_allclose(saved_json["wrist_tracker_offset_m"], finished["offset_m"], atol=1e-5)
                self.assertIn("已保存", saved["message"])

                # 左手柄一键 toggle_offset_cal：第一次开始标定，采集多轴样本后第二次直接完成并自动保存
                toggled_start = post({"action": "toggle_offset_cal", "auto_save": True})
                self.assertTrue(toggled_start["active"])
                self.assertIsNotNone(toggled_start["pivot_world"])
                for ax, ay, az in angles:
                    r_hand = Rotation.from_euler("xyz", [ax, ay, az], degrees=True)
                    r_ctrl = Rotation.from_matrix(XR_TO_WORLD.T @ r_hand.as_matrix() @ XR_TO_WORLD) * r_ctrl_ref
                    ctrl_xr = pivot_xr + XR_TO_WORLD.T @ r_hand.apply(true_offset)
                    bridge.receive({"space": "pico", "seq": seq, "pose": [*ctrl_xr, *r_ctrl.as_quat()]})
                    seq += 1
                bridge.publish_scene({"version": 1, "frame": 1})
                with urlopen(f"http://127.0.0.1:{port}/api/v1/scene") as resp:
                    scene_payload = json.loads(resp.read().decode())
                self.assertTrue(scene_payload["wrist_offset"]["active"])
                self.assertTrue(scene_payload["wrist_offset"]["ready"])

                toggled_finish = post({"action": "toggle_offset_cal", "auto_save": True})
                self.assertFalse(toggled_finish["active"])
                self.assertIn("已保存", toggled_finish["message"])
                np.testing.assert_allclose(
                    json.loads(cfg.read_text())["wrist_tracker_offset_m"], true_offset, atol=0.002)

        # 应用标定后，原地旋转 50° 残余位移 < 3 mm，且 AR 注册原点已回移至真实手套原点
        r_check = Rotation.from_euler("ZYX", [45., -35., 30.], degrees=True)
        r_ctrl_check = Rotation.from_matrix(XR_TO_WORLD.T @ r_check.as_matrix() @ XR_TO_WORLD) * r_ctrl_ref
        ctrl_xr_check = hand_xr_ref + XR_TO_WORLD.T @ r_check.apply(true_offset)
        bridge.receive({"space": "pico", "seq": seq, "pose": [*ctrl_xr_check, *r_ctrl_check.as_quat()]})
        root_after, _, reg_after, _ = bridge.wrist()
        self.assertLess(np.linalg.norm(root_after[:3]), 0.003)
        np.testing.assert_allclose(reg_after["xr_from_world"][:3], hand_xr_ref, atol=0.003)

    def test_ar_overlay_6dof_calibration_glues_virtual_hand_at_any_heading(self):
        import json
        from tempfile import TemporaryDirectory
        from scipy.spatial.transform import Rotation

        # 真实硬件：手柄以任意三维位置 + 任意倾斜角固定在手套手背支架上
        true_c_hand = np.array([-0.015, 0.01, 0.11])
        true_r_h2c = Rotation.from_euler("xyz", [42., -33., 118.], degrees=True)

        # 用户面朝任意椅子偏航角 73°，眼前虚拟手静止在 hand_xr_hologram
        r_heading = Rotation.from_euler("y", 73., degrees=True)
        r_hand_holo = r_heading * Rotation.from_matrix(XR_TO_WORLD.T)
        p_hand_holo = np.array([0.45, 1.05, -0.32])
        hand_pose_holo = [*p_hand_holo, *r_hand_holo.as_quat()]

        # 用户把真实手套伸进虚拟手完全重合，此时真实手柄在 XR 空间的位姿
        p_ctrl_match = p_hand_holo + r_hand_holo.apply(true_c_hand)
        r_ctrl_match = r_hand_holo * true_r_h2c
        ctrl_pose_match = [*p_ctrl_match, *r_ctrl_match.as_quat()]

        bridge = WristBridge()
        st = bridge.start_overlay_calibration()
        self.assertTrue(st["overlay_active"])
        root_frozen, status_frozen, _, _ = bridge.wrist()
        np.testing.assert_allclose(root_frozen, IDENTITY)
        self.assertIn("标定", status_frozen)

        with TemporaryDirectory() as tmp:
            cfg = Path(tmp) / "feedback.json"
            cfg.write_text(json.dumps({"hardness": 0.92, "wrist_tracker_offset_m": [0., 0., 0.], "wrist_tracker_pose": None}))
            confirmed = bridge.confirm_overlay_calibration("xr-overlay", ctrl_pose_match, hand_pose_holo, auto_save_path=cfg)
            self.assertFalse(confirmed["overlay_active"])
            self.assertTrue(confirmed["calibrated_6dof"])
            np.testing.assert_allclose(confirmed["offset_m"], true_c_hand, atol=1e-5)

            saved = json.loads(cfg.read_text())
            self.assertIsNotNone(saved["wrist_tracker_pose"])

            # 标定确认瞬间：虚拟手在世界坐标原点且 AR 配准精确落在全息手位置
            root0, status0, reg0, _ = bridge.wrist()
            self.assertEqual(status0, "右腕跟踪中")
            np.testing.assert_allclose(root0, IDENTITY, atol=1e-12)
            np.testing.assert_allclose(reg0["xr_from_world"][:3], p_hand_holo, atol=1e-12)

            # 任意 6DoF 移动 + 翻转真实手套，验证 AR 渲染出的虚拟手与真实手套严格重合
            p_hand_now = np.array([0.62, 1.18, -0.11])
            r_hand_now = Rotation.from_euler("ZYX", [-55., 48., -72.], degrees=True)
            p_ctrl_now = p_hand_now + r_hand_now.apply(true_c_hand)
            r_ctrl_now = r_hand_now * true_r_h2c
            bridge.receive({"space": "xr-overlay", "seq": 2, "pose": [*p_ctrl_now, *r_ctrl_now.as_quat()]})
            root_now, _, reg_now, _ = bridge.wrist()
            p_w = np.asarray(reg_now["xr_from_world"][:3])
            r_w = Rotation.from_quat(reg_now["xr_from_world"][3:])
            rendered_hand_p = p_w + r_w.apply(root_now[:3])
            rendered_hand_r = r_w * Rotation.from_quat(root_now[3:])
            np.testing.assert_allclose(rendered_hand_p, p_hand_now, atol=1e-12)
            self.assertLess((rendered_hand_r * r_hand_now.inv()).magnitude(), 1e-12)

            # 下次重启加载保存的 6DoF 外参，换到 -120° 朝向重新对齐：桌面保持水平且虚拟手依旧贴合真手
            bridge2 = WristBridge(controller_to_hand=saved["wrist_tracker_pose"])
            r_new_hand = Rotation.from_euler("y", -120., degrees=True) * Rotation.from_matrix(XR_TO_WORLD.T) * Rotation.from_euler("x", 20., degrees=True)
            p_new_hand = np.array([-0.3, 0.95, 0.4])
            p_new_ctrl = p_new_hand + r_new_hand.apply(true_c_hand)
            r_new_ctrl = r_new_hand * true_r_h2c
            bridge2.receive({"space": "xr-next", "seq": 1, "align": True, "pose": [*p_new_ctrl, *r_new_ctrl.as_quat()]})
            root2, _, reg2, _ = bridge2.wrist()
            r_w2 = Rotation.from_quat(reg2["xr_from_world"][3:])
            # 验证世界重力方向 [0, 0, -1] 严格映射到真实竖直向下 [0, -1, 0]（桌面 100% 水平）
            np.testing.assert_allclose(r_w2.apply([0., 0., -1.]), [0., -1., 0.], atol=1e-12)
            rendered2_p = np.asarray(reg2["xr_from_world"][:3]) + r_w2.apply(root2[:3])
            rendered2_r = r_w2 * Rotation.from_quat(root2[3:])
            np.testing.assert_allclose(rendered2_p, p_new_hand, atol=1e-5)
            self.assertLess((rendered2_r * r_new_hand.inv()).magnitude(), 1e-5)

    def test_left_controller_toggle_feedback_command(self):
        import json
        import socket
        from threading import Event
        from urllib.request import Request, urlopen
        from ve33_quest import wrist_server

        class FakeControl:
            grasp = None
            record_ready = None
            def __init__(self):
                self.requested = Event()
                self.zero = Event()
                self.can_feedback = True
            def read(self):
                return {"can_feedback": self.can_feedback, "active": self.requested.is_set(),
                        "feedback_rearm_required": False}
            def command(self, msg):
                if msg["action"] == "enable":
                    self.requested.set()
                elif msg["action"] == "disable":
                    self.requested.clear()

        bridge = WristBridge()
        ctrl = FakeControl()
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        with wrist_server(bridge, port, control=ctrl):
            def toggle():
                req = Request(f"http://127.0.0.1:{port}/api/v1/control",
                              data=json.dumps({"action": "toggle_feedback"}).encode(),
                              headers={"Content-Type": "application/json"})
                with urlopen(req) as resp:
                    return json.loads(resp.read().decode())

            r1 = toggle()
            self.assertTrue(ctrl.requested.is_set())
            self.assertIn("已开启", r1["message"])
            r2 = toggle()
            self.assertFalse(ctrl.requested.is_set())
            self.assertIn("已关闭", r2["message"])

    def test_tabletop_disables_hand_table_collision_while_supporting_objects(self):
        from types import SimpleNamespace
        import superdex.physics as sdp
        import vtk
        from ve33_tabletop import Tabletop

        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene("桌面手部免碰撞测试")
            scene.set_gravity([0., 0., -9.81])
            view = SimpleNamespace(scene=vtk.vtkRenderer())
            table = Tabletop(sdp, scene, view)
            self.assertIn("duck_lamp", table.groups)
            self.assertIn("paper_cup", table.groups)
            self.assertFalse(scene.is_layer_contact_enabled("RightHand", "tabletop"))
            self.assertFalse(scene.is_layer_contact_enabled("tabletop", "RightHand"))
            self.assertFalse(scene.is_layer_contact_enabled("LeftHand", "tabletop"))
            self.assertTrue(scene.is_layer_contact_enabled("TabletopObj", "tabletop"))
            self.assertTrue(scene.is_layer_contact_enabled("tabletop", "TabletopObj"))
            self.assertTrue(scene.is_layer_contact_enabled("RightHand", "TabletopObj"))
            # 确认 8 个物体（含实心碰撞纸杯和小黄鸭）在重力下稳定站立不穿桌、不互撞飞出
            initial_z = {n: float(a.get_root_transform().translation[2]) for n, a in table.dynamic_actors}
            for _ in range(120):
                scene.step(1 / 240)
            for name, actor in table.dynamic_actors:
                z = float(actor.get_root_transform().translation[2])
                self.assertLess(abs(z - initial_z[name]), 0.008, f"{name} 沉降异常: {z} vs {initial_z[name]}")
        finally:
            sdp.shutdown()

    def test_web_control_task_tab_switching_disarms_and_commits_cleanly(self):
        import multiprocessing as mp
        from ve33_web_control import WebControl

        ctx = mp.get_context("spawn")
        align = ctx.Event()
        ctrl = WebControl(ctx, align, default_enabled=True, allow_reset=True, local_grasp=True,
                          switchable_tasks=("local_grasp", "tabletop"), initial_task="local_grasp")
        self.assertEqual(ctrl.current_task(), "local_grasp")
        self.assertTrue(ctrl.is_local_grasp())
        self.assertIsNotNone(ctrl.grasp_request())
        self.assertTrue(ctrl.requested.is_set())

        ctrl.command({"action": "switch_task", "task": "tabletop"})
        self.assertFalse(ctrl.requested.is_set())
        self.assertEqual(ctrl.pending_task(), "tabletop")

        ctrl.commit_task("tabletop")
        self.assertEqual(ctrl.current_task(), "tabletop")
        self.assertFalse(ctrl.is_local_grasp())
        self.assertIsNone(ctrl.grasp_request())
        self.assertIsNone(ctrl.pending_task())

        ctrl.command({"action": "align"})
        self.assertTrue(align.is_set())
        ctrl.command({"action": "switch_task", "task": "local_grasp"})
        ctrl.commit_task("local_grasp")
        self.assertTrue(ctrl.is_local_grasp())
        self.assertIsNotNone(ctrl.grasp_request())


    def test_left_controller_proxy_can_push_the_official_cube(self):
        from types import SimpleNamespace
        from scipy.spatial.transform import Rotation
        import superdex.physics as sdp
        import vtk
        from ve33_cube import PuzzleCube, link_poses
        from ve33_left_hand import LeftHandSupport

        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene("左手支撑测试")
            scene.set_gravity([0, 0, 0])
            folder = Path(__file__).resolve().parents[2] / "assets/prefabs/puzzle_cube_quest"
            cube = PuzzleCube(sdp, scene, folder, SimpleNamespace(scene=vtk.vtkRenderer()))
            left = LeftHandSupport(sdp, scene, cube.contact_actors)
            initial = link_poses(cube.names, cube.transforms)
            for x in np.linspace(.16, .211, 75):
                left.update({"kind": "controller", "points": [[x, -.032, .050]]}, 1/240)
                scene.step(1 / 240)
            cube.update()
            moved = link_poses(cube.names, cube.transforms)
            angle = max((Rotation.from_quat(moved[n][3:]) *
                         Rotation.from_quat(initial[n][3:]).inv()).magnitude()
                        for n in cube.names if n.endswith("_shell_link"))
            self.assertGreater(angle, .01)
            self.assertEqual(left.update(None, 1/240), [None] * 6)
        finally:
            sdp.shutdown()

    def test_cube_switch_keeps_layers_and_can_pin_after_physical_motion(self):
        from types import SimpleNamespace
        from scipy.spatial.transform import Rotation
        import superdex.physics as sdp
        import vtk
        from ve33_cube import PuzzleCube, link_poses
        from ve33_left_hand import LeftHandSupport

        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene("魔方固定与自由抓取测试")
            scene.set_gravity([0, 0, 0])
            folder = Path(__file__).resolve().parents[2] / "assets/prefabs/puzzle_cube_quest"
            cube = PuzzleCube(sdp, scene, folder, SimpleNamespace(scene=vtk.vtkRenderer()), switchable=True)
            left = LeftHandSupport(sdp, scene, cube.contact_actors)

            def read():
                cube.update()
                return link_poses(cube.names, cube.transforms)

            self.assertEqual(cube.mode, "fixed")
            initial_links = read()
            initial = initial_links["center_link"]
            fixed_constraints = scene.get_num_constraints()
            cube.set_fixed(False)
            self.assertEqual(cube.mode, "free")
            self.assertEqual(scene.get_num_constraints(), fixed_constraints - 2)
            for x in np.linspace(.16, .245, 95):
                left.update({"kind": "controller", "points": [[x, -.032, .050]]}, 1/240)
                scene.step(1 / 240)
            left.update(None, 1/240)
            moved = read()["center_link"]
            self.assertGreater(np.linalg.norm(np.subtract(initial[:3], moved[:3])), .01)
            self.assertGreater((Rotation.from_quat(moved[3:]) * Rotation.from_quat(initial[3:]).inv()).magnitude(), .01)
            cube.set_fixed(True)
            self.assertEqual(cube.mode, "fixed")
            self.assertEqual(scene.get_num_constraints(), fixed_constraints)
            np.testing.assert_allclose(read()["center_link"], moved, atol=1e-5)
            for _ in range(40):
                scene.step(1 / 240)
            np.testing.assert_allclose(read()["center_link"], moved, atol=1e-4)
            cube.set_fixed(False)
            cube.reset()
            self.assertEqual(cube.mode, "fixed")
            reset_links = read()
            for link, pose in initial_links.items():
                np.testing.assert_allclose(reset_links[link], pose, atol=1e-5)
        finally:
            sdp.shutdown()

    def test_free_cube_lands_and_settles_on_table_after_a_bump(self):
        from types import SimpleNamespace
        import superdex.physics as sdp
        import vtk
        from ve33_cube import PuzzleCube, link_poses
        from ve33_tabletop import Tabletop

        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene("魔方桌面着陆测试")
            scene.set_gravity([0., 0., -9.81])
            view = SimpleNamespace(scene=vtk.vtkRenderer())
            folder = Path(__file__).resolve().parents[2] / "assets/prefabs/puzzle_cube_quest"
            cube = PuzzleCube(sdp, scene, folder, view, switchable=True)
            table = Tabletop(sdp, scene, view, puzzle_shells=cube.contact_actors)
            _, table_actor = table.contact_actors[0]
            top = float(table_actor.get_aabb_world().max[2])
            cube.set_fixed(False)
            root = cube.actor.get_articulated_shape_info().dof_info[0]
            pos = root.get_trans_offset()
            raised = list(cube.initial_pose)
            raised[pos + 2] += .09
            cube.actor.set_articulated_pose_from_joints(raised)
            velocities = [0.] * cube.actor.get_num_dofs()
            velocities[pos] = .24
            cube.actor.set_articulated_joint_velocities(velocities)

            def center():
                cube.update()
                return np.asarray(link_poses(cube.names, cube.transforms)["center_link"][:3])

            start = center()
            for _ in range(220):
                scene.step(1/240)
            landed = center()
            self.assertGreater(landed[0] - start[0], .01)
            self.assertGreater(start[2] - landed[2], .06)
            self.assertGreater(landed[2] - top, .024)
            self.assertLess(landed[2] - top, .05)
            self.assertGreater(min(shell.get_aabb_world().min[2] for _, shell in cube.contact_actors), top - .004)
            for _ in range(96):
                scene.step(1/240)
            self.assertLess(np.linalg.norm(center() - landed), .004)
        finally:
            sdp.shutdown()


if __name__ == "__main__":
    unittest.main()
