"""以真实物理场景回放按压、横移、撤手。运行 python tools/tests/test_ve33_contact_stream.py。"""

import json
import multiprocessing as mp
from pathlib import Path
import socket
import sys
import time
import unittest
from urllib.request import Request, urlopen
from unittest.mock import Mock

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ve33_g4_view import MODEL_DIR, ROOT, GloveMapping, GloveKinematics, MAPPING, MODEL_MOTOR_JOINTS, render
from ve33_quest import XR_TO_WORLD
from ve33_web_control import WebControl
from motorhand.feedback import (CONFIG, DIRECTION_END, FORCE_FRAME_SIZE, FORCE_TORQUE_END, PROXY_FRAME_SIZE, SURFACE_END,
                                ContactForceFeedbackLoop, surface_stiffness)
from motorhand.impedance import DriverImpedancePlanner


class ContactStreamTest(unittest.TestCase):
    def test_press_slide_release(self):
        mapping = GloveMapping.load(MAPPING)
        solver = GloveKinematics(MODEL_DIR, mapping)
        ctx = mp.get_context("spawn")
        stop = ctx.Event()
        with socket.socket() as endpoint:
            endpoint.bind(("127.0.0.1", 0))
            port = endpoint.getsockname()[1]
        api = f"http://127.0.0.1:{port}/api/v1/"
        cube = ROOT / "assets/prefabs/puzzle_cube_quest"
        target = ctx.Array("d", len(solver.joint_names) + 1)
        display = ctx.Array("d", 13)
        proxy_frame = ctx.Array("d", PROXY_FRAME_SIZE)
        force_frame = ctx.Array("d", FORCE_FRAME_SIZE)
        align = ctx.Event()
        control = WebControl(ctx, align, default_enabled=False, cube_switch=True)
        render_args = (
            solver.joint_names, MODEL_MOTOR_JOINTS, target,
            display, stop, False, port, cube, align, ctx.Array("c", 128), True, proxy_frame,
            False, False, False, 45., None, control, None, force_frame,
        )
        process = ctx.Process(target=render, args=render_args)
        process.start()

        def shutdown():
            stop.set()
            process.join(5)
            if process.is_alive():
                process.terminate()
                process.join()
        self.addCleanup(shutdown)

        def frame():
            with urlopen(api + "scene", timeout=2) as response:
                return json.load(response)

        def cube_command(action, **fields):
            request = Request(api + "control", json.dumps({"action": action, **fields}).encode(),
                              {"Content-Type": "application/json"})
            with urlopen(request, timeout=2) as response:
                self.assertEqual(response.status, 204)

        sequence = 0
        def move(position):
            nonlocal sequence
            sequence += 1
            target[0] = time.monotonic_ns()
            pose = [*(XR_TO_WORLD.T @ np.asarray(position, dtype=float)), 0, 0, 0, 1]
            data = {"space": "replay", "seq": sequence, "pose": pose, "align": sequence == 1}
            request = Request(api + "wrist", json.dumps(data).encode(), {"Content-Type": "application/json"})
            with urlopen(request, timeout=2):
                pass

        deadline = time.monotonic() + 40
        while time.monotonic() < deadline and process.is_alive():
            try:
                frame()
                break
            except OSError:
                time.sleep(.05)
        else:
            self.fail("回放场景未就绪")
        initial_scene = frame()
        move([0, 0, 0])
        time.sleep(.1)
        move([0, 0, .173])
        time.sleep(1)
        above = np.array([.088, -.036, .173])
        move(above)
        time.sleep(1.5)
        free = frame()
        self.assertFalse(free["hand_cube_contacts"])
        for depth in np.linspace(0, .13, 65):
            move(above - [0, 0, depth])
            time.sleep(.025)

        pressed = above - [0, 0, .13]
        frames = {}
        torque_samples = []
        for offset in np.linspace(0, .03, 100):
            move(pressed + [0, offset, 0])
            sample = frame()
            frames[sample["frame"]] = sample
            effort = force_frame[:]
            if effort[2]:
                torque_samples.append(max(map(abs, effort[3:FORCE_TORQUE_END])))
            time.sleep(.015)
        touching = [f for f in frames.values() if f["hand_cube_contacts"]]
        self.assertGreater(len(touching), 15)
        self.assertTrue(torque_samples and max(torque_samples) > .01,
                        "真实碰撞应该产生可下发的闭链电机轴力矩")
        loads = [(f, link, vector) for f in touching
                 for link, vector in f["right_contact_force_n"].items()]
        self.assertTrue(loads, "读到至少一个引擎计算的真实仿真接触合力")
        for f, link, vector in loads:
            self.assertIn(link, f["right_hand"])
            self.assertEqual(len(vector), 3)
            self.assertTrue(np.isfinite(vector).all())
        # 指腹外表面取自同一份 STL，检测角块实体内的深穿透。
        import trimesh
        from xml.etree import ElementTree
        names = ("index_dip_link", "middle_dip_link", "ring_dip_link", "pinky_dip_link")
        visuals = ElementTree.parse(MODEL_DIR / "hand_tree_only.urdf")
        samples = {}
        for name in names:
            visual = visuals.find(f"link[@name='{name}']/visual")
            mesh = trimesh.load_mesh(MODEL_DIR / visual.find("geometry/mesh").get("filename"))
            origin = visual.find("origin")
            shift = np.fromstring(origin.get("xyz"), sep=" ")
            rotation = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy"), sep=" "))
            samples[name] = rotation.apply(mesh.vertices) + shift
        depth = 0.
        for sample in touching:
            points = np.concatenate([Rotation.from_quat(sample["right_hand"][name][3:]).apply(vertices)
                                     + sample["right_hand"][name][:3] for name, vertices in samples.items()])
            for name, pose in sample["puzzle_cube"].items():
                if not name.endswith("_shell_link"):
                    continue
                local = Rotation.from_quat(pose[3:]).inv().apply(points - pose[:3])
                inside = np.minimum(local, .034-local).min(axis=1)
                depth = max(depth, float(inside.max()))
        print(f"按压和横移期间，指腹进入角块外包围盒的最大深度 {depth * 1000:.2f} mm")
        self.assertLess(depth, .003, f"指腹进入角块 {depth * 1000:.1f} mm")
        # 电机输入持续为零。用世界坐标回读每个相邻指节的相对旋转，腕部可受力退让。
        adjacent = [(f"{finger}_{parent}_link", f"{finger}_{child}_link")
                    for finger in ("index", "middle", "ring", "pinky")
                    for parent, child in (("mount", "mcp"), ("mcp", "pip"), ("pip", "dip"))]
        adjacent += [("thumb_cmc_flex_link", "thumb_cmc_abd_link"),
                     ("thumb_cmc_abd_link", "thumb_mcp_link"), ("thumb_mcp_link", "thumb_ip_link")]
        for parent, child in adjacent:
            def angle(f):
                a = Rotation.from_quat(f["right_hand"][parent][3:])
                b = Rotation.from_quat(f["right_hand"][child][3:])
                return a.inv() * b
            neutral = angle(free)
            self.assertLess(max((neutral.inv() * angle(f)).magnitude() for f in touching), 1e-4)
        np.testing.assert_allclose(free["puzzle_cube"]["center_link"], sample["puzzle_cube"]["center_link"], atol=1e-4)
        interval = np.diff([f["time_monotonic_s"] for f in frames.values()])
        self.assertLess(np.percentile(interval, 95), .10)
        self.assertLess(max(interval), .25)
        rotations = Rotation.from_quat([f["puzzle_cube"]["corner5_shell_link"][3:] for f in touching])
        self.assertGreater(max((rotations * rotations[0].inv()).magnitude()), .005)
        # 被压住时主动闭合四指，检查物理回读与反馈使用同一个受阻面。
        q = np.zeros(12)
        held = []
        for value in np.linspace(.005, .20, 32):
            q[4:] = value
            tree = solver.full_pose(q, None, mapping)
            target[:] = [time.monotonic_ns(), *tree]
            move(pressed + [0, .03, 0])
            time.sleep(.035)
            _, _, tracking, *surface = proxy_frame[:SURFACE_END]
            real_proxy = np.array(display[1:])
            active = np.isfinite(surface) & (np.array(surface) < q-.01)
            if tracking and active.any():
                held.append(np.max(np.abs(real_proxy[active]-np.array(surface)[active])))
        self.assertTrue(held, "继续闭合时应该有受阻的电机轴")
        self.assertLess(min(held), 1e-4)
        sample = frame()
        self.assertTrue(sample["blocked_motor_ids"])
        self.assertTrue(sample["target_cube_contacts"], "阻抗应使用未限位的实测目标接触")
        _, _, tracking, *surface = proxy_frame[:SURFACE_END]
        self.assertTrue(tracking)
        planner = DriverImpedancePlanner(CONFIG["driver_impedance"], CONFIG["torque_limit_nm"])
        blocks = {i: (1, angle, surface_stiffness()) for i, angle in enumerate(surface) if np.isfinite(angle)}
        nominal = {i + 1: round(command.torque(q[i]), 3) for i, command in
                   enumerate(planner.plan(q, np.zeros(12), blocks)) if i in blocks}
        print("魔方持续屈指的受阻角与名义静态阻力", blocks, nominal)
        self.assertGreaterEqual(sum(abs(torque) >= 9.45 for torque in nominal.values()), 4)

        # 头显退出后，固定最后目标腕位姿，继续计算真实手指和同一个魔方的交互。
        sequence += 1
        paused = {"space": "replay", "seq": sequence, "pose": None, "paused": True}
        request = Request(api + "wrist", json.dumps(paused).encode(), {"Content-Type": "application/json"})
        with urlopen(request, timeout=2):
            pass
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        raw = {m.motor_id: m.zero_deg for m in mapping.motors}
        gateway = Mock()
        feedback = ContactForceFeedbackLoop(gateway, axes)
        grip = solver.full_pose(q, None, mapping)
        peak = 0.
        until = time.monotonic() + 2.
        while time.monotonic() < until and peak <= .01:
            stamp = time.monotonic_ns()
            target[:] = [stamp, *grip]
            feedback.follow_request(q, raw, stamp, force_frame[:], True)
            if feedback.active and feedback.nominal is not None:
                peak = max(peak, max(map(abs, feedback.nominal)))
            time.sleep(1. / CONFIG["control_hz"])
        self.assertEqual(frame()["wrist_received_ns"], 0)
        self.assertTrue(frame()["blocked_motor_ids"])
        self.assertGreater(peak, .01, feedback.message)
        gateway.send.assert_called()
        gateway.release.assert_not_called()

        move(above)
        time.sleep(3)
        self.assertFalse(frame()["hand_cube_contacts"])

        def publish(q):
            target[:] = [time.monotonic_ns(), *solver.full_pose(q, None, mapping)]

        # 从角块下方伸指，实际碰撞发生在手指外侧。先闭起，接近后再伸开。
        q[:] = 0
        q[4:] = .30
        publish(q)
        underneath = np.array([.09, -.024, -.012])
        for _ in range(14):
            move(underneath)
            time.sleep(.045)
        stop_points = {}
        for angle_rad in np.linspace(.30, 0., 32):
            q[4:] = angle_rad
            publish(q)
            move(underneath)
            time.sleep(.042)
            current = frame()
            for mid in current["blocked_extension_motor_ids"]:
                stop_points[mid] = display[mid]
        self.assertTrue(stop_points, "伸指时手背侧碰撞应限制伸展")
        self.assertGreater(max(stop_points.values()), 0.01)
        extension_blocks = {mid - 1: (-1, angle, surface_stiffness()) for mid, angle in stop_points.items()}
        extension = planner.plan(q, np.zeros(12), extension_blocks)
        self.assertGreater(max(extension[mid - 1].torque(q[mid - 1]) for mid in stop_points), 9.45)
        print("手指外侧受阻轴与显示角 rad", stop_points)

        # 保持电机角不变，用掌部靠近，检查掌部触觉标志进入共享帧。
        palm_approach = np.array([.188, -.015, .15])
        for _ in range(14):
            move(palm_approach)
            time.sleep(.045)
        seen = False
        for depth in np.linspace(0, .12, 38):
            move(palm_approach - [0, 0, depth])
            time.sleep(.043)
            if frame()["palm_contact"] and proxy_frame[DIRECTION_END]:
                seen = True
                break
        self.assertTrue(seen, "掌部靠近时应该发布独立的触觉标志")

        touching_wrist = np.array(frame()["right_hand"]["base_link"][:3])
        cube_command("cube_mode", mode="free")
        until = time.monotonic() + 2
        while frame()["cube_mode"] != "free" and time.monotonic() < until:
            time.sleep(.02)
        self.assertEqual(frame()["cube_mode"], "free")
        self.assertEqual(frame()["finger_control"], "physics")
        lifted = above + [0, 0, .08]
        for _ in range(18):
            move(lifted)
            time.sleep(.027)
        lifted_frame = frame()
        np.testing.assert_allclose(lifted_frame["wrist_target"][:3], lifted, atol=1e-6)
        self.assertGreater(lifted_frame["right_hand"]["base_link"][2] - touching_wrist[2], .04)
        displaced = frame()["right_hand"]["base_link"]
        home = initial_scene["right_hand"]["base_link"]
        self.assertGreater(np.linalg.norm(np.subtract(displaced[:3], home[:3])), .05)
        move(lifted)
        cube_command("reset_scene")
        until = time.monotonic() + 2
        while frame()["scene_revision"] == 0 and time.monotonic() < until:
            time.sleep(.02)
        with urlopen(api + "control", timeout=2) as response:
            state = json.load(response)
        self.assertEqual(state["scene_revision"], 1)
        self.assertEqual(state["cube_mode"], "fixed")
        self.assertEqual(frame()["finger_control"], "contact_guard")
        # 重力会让角块在复位后重新落到预载平衡，比较稳定后的实际位姿。
        move(lifted)
        time.sleep(.30)
        restored = frame()
        np.testing.assert_allclose(restored["right_hand"]["base_link"], home, atol=1e-4)
        self.assertEqual(restored["objects"], initial_scene["objects"])
        reset_cube = restored["puzzle_cube"]
        for link, pose in free["puzzle_cube"].items():
            actual = reset_cube[link]
            np.testing.assert_allclose(actual[:3], pose[:3], atol=1e-4)
            difference = Rotation.from_quat(actual[3:]) * Rotation.from_quat(pose[3:]).inv()
            self.assertLess(difference.magnitude(), .005)
        self.assertFalse(control.requested.is_set())



if __name__ == "__main__":
    unittest.main()
