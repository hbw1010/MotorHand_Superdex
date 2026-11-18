"""固定腕部的掌心测试从同一入口运行，不需要头显或物理串口。"""
import json
import multiprocessing as mp
from pathlib import Path
import socket
import sys
import time
import unittest
from urllib.request import Request, urlopen

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motorhand.feedback import MAPPING, PROXY_FRAME_SIZE
from motorhand.kinematics import GloveKinematics
from motorhand.mapping import GloveMapping, MODEL_MOTOR_JOINTS
from ve33_g4_view import MODEL_DIR, ROOT, render
from ve33_web_control import WebControl


class FixedPalmTest(unittest.TestCase):
    def test_ball_size_uses_same_geometry_and_preserves_palm_clearance(self):
        from types import SimpleNamespace
        import superdex.physics as sdp
        import vtk
        from ve33_palm_object import PalmBall

        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene("尺寸验证")
            view = SimpleNamespace(scene=vtk.vtkRenderer())
            env = PalmBall(sdp, scene, view, 60)
            _, actor = env.contact_actors[0]
            bounds = actor.get_aabb_world()
            np.testing.assert_allclose(np.asarray(bounds.max) - np.asarray(bounds.min), [.06]*3)
            self.assertAlmostEqual(bounds.max[2], -.007, places=6)
            part, = env.groups["palm_ball"].GetParts()
            part.GetMapper().Update()
            rendered = part.GetMapper().GetInput().GetBounds()
            self.assertAlmostEqual(rendered[5] - rendered[4], .06, places=6)
        finally:
            sdp.shutdown()

    def test_fixed_wrist_and_palm_contact_without_xr(self):
        self.check_fixed_scene(False)

    def test_official_puzzle_in_the_fixed_palm_without_xr(self):
        self.check_fixed_scene(True)

    def check_fixed_scene(self, official):
        mapping = GloveMapping.load(MAPPING)
        solver = GloveKinematics(MODEL_DIR, mapping)
        ctx = mp.get_context("spawn")
        stop = ctx.Event()
        with socket.socket() as endpoint:
            endpoint.bind(("127.0.0.1", 0))
            port = endpoint.getsockname()[1]
        api = f"http://127.0.0.1:{port}/api/v1/"
        target = ctx.Array("d", len(solver.joint_names) + 1)
        display = ctx.Array("d", 13)
        contact = ctx.Array("d", PROXY_FRAME_SIZE)
        wrist_state = ctx.Array("c", 128)
        control = WebControl(ctx, None, allow_reset=True)
        args = (solver.joint_names, MODEL_MOTOR_JOINTS, target, display, stop,
                False, port, ROOT / "assets/prefabs/puzzle_cube_quest", None,
                wrist_state, True, contact, False, not official, official, 45., None, control)
        process = ctx.Process(target=render, args=args)
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

        until = time.monotonic() + 30
        while time.monotonic() < until and process.is_alive():
            try:
                initial = frame()
                break
            except OSError:
                time.sleep(.04)
        else:
            self.fail("掌心场景未启动")
        if official:
            self.assertEqual(initial["scene_kind"], "official_puzzle")
            self.assertEqual(len(initial["puzzle_cube"]), 16)
            np.testing.assert_allclose(initial["puzzle_cube"]["center_link"][:3],
                                       [.072, -.005, -.055], atol=1e-6)
        else:
            self.assertEqual(initial["scene_kind"], "palm_test")
            self.assertEqual(set(initial["objects"]), {"palm_ball"})
            np.testing.assert_allclose(initial["objects"]["palm_ball"][:3], [.072, -.005, -.0295])

        # 本地模式不消费这个外部定位，即使外部浏览器仍在上传。
        packet = {"space": "irrelevant", "seq": 1, "pose": [1, 1, 1, 0, 0, 0, 1], "align": True}
        request = Request(api + "wrist", json.dumps(packet).encode(), {"Content-Type": "application/json"})
        with urlopen(request, timeout=2):
            pass

        grip = np.array([.20, .15, .60, .60, .80, 1.20, .80, 1.20, .80, 1.20, .80, 1.20])
        blocked = set()
        for fraction in np.linspace(0., 1., 80):
            q = grip * fraction
            target[:] = [time.monotonic_ns(), *solver.full_pose(q, None, mapping)]
            time.sleep(.027)
            state = frame()
            np.testing.assert_allclose(state["right_hand"]["base_link"], initial["right_hand"]["base_link"], atol=1e-6)
            self.assertEqual(state["wrist_status"], "固定腕部")
            blocked.update(state["blocked_motor_ids"])
        self.assertTrue(blocked.intersection(range(5, 13)))
        self.assertTrue(contact[2], "力反馈必须可以使用本地仿真帧")
        self.assertLess(time.monotonic_ns() - contact[1], 100_000_000)

        request = Request(api + "control", b'{"action":"reset_scene"}', {"Content-Type": "application/json"})
        with urlopen(request, timeout=2) as response:
            self.assertEqual(response.status, 204)
        until = time.monotonic() + 2
        state = frame()
        while state["scene_revision"] == 0 and time.monotonic() < until:
            time.sleep(.03)
            state = frame()
        self.assertEqual(state["scene_revision"], 1)
        self.assertNotIn("cube_mode", state)
        self.assertTrue(control.requested.is_set())
        if not official:
            self.assertEqual(state["objects"], initial["objects"])


if __name__ == "__main__":
    unittest.main()
