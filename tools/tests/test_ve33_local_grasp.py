"""验证本地抓握物体、软球回弹和配置边界。"""

from pathlib import Path
import multiprocessing as mp
import json
import os
import socket
import subprocess
import sys
import time
import unittest
from tempfile import TemporaryDirectory
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class LocalGraspTest(unittest.TestCase):
    def test_local_browser_entry_without_a_serial_port(self):
        root = Path(__file__).resolve().parents[2]
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        base = f"http://127.0.0.1:{port}/api/v1/"
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        process = subprocess.Popen([sys.executable, str(root / "tools/ve33_g4_view.py"),
                                    "--local-grasp", "--demo", "--quest-port", str(port)],
                                   cwd=root, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
                                   env={**os.environ, "XDG_STATE_HOME": directory.name})

        def stop():
            if process.poll() is None:
                process.send_signal(2)
            try:
                process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
        self.addCleanup(stop)

        def read(path, data=None):
            req = Request(base + path, json.dumps(data).encode() if data else None,
                          {"Content-Type": "application/json"})
            with urlopen(req, timeout=2) as response:
                return response.status if data else json.load(response)

        for _ in range(150):
            if process.poll() is not None:
                self.fail(process.stderr.read())
            try:
                state = read("control")
                if state.get("grasp") and state.get("online") == 12:
                    break
            except URLError:
                pass
            time.sleep(.06)
        else:
            self.fail("本地演示网页未启动")
        self.assertTrue(state["demo"])
        self.assertEqual(state["grasp_backend"], "cpp")
        self.assertFalse(state["can_feedback"])
        self.assertTrue(state["fixed_wrist"])
        self.assertFalse(state["can_align"])
        self.assertEqual(state["grasp"]["kind"], "ball_soft")
        with self.assertRaises(HTTPError) as err:
            read("wrist", {"space": "xr", "seq": 1, "pose": [0, 0, 0, 0, 0, 0, 1]})
        self.assertEqual(err.exception.code, 404)
        read("control", {"action": "demo_grip", "value": 0})
        read("control", {"action": "reset_scene"})
        for _ in range(30):
            frame = read("scene")
            if frame["scene_revision"] and not frame["hand_cube_contacts"]:
                break
            time.sleep(.05)
        self.assertFalse(frame["hand_cube_contacts"])
        wrist = np.asarray(frame["right_hand"]["base_link"])
        upright = Rotation.from_quat(wrist[3:])
        np.testing.assert_allclose(upright.apply([1, 0, 0]), [0, 0, 1], atol=1e-6)
        np.testing.assert_allclose(upright.apply([0, 0, -1]), [0, -1, 0], atol=1e-6)
        np.testing.assert_allclose(wrist, frame["wrist_target"], atol=1e-6)
        table = Rotation.from_quat(frame["objects"]["table"][3:])
        np.testing.assert_allclose(table.apply([0, 0, 1]), [0, 0, 1], atol=1e-6)
        center = np.asarray(frame["grasp_object"]["pose"][:3])
        np.testing.assert_allclose(upright.inv().apply(center - wrist[:3]),
                                   [.072, -.005, -.0295], atol=.001)
        for level in (.2, .4, .6, .8, 1.):
            read("control", {"action": "demo_grip", "value": level})
            time.sleep(.25)
        for _ in range(40):
            frame = read("scene")
            state = read("control")
            compressed = np.ptp(frame["grasp_object"]["vertices"][2::3])
            if compressed < .0445 and max(abs(row["contact_motor_nm"] or 0) for row in state["rows"]) > .01:
                break
            time.sleep(.05)
        self.assertLess(compressed, .0445)
        self.assertIn("grasp_object", frame["hand_cube_contacts"])
        self.assertGreater(max(abs(row["contact_motor_nm"] or 0) for row in state["rows"]), .01)
        self.assertFalse(state["active"])
        self.assertTrue(all(row["sent_motor_nm"] is None for row in state["rows"]))
        read("control", {"action": "demo_grip", "value": 0})
        for _ in range(60):
            frame = read("scene")
            state = read("control")
            recovered = np.ptp(frame["grasp_object"]["vertices"][2::3])
            if not frame["hand_cube_contacts"] and recovered > .044 and all(row["nominal_nm"] == 0 for row in state["rows"]):
                break
            time.sleep(.05)
        self.assertFalse(frame["hand_cube_contacts"])
        self.assertGreater(recovered, .044)
        self.assertTrue(all(row["nominal_nm"] == 0 for row in state["rows"]))
        read("control", {"action": "configure_grasp", "values": {"size_mm": 90, "hardness": 1}})
        read("control", {"action": "reset_scene"})
        for _ in range(50):
            frame = read("scene")
            if frame["grasp_object"]["size_mm"] == 90 and not frame["hand_cube_contacts"]:
                break
            time.sleep(.05)
        self.assertEqual(frame["grasp_object"]["size_mm"], 90)
        for step in range(1, 8):
            read("control", {"action": "demo_grip", "value": step / 10})
            time.sleep(.35)
        time.sleep(1.)
        loaded = 0
        samples = 32
        for _ in range(samples):
            state = read("control")
            loaded += any(abs(row["nominal_nm"] or 0) >= .01 for row in state["rows"])
            time.sleep(.055)
        self.assertGreaterEqual(loaded, 29, f"90 mm 软球持续抓握时仅 {loaded}/{samples} 次有反馈")
        read("control", {"action": "demo_grip", "value": 0})
        read("control", {"action": "configure_grasp", "values": {"kind": "ball_hard", "size_mm": 55}})
        for _ in range(35):
            frame = read("scene")
            if frame["grasp_object"]["kind"] == "ball_hard":
                break
            time.sleep(.05)
        self.assertEqual(frame["grasp_object"]["size_mm"], 55)
        self.assertEqual(frame["finger_control"], "physics")
        self.assertEqual(frame["wrist_status"], "固定腕部")
        read("control", {"action": "start_recording"})
        time.sleep(.25)
        read("control", {"action": "stop_recording"})
        for _ in range(30):
            if read("control").get("recording_file"):
                break
            time.sleep(.05)
        with urlopen(base + "recording.csv", timeout=2) as response:
            csv = response.read().decode("utf-8-sig")
        self.assertIn("contact_motor_nm", csv)
        self.assertIn("position_motor_nm", csv)
        self.assertIn("encoder_rad,contact_encoder_rad", csv.splitlines()[0])
        self.assertIn("feedback_handoffs,last_feedback_handoff_ms", csv.splitlines()[0])
        self.assertIn("ball_hard", csv)
        self.assertGreater(len(csv.splitlines()), 13)

        read("control", {"action": "configure_grasp", "values": {
            "kind": "official_quest_cube", "layout": "palm", "locked": True}})
        for _ in range(100):
            if read("scene")["grasp_object"]["kind"] == "official_quest_cube":
                break
            time.sleep(.05)
        else:
            self.fail("官方魔方未完成加载")
        read("control", {"action": "reset_scene"})
        time.sleep(.4)
        for grip in (.17, .34, .51, .68, .85):
            read("control", {"action": "demo_grip", "value": grip})
            time.sleep(.25)
        time.sleep(.5)
        fresh = 0
        for _ in range(20):
            state = read("control")
            fresh += any(abs(row["contact_motor_nm"] or 0.) >= .01 for row in state["rows"])
            time.sleep(.06)
        self.assertGreaterEqual(fresh, 15, f"固定腕部握持官方魔方时仅 {fresh}/20 次有新鲜接触力矩")

    def test_control_merges_settings_and_resets_placement(self):
        from ve33_web_control import WebControl

        control = WebControl(mp.get_context("spawn"), None, allow_reset=True, local_grasp=True)
        self.assertEqual(control.grasp_request()["contact_revision"], 0)
        control.command({"action": "configure_grasp", "values": {"layout": "desk", "size_mm": 65}})
        self.assertEqual(control.grasp_request()["contact_revision"], 1)
        control.command({"action": "configure_grasp", "values": {"drag": [.2, .01, -.04], "angle_deg": 35}})
        self.assertEqual(control.grasp_request()["contact_revision"], 1)
        self.assertEqual(control.grasp_request()["size_mm"], 65)
        self.assertEqual(control.grasp_request()["angle_deg"], 35)
        with self.assertRaises(ValueError):
            control.command({"action": "configure_grasp", "values": {"kind": "bad", "size_mm": 90}})
        self.assertEqual(control.grasp_request()["size_mm"], 65)
        control.command({"action": "reset_scene"})
        self.assertIsNone(control.grasp_request()["drag"])
        self.assertEqual(control.grasp_request()["angle_deg"], 0.)
        self.assertEqual(control.grasp_request()["contact_revision"], 2)
        self.assertEqual(control.scene_request()[1], 1)

    def test_settings_and_presets(self):
        from ve33_local_grasp import DEFAULT_GRASP, GRASP_KINDS, grasp_settings

        self.assertEqual(len(GRASP_KINDS), 6)
        self.assertEqual(DEFAULT_GRASP["kind"], "ball_soft")
        self.assertEqual(DEFAULT_GRASP["size_mm"], 45)
        self.assertEqual(grasp_settings(DEFAULT_GRASP, {"kind": "cylinder_hard"})["hardness"], .92)
        for patch in ({"size_mm": 19}, {"hardness": float("nan")}, {"locked": 1},
                      {"kind": "other"}, {"drag": [0., 0., -100.]}):
            with self.assertRaises(ValueError):
                grasp_settings(DEFAULT_GRASP, patch)

    def test_official_prefabs_load_step_and_switch_in_one_scene(self):
        import superdex.physics as sdp
        from ve33_grasp_assets import GRASP_ASSETS, render_model_path
        from ve33_local_grasp import DEFAULT_GRASP, UPRIGHT_WRIST_POSE, LocalGrasp, grasp_settings

        self.assertEqual(len(GRASP_ASSETS), 17)
        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene("官方抓握资产验证")
            scene.set_gravity([0., 0., 0.])
            wrist = Rotation.from_quat(UPRIGHT_WRIST_POSE[3:])
            open_surface = wrist.inv().apply([[.11, -.075, .045]])
            env = LocalGrasp(sdp, scene, DEFAULT_GRASP, open_surface=open_surface)
            settings = DEFAULT_GRASP
            for kind, asset in GRASP_ASSETS.items():
                with self.subTest(kind=kind):
                    settings = grasp_settings(settings, {"kind": kind, "layout": "palm"})
                    if kind in ("official_duck_lamp", "official_quest_cube"):
                        settings = grasp_settings(settings, {"locked": True})
                    self.assertEqual(settings["layout"], "desk" if asset.desk_only else "palm")
                    self.assertTrue(env.configure(settings))
                    if kind == "official_shape_box":
                        front = max(part.get_aabb_world().max[1] for _, part in env.contact_actors)
                        self.assertLess(front, -.075)
                    env.step(1 / 240)
                    scene.step(1 / 240)
                    if settings["locked"]:
                        self.assertGreater(scene.get_num_constraints(), 0)
                    payload = env.packet()
                    self.assertEqual(payload["kind"], kind)
                    self.assertTrue(payload["asset"])
                    self.assertTrue(env.contact_actors)
                    if "vertices" in payload:
                        self.assertEqual(kind, "official_duck_lamp")
                        self.assertTrue(np.isfinite(payload["vertices"]).all())
                        self.assertTrue(payload["indices"])
                    else:
                        self.assertEqual(len(payload["parts"]), len(env.contact_actors))
                        self.assertTrue(all(render_model_path(part["model"]).is_file() for part in payload["parts"]))
                        self.assertTrue(all(np.isfinite(part["pose"]).all() for part in payload["parts"]))
            env.configure(DEFAULT_GRASP)
            self.assertEqual(scene.get_num_actors(), 1)
            self.assertEqual(scene.get_num_constraints(), 0)
        finally:
            sdp.shutdown()

    def test_scene_can_switch_all_objects_and_recover_soft_ball(self):
        import superdex.physics as sdp
        from ve33_local_grasp import DEFAULT_GRASP, GRASP_KINDS, LocalGrasp, grasp_settings

        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene("本地抓握验证")
            scene.set_gravity([0., 0., 0.])
            settings = grasp_settings(DEFAULT_GRASP, {"locked": False})
            env = LocalGrasp(sdp, scene, settings)
            scene.step(.0025)
            mesh = env.packet()
            self.assertTrue(mesh["vertices"])
            actor = env.contact_actors[0][1]
            actor.register_query(sdp.QueryType.NODE_POSITIONS)
            scene.step(.0025)
            neutral = np.asarray(actor.get_node_positions_local()).reshape(-1, 3).copy()
            compressed = neutral * [.75, 1.10, 1.10]
            actor.set_node_positions_local(compressed.ravel().tolist())
            for _ in range(200):
                scene.step(.0025)
            recovered = np.asarray(actor.get_node_positions_local()).reshape(-1, 3)
            self.assertAlmostEqual(np.ptp(neutral[:, 0]), .045, delta=.0002)
            self.assertAlmostEqual(np.ptp(recovered[:, 0]), .045, delta=.001)

            for kind in GRASP_KINDS:
                settings = grasp_settings(settings, {"kind": kind, "size_mm": 60, "locked": True})
                env.configure(settings)
                scene.step(.0025)
                payload = env.packet()
                self.assertEqual(payload["kind"], kind)
                self.assertEqual(payload["size_mm"], 60)
                self.assertEqual(bool(payload.get("vertices")), kind == "ball_soft")
                self.assertEqual(scene.get_num_actors(), 1)
            old = env.packet()
            np.testing.assert_allclose(Rotation.from_quat(old["pose"][3:]).apply([0, 0, 1]),
                                       [0, 1, 0], atol=1e-6)
            env.configure(grasp_settings(settings, {"layout": "desk", "angle_deg": 45}))
            scene.step(.0025)
            desk_pose = env.packet()["pose"]
            self.assertNotEqual(desk_pose[:3], old["pose"][:3])
            np.testing.assert_allclose(Rotation.from_quat(desk_pose[3:]).apply([0, 0, 1]),
                                       [0, 0, 1], atol=1e-6)
            env.configure(settings, reset=True)
            np.testing.assert_allclose(env.packet()["pose"], old["pose"], atol=.0002)
        finally:
            sdp.shutdown()


if __name__ == "__main__":
    unittest.main()
