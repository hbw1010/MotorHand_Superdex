"""画面只能显示与最后一次物理解算对应的控制样本。"""
from pathlib import Path
import json
import socket
import sys
import time
import unittest
from unittest.mock import patch
from urllib.request import urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ve33_g4_view import aligned_feedback_display
from ve33_quest import WristBridge, wrist_server


class SceneFeedbackTest(unittest.TestCase):
    def test_http_pairs_feedback_outside_physics_scene_lock(self):
        bridge = WristBridge()
        bridge.publish_scene({"version": 1, "frame": 1, "physics_frame_ns": 80, "motor_feedback": None})
        def feedback(frame):
            # 即使控制样本尚未生成，场景发布者也可以发布下一张快照。
            self.assertTrue(bridge.lock.acquire(blocking=False))
            bridge.lock.release()
            return {"contact_frame_ns": frame["physics_frame_ns"], "expires_ns": time.monotonic_ns()+20_000_000}
        with socket.socket() as port:
            port.bind(("127.0.0.1", 0))
            address = port.getsockname()[1]
        with wrist_server(bridge, address, scene_feedback=feedback):
            with urlopen(f"http://127.0.0.1:{address}/api/v1/scene", timeout=2) as reply:
                frame = json.load(reply)
        self.assertEqual(frame["physics_frame_ns"], frame["motor_feedback"]["contact_frame_ns"])
        self.assertGreater(frame["motor_feedback"]["remaining_ms"], 0.)
        self.assertIsNone(bridge.scene["motor_feedback"])

    def test_old_force_in_same_revision_cannot_be_drawn_on_new_pose(self):
        frame = {"physics_frame_ns": 80, "grasp_object": {"contact_revision": 5}}
        def read(stamp, revision):
            return [100, 0, *([1.] * 12), *([.3] * 12), 5, 200, 70]
        with patch("ve33_g4_view.time.monotonic_ns", return_value=100):
            self.assertIsNone(aligned_feedback_display(read, frame, False, wait_s=0))

    def test_http_can_pair_a_new_control_sample_without_waiting_in_the_solver(self):
        frame = {"physics_frame_ns": 80, "grasp_object": {"contact_revision": 5}}
        samples = iter(([], [100, 1, *([2.] * 12), *([.5] * 12), 5, 200, 80]))
        def read(stamp, revision):
            self.assertEqual((stamp, revision), (80, 5))
            return next(samples)
        with patch("ve33_g4_view.time.monotonic_ns", return_value=100):
            result = aligned_feedback_display(read, frame, True, wait_s=.05)
        self.assertEqual((result["contact_frame_ns"], result["nominal_nm"][4]), (80, 2.))
        self.assertEqual(len(result["current_a"]), 12)

    def test_quest_scene_revision_pairs_without_grasp_object(self):
        frame = {"physics_frame_ns": 80, "scene_revision": 3}
        def read(stamp, revision):
            self.assertEqual((stamp, revision), (80, 3))
            return [100, 1, *([1.5] * 12), *([.2] * 12), 3, 200, 80]
        with patch("ve33_g4_view.time.monotonic_ns", return_value=100):
            result = aligned_feedback_display(read, frame, True, wait_s=0)
        self.assertEqual((result["contact_frame_ns"], result["nominal_nm"][0]), (80, 1.5))

    def test_quest_tabletop_defaults_to_cpp_native_control_end_to_end(self):
        import os
        import pty
        import subprocess
        from tempfile import TemporaryDirectory
        from urllib.error import URLError
        from urllib.request import Request
        from motorhand.feedback import MAPPING
        from motorhand.mapping import GloveMapping
        from ve33_g4_view import mapping_identity

        root = Path(__file__).resolve().parents[2]
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        base = f"http://127.0.0.1:{port}/api/v1/"
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        zero_file = Path(directory.name) / "zero.json"
        zero_file.write_text(json.dumps({
            "hand": "right",
            "zero_deg": {str(m.motor_id): 0. for m in mapping.motors},
            "axes": mapping_identity(axes),
        }))

        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        stop_pty = __import__("threading").Event()
        wire = []
        import threading
        def fake_glove():
            import binascii, select, struct
            buf = bytearray()
            while not stop_pty.is_set():
                if not select.select([master], [], [], .02)[0]:
                    continue
                buf.extend(os.read(master, 65536))
                while len(buf) >= 32:
                    pkt = bytes(buf[:32])
                    del buf[:32]
                    if pkt[:4] != bytes.fromhex("a55a0102"):
                        continue
                    bus, mid = pkt[6], int.from_bytes(pkt[8:10], "little")
                    if pkt[7] == 8:
                        wire.append(mid)
                    reply = bytearray(32)
                    reply[:4] = bytes.fromhex("a55a0190")
                    struct.pack_into("<BBH", reply, 6, bus, 6, mid)
                    reply[16:22] = struct.pack(">BBf", 0xa0, 1, 0.)
                    struct.pack_into("<H", reply, 30, binascii.crc_hqx(reply[:30], 65535))
                    os.write(master, reply)
        thread = threading.Thread(target=fake_glove, daemon=True)
        thread.start()
        self.addCleanup(lambda: (stop_pty.set(), thread.join(timeout=2)))

        process = subprocess.Popen([
            sys.executable, str(root / "tools/ve33_g4_view.py"),
            "--tabletop", "--headset", "--port", os.ttyname(slave), "--zero-file", str(zero_file),
            "--feedback-profile", "nominal-fingers", "--preview", "--quest-port", str(port),
        ], cwd=root, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
           env={**os.environ, "OPENBLAS_NUM_THREADS": "1", "XDG_STATE_HOME": directory.name})

        def stop():
            if process.poll() is None:
                process.send_signal(2)
            try:
                process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
        self.addCleanup(stop)

        def api(path, data=None):
            req = Request(base + path, json.dumps(data).encode() if data else None,
                          {"Content-Type": "application/json"})
            with urlopen(req, timeout=2) as reply:
                return reply.status if data else json.load(reply)

        state = None
        for _ in range(150):
            if process.poll() is not None:
                self.fail(process.stderr.read())
            try:
                state = api("control")
                if state.get("online") == 12 and state.get("calibrated") and state.get("contact_frame_ns"):
                    break
            except URLError:
                pass
            time.sleep(.06)
        else:
            self.fail(f"Quest C++ 模式未就绪: {state}")

        self.assertEqual(state["control_backend"], "cpp")
        self.assertEqual(state["grasp_backend"], "cpp")
        self.assertFalse(state["fixed_wrist"])
        self.assertTrue(state["can_align"])
        self.assertTrue(state["can_feedback"])
        self.assertTrue(state["transport_online"])

        # 上报 Quest 腕部定位并验证 scene_feedback 与 wrist_ms 对齐。
        api("wrist", {"space": "xr", "seq": 1, "pose": [0., .2, -.3, 0., 0., 0., 1.]})
        frame = None
        for _ in range(60):
            frame = api("scene")
            state = api("control")
            if frame.get("physics_frame_ns") and state.get("wrist_ms") is not None:
                break
            time.sleep(.05)
        self.assertEqual(frame["scene_kind"], "tabletop")
        self.assertEqual(frame["finger_control"], "physics")
        self.assertGreater(frame["physics_frame_ns"], 0)
        self.assertIsNotNone(state["wrist_ms"])
        expected_objects = {"table", "test_box", "block_blue", "block_yellow",
                            "block_green", "paper_cup", "sphere_ball", "peg_cylinder", "duck_lamp"}
        self.assertTrue(expected_objects.issubset(frame["objects"].keys()))

        # 验证场景复位更新 scene_revision 后 C++ 控制线程同步切版且保持在线。
        api("control", {"action": "reset_scene"})
        for _ in range(60):
            state = api("control")
            if state.get("scene_revision") == 1 and state.get("control_revision") == 1 and state.get("contact_frame_ns"):
                break
            time.sleep(.05)
        self.assertEqual(state["scene_revision"], 1)
        self.assertEqual(state["control_revision"], 1)
        self.assertIsNotNone(state["contact_frame_ns"])

        # 验证启用与关闭力反馈在 C++ NativeLoop 中正常工作。
        api("control", {"action": "enable"})
        for _ in range(60):
            state = api("control")
            if state.get("active"):
                break
            time.sleep(.05)
        self.assertTrue(state["active"])
        self.assertTrue(wire)
        api("control", {"action": "disable"})
        for _ in range(40):
            state = api("control")
            if not state.get("active"):
                break
            time.sleep(.05)
        self.assertFalse(state["active"])
