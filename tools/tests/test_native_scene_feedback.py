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
